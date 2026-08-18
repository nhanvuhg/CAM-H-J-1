#include "unified_control_gui/robot_controller.hpp"
#include "dobot_msgs_v3/srv/continues.hpp"
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QRegularExpression>
#include <QSettings>
#include <QTcpSocket>
#include <QThread>
#include <QDateTime>
#include <QMetaObject>
#include <QProcess>
#include <QProcessEnvironment>
#include <QCoreApplication>
#include <QStandardPaths>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>

namespace {

// Workspace suy ra lúc chạy thay vì hardcode. Binary được cài ở
// <WS>/install/unified_control_gui/lib/unified_control_gui nên lùi 4 cấp là <WS>.
QString resolveWorkspaceRoot()
{
    const QString fromEnv =
        QProcessEnvironment::systemEnvironment().value("ROS2_WS");
    if (!fromEnv.isEmpty() && QDir(fromEnv).exists()) {
        return QDir(fromEnv).absolutePath();
    }

    QDir dir(QCoreApplication::applicationDirPath());
    bool climbed = true;
    for (int i = 0; i < 4 && climbed; ++i) climbed = dir.cdUp();
    if (climbed && (dir.exists("install") || dir.exists("src"))) {
        return dir.absolutePath();
    }

    return QDir::home().filePath("ros2_ws");
}

// Script restart được cài cạnh binary; vẫn giữ fallback sang cây nguồn để chạy
// được cả khi workspace chưa install.
QString resolveScriptPath(const QString &name)
{
    const QString ws = resolveWorkspaceRoot();
    const QStringList candidates = {
        QDir(QCoreApplication::applicationDirPath()).filePath(name),
        ws + "/install/unified_control_gui/lib/unified_control_gui/" + name,
        ws + "/src/unified_control_gui/scripts/" + name,
    };
    for (const QString &path : candidates) {
        if (QFile::exists(path)) return path;
    }
    return QString();
}

}  // namespace

RobotController::RobotController(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent)
    , node_(node)
{
    // Init joint angles to 6 zeros
    for (int i = 0; i < 6; i++) joint_angles_.append(0.0);
    for (int i = 0; i < 6; i++) cartesian_pose_.append(0.0);
    for (int i = 0; i < 5; i++) row_ready_.append(false);
    for (int i = 0; i < 10; i++) slot_ready_.append(false);

    // Create service clients
    enable_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/enable_system");
    start_system_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/start_system");
    emergency_stop_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/emergency_stop");
    pause_system_client_   = node_->create_client<std_srvs::srv::SetBool>("/robot/pause_system");  // NEW

    manual_mode_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/set_manual_mode");
    ai_mode_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/set_ai_mode");
    
    // Dobot jog service clients
    jog_client_ = node_->create_client<dobot_msgs_v3::srv::MoveJog>("/nova5/dobot_bringup/MoveJog");
    get_angle_client_ = node_->create_client<dobot_msgs_v3::srv::GetAngle>("/nova5/dobot_bringup/GetAngle");
    get_pose_client_ = node_->create_client<dobot_msgs_v3::srv::GetPose>("/nova5/dobot_bringup/GetPose");
    joint_movj_client_ = node_->create_client<dobot_msgs_v3::srv::JointMovJ>("/nova5/dobot_bringup/JointMovJ");
    movl_client_ = node_->create_client<dobot_msgs_v3::srv::MovL>("/nova5/dobot_bringup/MovL");
    servo_j_client_ = node_->create_client<dobot_msgs_v3::srv::ServoJ>("/nova5/dobot_bringup/ServoJ");
    servo_p_client_ = node_->create_client<dobot_msgs_v3::srv::ServoP>("/nova5/dobot_bringup/ServoP");
    do_client_ = node_->create_client<dobot_msgs_v3::srv::DO>("/nova5/dobot_bringup/DO");
    pause_client_ = node_->create_client<dobot_msgs_v3::srv::Pause>("/nova5/dobot_bringup/Pause");
    continue_client_ = node_->create_client<dobot_msgs_v3::srv::Continues>("/nova5/dobot_bringup/Continue");
    dobot_emergency_stop_client_ = node_->create_client<dobot_msgs_v3::srv::EmergencyStop>("/nova5/dobot_bringup/EmergencyStop");
    dobot_stop_script_client_ = node_->create_client<dobot_msgs_v3::srv::StopScript>("/nova5/dobot_bringup/StopScript");
    disable_robot_client_ = node_->create_client<dobot_msgs_v3::srv::DisableRobot>("/nova5/dobot_bringup/DisableRobot");
    dobot_enable_robot_client_ = node_->create_client<dobot_msgs_v3::srv::EnableRobot>("/nova5/dobot_bringup/EnableRobot");

    // [STOP-PRESERVE] Track gripper/picker state real-time tu gripper_festo_node
    gripper_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/robot/gripper_status", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (last_gripper_state_ == msg->data) return;
            last_gripper_state_ = msg->data;
            QMetaObject::invokeMethod(this, [this]() { emit gripperOnChanged(); }, Qt::QueuedConnection);
        });
    picker_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/robot/picker_status", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (last_picker_state_ == msg->data) return;
            last_picker_state_ = msg->data;
            QMetaObject::invokeMethod(this, [this]() { emit pickerOnChanged(); }, Qt::QueuedConnection);
        });
    cyl_loadcell_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/robot/cyl_loadcell_status", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (last_cyl_loadcell_state_ == msg->data) return;
            last_cyl_loadcell_state_ = msg->data;
            QMetaObject::invokeMethod(this, [this]() { emit cylLoadcellOnChanged(); }, Qt::QueuedConnection);
        });
    gripper_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/gripper_cmd", 10);
    picker_cmd_pub_  = node_->create_publisher<std_msgs::msg::Bool>("/robot/picker_cmd", 10);
    cyl_loadcell_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/cyl_loadcell_cmd", 10);
    clear_error_client_ = node_->create_client<dobot_msgs_v3::srv::ClearError>("/nova5/dobot_bringup/ClearError");
    reset_robot_client_ = node_->create_client<dobot_msgs_v3::srv::ResetRobot>("/nova5/dobot_bringup/ResetRobot");
    speed_factor_client_ = node_->create_client<dobot_msgs_v3::srv::SpeedFactor>("/nova5/dobot_bringup/SpeedFactor");
    get_error_id_client_ = node_->create_client<dobot_msgs_v3::srv::GetErrorID>("/nova5/dobot_bringup/GetErrorID");
    reset_state_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/reset_state");
    soft_stop_to_manual_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/soft_stop_to_manual");

    // Create publishers
    camera_select_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_camera", 10);
    command_row_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_row", 10);
    command_slot_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/command_slot", 10);
    goto_state_pub_ = node_->create_publisher<std_msgs::msg::String>("/robot/goto_state", 10);
    set_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/set_mode", 10);
    feed_chamber_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/revpi/feed_chamber", 10);
    fill_done_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/revpi/fill_done", 10);
    input_tray_ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cartridge_providesystem/new_tray_loaded", 10);
    output_tray_ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cartridge_providesystem/new_trayoutput_loaded", 10);
    scale_result_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/scale/result", 10);
    speed_ratio_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robot/speed_ratio", rclcpp::QoS(10).reliable().transient_local());
    system_start_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/system/start_button", 10);  // shared with cartridge
    system_pause_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/system/pause_button", 10);  // sync sang cartridge
    system_resume_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/system/resume_button", 10); // sync sang cartridge
    ignore_scale_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/ignore_scale", rclcpp::QoS(10).reliable().transient_local());

    // Duplicate sub gripper/picker đã xoá — đã khởi tạo 1 lần ở phía trên (L43-48).
    // Tạo lại 2 lần sẽ overwrite SharedPtr → sub đầu bị destruct, callback đầu chết âm thầm.

    // Create subscribers
    system_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/system_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                // robot_logic_node phat lai trang thai moi 1 s de GUI khoi dong
                // tre van bat duoc gia tri hien tai. Chi emit khi gia tri that
                // su doi: handler onSystemStatusChanged trong Main.qml goi
                // confirmEmptyBufferPopup.close(), nen emit lap lai se dong mat
                // bang xac nhan ngay khi operator dang doc no.
                const QString value = QString::fromStdString(msg->data);
                if (system_status_ == value)
                    return;
                system_status_ = value;
                emit systemStatusChanged();
            }, Qt::QueuedConnection);
        });
    
    error_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/error", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                error_message_ = QString::fromStdString(msg->data);
                emit errorMessageChanged();
            }, Qt::QueuedConnection);
        });
    
    // Latched (transient_local) — GUI bat sau vision_decision_node van nhan
    // duoc trang thai ROI thay vi cho toi lan publish sau.
    roi_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/vision/roi_status", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString next = QString::fromStdString(msg->data);
                if (next != roi_error_) {
                    roi_error_ = next;
                    emit roiErrorChanged();
                }
            }, Qt::QueuedConnection);
        });

    selected_row_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/selected_input_row", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                selected_row_ = msg->data;
                emit selectedRowChanged();
            }, Qt::QueuedConnection);
        });
    
    selected_slot_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/selected_output_slot", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                selected_slot_ = msg->data;
                emit selectedSlotChanged();
            }, Qt::QueuedConnection);
        });

    row_status_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/vision/input_tray/row_status", 10,
        [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QVariantList next;
                for (int i = 0; i < 5; ++i) {
                    next.append(i < (int)msg->data.size() && msg->data[i] == 1);
                }
                if (next != row_ready_) {
                    row_ready_ = next;
                    emit rowReadyChanged();
                }
            }, Qt::QueuedConnection);
        });

    slot_status_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/vision/output_tray/slot_status", 10,
        [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QVariantList next;
                // slot_status: 0=empty (ready to place), 1=occupied (blank)
                for (int i = 0; i < 10; ++i) {
                    next.append(i < (int)msg->data.size() && msg->data[i] == 0);
                }
                if (next != slot_ready_) {
                    slot_ready_ = next;
                    emit slotReadyChanged();
                }
            }, Qt::QueuedConnection);
        });
    
    system_uptime_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/system_uptime", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                system_uptime_ = QString::fromStdString(msg->data);
                emit systemUptimeChanged();
            }, Qt::QueuedConnection);
        });

    in_ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/cartridge_providesystem/new_tray_loaded", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                if (in_ready_ != msg->data) {
                    in_ready_ = msg->data;
                    emit inReadyChanged();
                }
            }, Qt::QueuedConnection);
        });

    out_ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/cartridge_providesystem/new_trayoutput_loaded", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                if (out_ready_ != msg->data) {
                    out_ready_ = msg->data;
                    emit outReadyChanged();
                }
            }, Qt::QueuedConnection);
        });

    tray_count_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/tray_count", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                tray_count_ = msg->data;
                emit trayCountChanged();
            }, Qt::QueuedConnection);
        });
    
    // Subscribe to hardware speed factor readback from Dobot feedback node
    hw_speed_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/robot/hw_speed_factor", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                if (hw_speed_ratio_ != msg->data) {
                    hw_speed_ratio_ = msg->data;
                    emit hwSpeedRatioChanged();
                }
            }, Qt::QueuedConnection);
        });

    // Realtime joint angles from feedback streaming (Dobot port 30004, ~100Hz)
    // Replaces GetAngle service polling — eliminates 500ms display lag.
    // feedback.py publishes JointState in radians; convert to degrees for GUI.
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/nova5/joint_states_robot", sensor_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            QVariantList angles;
            constexpr double RAD2DEG = 57.29577951308232;
            for (double p : msg->position) angles.append(p * RAD2DEG);
            while (angles.size() < 6) angles.append(0.0);
            
            std::lock_guard<std::mutex> lock(status_mutex_);
            next_joint_angles_ = angles;
            has_new_joints_ = true;
        });

    // Realtime TCP pose from feedback streaming — already in mm/deg.
    tool_vector_sub_ = node_->create_subscription<dobot_msgs_v3::msg::ToolVectorActual>(
        "/nova5/dobot_msgs_v3/msg/ToolVectorActual", sensor_qos,
        [this](const dobot_msgs_v3::msg::ToolVectorActual::SharedPtr msg) {
            QVariantList pose;
            pose.append(msg->x); pose.append(msg->y); pose.append(msg->z);
            pose.append(msg->rx); pose.append(msg->ry); pose.append(msg->rz);
            
            std::lock_guard<std::mutex> lock(status_mutex_);
            next_cartesian_pose_ = pose;
            has_new_pose_ = true;
        });

    // High-frequency GUI Update Timer (20Hz / 50ms)
    // Decouples high-frequency ROS streaming (~100Hz) from Qt Main Thread rendering.
    // Prevents GUI event loop starvation, drastically reduces CPU usage, and guarantees thread safety.
    QTimer* gui_update_timer = new QTimer(this);
    connect(gui_update_timer, &QTimer::timeout, this, [this]() {
        bool update_joints = false;
        bool update_pose = false;
        QVariantList joints_to_set;
        QVariantList pose_to_set;

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            if (has_new_joints_) {
                joints_to_set = next_joint_angles_;
                has_new_joints_ = false;
                update_joints = true;
            }
            if (has_new_pose_) {
                pose_to_set = next_cartesian_pose_;
                has_new_pose_ = false;
                update_pose = true;
            }
        }

        if (update_joints) {
            joint_angles_ = joints_to_set;
            emit jointAnglesChanged();
        }
        if (update_pose) {
            cartesian_pose_ = pose_to_set;
            emit cartesianPoseChanged();
        }
    });
    gui_update_timer->start(50); // 50ms (20Hz)

    qDebug() << "RobotController initialized";

    // Load persisted speed ratio
    QSettings settings("RobotControl", "ManualMode");
    int savedSpeed = settings.value("speedRatio", 100).toInt();
    speed_ratio_ = qBound(1, savedSpeed, 100);
    qDebug() << "Loaded speed ratio:" << speed_ratio_;

    // Publish initial speed ratio so motion_executor AND Dobot hardware get it on startup
    // Use a single-shot timer to ensure services/publishers are ready before sending
    QTimer::singleShot(3000, this, [this]() {
        // 1. Sync to motion_executor (for SpeedJ/SpeedL before each move)
        std_msgs::msg::Int32 msg;
        msg.data = speed_ratio_;
        speed_ratio_pub_->publish(msg);
        qDebug() << "[STARTUP] Published initial speed ratio:" << speed_ratio_;

        // Sync ignore_scale default (false) to robot_logic on GUI start
        {
            auto is_msg = std_msgs::msg::Bool();
            is_msg.data = ignore_scale_;  // default false
            if (ignore_scale_pub_) ignore_scale_pub_->publish(is_msg);
            qDebug() << "[STARTUP] Published initial ignore_scale:" << ignore_scale_;
        }

        // 2. Sync to Dobot hardware (SpeedFactor — persists until power cycle)
        if (speed_factor_client_ && speed_factor_client_->service_is_ready()) {
            auto req = std::make_shared<dobot_msgs_v3::srv::SpeedFactor::Request>();
            req->ratio = speed_ratio_;
            speed_factor_client_->async_send_request(req,
                [this](rclcpp::Client<dobot_msgs_v3::srv::SpeedFactor>::SharedFuture f) {
                    try {
                        auto r = f.get();
                        qDebug() << "[STARTUP] SpeedFactor synced to hardware:" << speed_ratio_ << "% (res:" << r->res << ")";
                    } catch (...) {
                        qWarning() << "[STARTUP] SpeedFactor sync failed — hardware may use default speed";
                    }
                });
        } else {
            qWarning() << "[STARTUP] SpeedFactor service not ready — hardware speed may differ from GUI";
        }
    });

    // Position/pose stream in at ~100Hz via joint_state_sub_ + tool_vector_sub_.
    // Only error ID still needs service polling — slower cadence is fine.
    poll_timer_ = new QTimer(this);
    connect(poll_timer_, &QTimer::timeout, this, &RobotController::pollRobotState);
    poll_timer_->start(1000);
}

void RobotController::callServiceAsync(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value)
{
    if (!client->service_is_ready()) {
        qWarning() << "Service not available yet, sending anyway...";
    }
    
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = value;
    client->async_send_request(request);
    emit serviceCallResult(true, tr("Request sent"));
}

QString RobotController::captureScreenshot()
{
    if (screenshot_process_) {
        const QString message = tr("Screenshot already in progress");
        qWarning() << message;
        emit serviceCallResult(false, message);
        return QString();
    }

    const QString outputDir = QDir::home().filePath("PicturesGUI");
    if (!QDir().mkpath(outputDir)) {
        const QString message = tr("Cannot create screenshot directory: %1").arg(outputDir);
        qWarning() << message;
        emit serviceCallResult(false, message);
        return QString();
    }

    const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
    const QString outputPath = outputDir + "/screenshot_" + timestamp + ".png";

    auto* process = new QProcess(this);
    screenshot_process_ = process;

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    if (!env.contains("DISPLAY") || env.value("DISPLAY").isEmpty()) {
        env.insert("DISPLAY", ":0");
    }
    if (!env.contains("XAUTHORITY") || env.value("XAUTHORITY").isEmpty()) {
        const QString runtimeAuth = QDir(
            QStandardPaths::writableLocation(QStandardPaths::RuntimeLocation))
            .filePath("gdm/Xauthority");
        const QString homeAuth = QDir::home().filePath(".Xauthority");
        env.insert("XAUTHORITY", QFile::exists(runtimeAuth) ? runtimeAuth : homeAuth);
    }
    process->setProcessEnvironment(env);

    QString captureProgram;
    QStringList captureArguments;
    // scrot uses a small direct X11 capture path. Prefer it over
    // gnome-screenshot, which asks the compositor/session to capture and has
    // triggered nvgpu MMU/SM faults on this Jetson under concurrent Qt Quick,
    // camera and TensorRT rendering load.
    if (QFile::exists("/usr/bin/scrot")) {
        captureProgram = "/usr/bin/scrot";
        captureArguments << outputPath;
    } else {
        // Do not fall back to gnome-screenshot on Jetson: that path caused a
        // reproducible display/GPU reset. Failing closed is safer.
        const QString message = tr("Screenshot failed: scrot is not installed");
        qWarning() << message;
        emit serviceCallResult(false, message);
        screenshot_process_ = nullptr;
        process->deleteLater();
        return QString();
    }

    connect(process, &QProcess::errorOccurred, this,
        [this, process, outputPath](QProcess::ProcessError error) {
            if (screenshot_process_ != process) return;
            screenshot_process_ = nullptr;
            QFile::remove(outputPath);
            const QString message = tr("Screenshot process error (%1): %2")
                .arg(static_cast<int>(error)).arg(process->errorString());
            qWarning() << message;
            emit serviceCallResult(false, message);
            process->deleteLater();
        });

    connect(process,
        qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
        [this, process, outputPath](int exitCode, QProcess::ExitStatus exitStatus) {
            if (screenshot_process_ != process) return;
            screenshot_process_ = nullptr;
            const QString err = QString::fromLocal8Bit(
                process->readAllStandardError()).trimmed();
            const QFileInfo outputInfo(outputPath);
            const bool ok = exitStatus == QProcess::NormalExit
                         && exitCode == 0
                         && outputInfo.isFile()
                         && outputInfo.size() > 0;
            if (ok) {
                qDebug() << "Screenshot saved asynchronously:" << outputPath;
                emit serviceCallResult(true, tr("Screenshot saved: %1").arg(outputPath));
            } else {
                QFile::remove(outputPath);
                const QString message = err.isEmpty()
                    ? tr("Screenshot failed (exit=%1)").arg(exitCode)
                    : err;
                qWarning() << message;
                emit serviceCallResult(false, message);
            }
            process->deleteLater();
        });

    // A failed display/session capture must never leave a process or busy flag
    // hanging indefinitely. kill() is asynchronous and does not block Qt.
    QTimer::singleShot(10000, this, [this, process, outputPath]() {
        if (screenshot_process_ != process) return;
        screenshot_process_ = nullptr;
        process->kill();
        QFile::remove(outputPath);
        const QString message = tr("Screenshot timed out after 10 seconds");
        qWarning() << message;
        emit serviceCallResult(false, message);
        process->deleteLater();
    });

    process->start(captureProgram, captureArguments, QIODevice::ReadOnly);
    qDebug() << "Screenshot scheduled asynchronously with" << captureProgram
             << "->" << outputPath;
    return outputPath;
}

QString RobotController::restartSystemNodes()
{
    const QString workspace = resolveWorkspaceRoot();
    const QString scriptPath = resolveScriptPath("restart_system_nodes.sh");

    if (scriptPath.isEmpty()) {
        const QString message = tr("Restart script not found");
        qWarning() << message << workspace;
        emit serviceCallResult(false, message);
        return QString();
    }

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    if (!env.contains("DISPLAY") || env.value("DISPLAY").isEmpty()) {
        env.insert("DISPLAY", ":0");
    }
    env.insert("ROS2_WS", workspace);

    // Phải dùng overload non-static: bản static bỏ qua process environment nên
    // ROS2_WS/DISPLAY set ở trên sẽ không tới được script.
    QProcess process;
    process.setProgram("/bin/bash");
    process.setArguments(QStringList() << scriptPath);
    process.setProcessEnvironment(env);
    process.setWorkingDirectory(workspace);

    qint64 pid = 0;
    const bool started = process.startDetached(&pid);
    if (!started) {
        const QString message = tr("Cannot start restart script");
        qWarning() << message << scriptPath;
        emit serviceCallResult(false, message);
        return QString();
    }

    const QString message = tr("Restarting system nodes (PID %1)").arg(pid);
    qDebug() << message;
    emit serviceCallResult(true, message);
    return message;
}

QString RobotController::restartGui()
{
    const bool managedByLauncher =
        QProcessEnvironment::systemEnvironment().value("UNIFIED_GUI_MANAGED_RESTART") == "1";

    if (managedByLauncher) {
        QFile flag("/tmp/unified_gui_restart_requested");
        if (flag.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            flag.write("1\n");
            flag.close();
        }

        const QString message = tr("Restarting GUI");
        qDebug() << message;
        emit serviceCallResult(true, message);
        QTimer::singleShot(150, QCoreApplication::instance(), []() {
            QCoreApplication::exit(42);
        });
        return message;
    }

    const QString workspace = resolveWorkspaceRoot();
    const QString scriptPath = resolveScriptPath("restart_gui.sh");
    if (scriptPath.isEmpty()) {
        const QString message = tr("Restart GUI script not found");
        qWarning() << message << workspace;
        emit serviceCallResult(false, message);
        return QString();
    }

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    if (!env.contains("DISPLAY") || env.value("DISPLAY").isEmpty()) {
        env.insert("DISPLAY", ":0");
    }
    env.insert("ROS2_WS", workspace);

    QProcess process;
    process.setProgram("/bin/bash");
    process.setArguments(QStringList() << scriptPath
                                       << QString::number(QCoreApplication::applicationPid()));
    process.setProcessEnvironment(env);
    process.setWorkingDirectory(workspace);

    qint64 pid = 0;
    const bool started = process.startDetached(&pid);
    if (!started) {
        const QString message = tr("Cannot start Restart GUI script");
        qWarning() << message << scriptPath;
        emit serviceCallResult(false, message);
        return QString();
    }

    const QString message = tr("Restarting GUI (PID %1)").arg(pid);
    qDebug() << message;
    emit serviceCallResult(true, message);
    return message;
}

void RobotController::enableSystem(bool enable)
{
    qDebug() << "Enable system:" << enable;
    callServiceAsync(enable_client_, enable);

    if (enable) {
        // /robot/enable_system already requests ClearError -> EnableRobot ->
        // Continue. Those Dobot calls are asynchronous, however, so Continue
        // can occasionally arrive before EnableRobot has fully left the STOP
        // state. Retry once after the enable sequence has settled. Both the
        // CameraPage and Manual Robot Control ENABLE buttons use this method.
        //
        // STOP clears the active command with StopScript + ResetRobot first,
        // therefore this delayed Continue only releases the Cartesian/ServoP
        // channel; it cannot resume an old queued target.
        QTimer::singleShot(800, this, [this]() {
            if (!continue_client_ || !continue_client_->service_is_ready()) {
                qWarning() << "Continue service not ready after ENABLE";
                return;
            }

            auto request = std::make_shared<dobot_msgs_v3::srv::Continues::Request>();
            continue_client_->async_send_request(
                request,
                [](rclcpp::Client<dobot_msgs_v3::srv::Continues>::SharedFuture future) {
                    try {
                        qDebug() << "Continue retry after ENABLE:" << future.get()->res;
                    } catch (const std::exception& e) {
                        qWarning() << "Continue retry after ENABLE failed:" << e.what();
                    }
                });
        });
    }
}

void RobotController::stopAndResetRobot()
{
    qDebug() << "Stop & Reset Robot";
    stopManualJogMotion();

    if (set_mode_pub_) {
        auto modeMsg = std_msgs::msg::Int32();
        modeMsg.data = 3; // 3 = MANUAL
        set_mode_pub_->publish(modeMsg);
    }

    // Reset UI-facing selection/ready state immediately so the GUI blanks out
    // as soon as STOP is pressed.
    if (selected_row_ != -1) {
        selected_row_ = -1;
        emit selectedRowChanged();
    }
    if (selected_slot_ != -1) {
        selected_slot_ = -1;
        emit selectedSlotChanged();
    }
    if (in_ready_) {
        in_ready_ = false;
        emit inReadyChanged();
    }
    if (out_ready_) {
        out_ready_ = false;
        emit outReadyChanged();
    }
    bool rowChanged = false;
    for (int i = 0; i < row_ready_.size(); ++i) {
        if (row_ready_[i].toBool()) {
            row_ready_[i] = false;
            rowChanged = true;
        }
    }
    if (rowChanged) emit rowReadyChanged();
    bool slotChanged = false;
    for (int i = 0; i < slot_ready_.size(); ++i) {
        if (slot_ready_[i].toBool()) {
            slot_ready_[i] = false;
            slotChanged = true;
        }
    }
    if (slotChanged) emit slotReadyChanged();

    // [STOP-PRESERVE] Snapshot picker+gripper state ngay TRUOC khi gui STOP
    // services. Tat ca STOP services khong duoc set DO1/DO2, nhung Dobot
    // ResetRobot mot so phien ban firmware co the reset DO outputs nhu side
    // effect → trigger gripper_festo node receive ResetEvent. De an toan,
    // sau khi EnableRobot xong, ta REPUBLISH state cu de force gripper_festo
    // restore CPX coil ve nguyen trang truoc STOP.
    const bool snapshot_gripper = last_gripper_state_;
    const bool snapshot_picker  = last_picker_state_;
    qDebug() << "  -> Snapshot gripper=" << snapshot_gripper
             << ", picker=" << snapshot_picker;

    // A normal STOP pauses first so a running MovL/MovJ decelerates smoothly,
    // then StopScript clears the queued command before ResetRobot recovers.

    // Step 1: Reset state machine → IDLE (stops auto mode, clears all flags)
    auto resetReq = std::make_shared<std_srvs::srv::SetBool::Request>();
    resetReq->data = true;
    if (reset_state_client_->service_is_ready()) {
        reset_state_client_->async_send_request(resetReq,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
                try { qDebug() << "Reset state:" << f.get()->message.c_str(); } catch (...) {}
            });
    } else {
        qWarning() << "/robot/reset_state not available (robot_logic_node not running?)";
    }

    // Step 2: Clear E-stop flag
    auto estopReq = std::make_shared<std_srvs::srv::SetBool::Request>();
    estopReq->data = false;
    if (emergency_stop_client_->service_is_ready()) {
        emergency_stop_client_->async_send_request(estopReq);
    }

    // Step 3: Clear the current motion command. Do not ClearError/Enable/Continue
    // here, otherwise a paused MovL/ServoP can resume toward the old target.
    auto resetMotion = [this, snapshot_gripper, snapshot_picker]() {
        auto stopScriptReq = std::make_shared<dobot_msgs_v3::srv::StopScript::Request>();
        if (dobot_stop_script_client_ && dobot_stop_script_client_->service_is_ready()) {
            dobot_stop_script_client_->async_send_request(stopScriptReq,
                [this, snapshot_gripper, snapshot_picker](rclcpp::Client<dobot_msgs_v3::srv::StopScript>::SharedFuture sf) {
                    try { qDebug() << "StopScript after Pause:" << sf.get()->res; } catch (...) {}
                    QMetaObject::invokeMethod(this, [this, snapshot_gripper, snapshot_picker]() {
                        QTimer::singleShot(100, this, [this, snapshot_gripper, snapshot_picker]() {
                            auto resetRobotReq = std::make_shared<dobot_msgs_v3::srv::ResetRobot::Request>();
                            if (reset_robot_client_ && reset_robot_client_->service_is_ready()) {
                                reset_robot_client_->async_send_request(resetRobotReq,
                                    [this, snapshot_gripper, snapshot_picker](rclcpp::Client<dobot_msgs_v3::srv::ResetRobot>::SharedFuture f) {
                                        try { qDebug() << "ResetRobot:" << f.get()->res; } catch (...) {}
                                        QMetaObject::invokeMethod(this, [this, snapshot_gripper, snapshot_picker]() {
                                            QTimer::singleShot(150, this, [this, snapshot_gripper, snapshot_picker]() {
                                                qDebug() << "Robot stopped and command queue cleared — press ENABLE for new commands";

                                                QTimer::singleShot(200, this, [this, snapshot_gripper, snapshot_picker]() {
                                                    if (gripper_cmd_pub_) {
                                                        auto gMsg = std_msgs::msg::Bool();
                                                        gMsg.data = snapshot_gripper;
                                                        gripper_cmd_pub_->publish(gMsg);
                                                    }
                                                    if (picker_cmd_pub_) {
                                                        auto pMsg = std_msgs::msg::Bool();
                                                        pMsg.data = snapshot_picker;
                                                        picker_cmd_pub_->publish(pMsg);
                                                    }
                                                    qDebug() << "  -> Restored gripper=" << snapshot_gripper
                                                             << ", picker=" << snapshot_picker;
                                                });
                                            });
                                        }, Qt::QueuedConnection);
                                    });
                            } else {
                                qWarning() << "ResetRobot service not ready after STOP";
                            }
                        });
                    }, Qt::QueuedConnection);
                });
        } else {
            qWarning() << "StopScript service not ready after Pause; reset may not clear queued motion";
            auto resetRobotReq = std::make_shared<dobot_msgs_v3::srv::ResetRobot::Request>();
            if (reset_robot_client_ && reset_robot_client_->service_is_ready()) {
                reset_robot_client_->async_send_request(resetRobotReq,
                    [this, snapshot_gripper, snapshot_picker](rclcpp::Client<dobot_msgs_v3::srv::ResetRobot>::SharedFuture f) {
                        try { qDebug() << "ResetRobot:" << f.get()->res; } catch (...) {}
                        QMetaObject::invokeMethod(this, [this, snapshot_gripper, snapshot_picker]() {
                            QTimer::singleShot(150, this, [this, snapshot_gripper, snapshot_picker]() {
                                qDebug() << "Robot paused and command queue cleared — press ENABLE to move again";

                                QTimer::singleShot(200, this, [this, snapshot_gripper, snapshot_picker]() {
                                    if (gripper_cmd_pub_) {
                                        auto gMsg = std_msgs::msg::Bool();
                                        gMsg.data = snapshot_gripper;
                                        gripper_cmd_pub_->publish(gMsg);
                                    }
                                    if (picker_cmd_pub_) {
                                        auto pMsg = std_msgs::msg::Bool();
                                        pMsg.data = snapshot_picker;
                                        picker_cmd_pub_->publish(pMsg);
                                    }
                                    qDebug() << "  -> Restored gripper=" << snapshot_gripper
                                             << ", picker=" << snapshot_picker;
                                });
                            });
                        }, Qt::QueuedConnection);
                    });
            } else {
                qWarning() << "ResetRobot service not ready after STOP";
            }
        }
    };

    // STOP KEEPS THE ROBOT POWERED (user choice: "dừng nhưng giữ nguồn").
    // No DisableRobot — just halt the motion and flush the queue via ResetRobot
    // (inside resetMotion: StopScript + ResetRobot, NO Pause). The robot stays
    // at RobotMode 5 (ENABLE), so JOG (cartesian/joint) and SEND work IMMEDIATELY
    // after STOP without a separate ENABLE press. Powering off (DisableRobot)
    // left the robot at Mode 4 (DISABLED) which blocked Cartesian jog until an
    // explicit ENABLE — that regression is what this removes.
    resetMotion();
}

void RobotController::softStopAndManual()
{
    qDebug() << "Soft Stop & Switch to MANUAL (keep state + CPX)";
    stopManualJogMotion();

    // STOP from MANUAL follows this path (instead of stopAndResetRobot()).
    // Clear the momentary tray-ready indicators here as well so a subsequent
    // ENABLE cannot leave IN_READY / OUT_READY visually latched on the GUI.
    if (in_ready_) {
        in_ready_ = false;
        emit inReadyChanged();
    }
    if (out_ready_) {
        out_ready_ = false;
        emit outReadyChanged();
    }

    // ResetRobot clears the motion buffer without leaving ServoP paused.
    auto resetRobotReq = std::make_shared<dobot_msgs_v3::srv::ResetRobot::Request>();
    if (reset_robot_client_ && reset_robot_client_->service_is_ready()) {
        reset_robot_client_->async_send_request(resetRobotReq);
    }

    // 2. Gọi /robot/soft_stop_to_manual: cancel motion action + manual mode, giữ state
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    if (soft_stop_to_manual_client_ && soft_stop_to_manual_client_->service_is_ready()) {
        soft_stop_to_manual_client_->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
                try { qDebug() << "Soft stop:" << f.get()->message.c_str(); } catch (...) {}
            });
    } else {
        qWarning() << "/robot/soft_stop_to_manual not available";
    }

    // 3. ClearError sau 300ms (drive có thể vào error mode khi pause giữa motion)
    QTimer::singleShot(300, this, [this]() {
        auto clearReq = std::make_shared<dobot_msgs_v3::srv::ClearError::Request>();
        if (clear_error_client_ && clear_error_client_->service_is_ready()) {
            clear_error_client_->async_send_request(clearReq);
        }
    });
}

void RobotController::startSystem(bool start)
{
    qDebug() << "Start system:" << start;
    // Publish one synchronized START edge only. robot_logic_node also subscribes
    // to /system/start_button and performs ClearError -> EnableRobot -> HOME
    // there. Calling /robot/start_system first races the state machine into
    // INIT_LOAD before HOME can be sent.
    if (start) {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        system_start_pub_->publish(msg);
        qDebug() << "Published /system/start_button = true (robot + cartridge trigger)";
    } else {
        callServiceAsync(start_system_client_, false);
    }
}

void RobotController::emergencyStop(bool stop)
{
    qDebug() << "Emergency stop:" << stop;
    if (stop) {
        stopManualJogMotion();

        if (set_mode_pub_) {
            auto modeMsg = std_msgs::msg::Int32();
            modeMsg.data = 3; // 3 = MANUAL
            set_mode_pub_->publish(modeMsg);
        }

        if (dobot_emergency_stop_client_ && dobot_emergency_stop_client_->service_is_ready()) {
            auto estopReq = std::make_shared<dobot_msgs_v3::srv::EmergencyStop::Request>();
            dobot_emergency_stop_client_->async_send_request(estopReq,
                [](rclcpp::Client<dobot_msgs_v3::srv::EmergencyStop>::SharedFuture ef) {
                    try { qDebug() << "Dobot EmergencyStop:" << ef.get()->res; } catch (...) {}
                });
        } else {
            qWarning() << "Dobot EmergencyStop service not ready";
        }

        if (dobot_stop_script_client_ && dobot_stop_script_client_->service_is_ready()) {
            auto stopScriptReq = std::make_shared<dobot_msgs_v3::srv::StopScript::Request>();
            dobot_stop_script_client_->async_send_request(stopScriptReq);
        }

        if (pause_client_ && pause_client_->service_is_ready()) {
            auto pauseReq = std::make_shared<dobot_msgs_v3::srv::Pause::Request>();
            pause_client_->async_send_request(pauseReq);
        }

        if (disable_robot_client_ && disable_robot_client_->service_is_ready()) {
            auto disableReq = std::make_shared<dobot_msgs_v3::srv::DisableRobot::Request>();
            disable_robot_client_->async_send_request(disableReq,
                [](rclcpp::Client<dobot_msgs_v3::srv::DisableRobot>::SharedFuture df) {
                    try { qDebug() << "DisableRobot after E-STOP:" << df.get()->res; } catch (...) {}
                });
        } else {
            qWarning() << "DisableRobot service not ready after E-STOP";
        }

        callServiceAsync(enable_client_, false);
    }
    callServiceAsync(emergency_stop_client_, stop);
}

void RobotController::setManualMode(bool enable)
{
    qDebug() << "Set Manual mode";
    if (enable) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 3; // 3 = MANUAL
        set_mode_pub_->publish(msg);
    }
}

void RobotController::setAiMode(bool enable)
{
    qDebug() << "Set AI mode";
    if (enable) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 2; // 2 = AI
        set_mode_pub_->publish(msg);
    }
}

void RobotController::setAutoMode(bool enable)
{
    qDebug() << "Set Auto mode";
    if (enable) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 1; // 1 = AUTO
        set_mode_pub_->publish(msg);
    }
}

void RobotController::switchCamera(int cameraId)
{
    qDebug() << "Switch camera:" << cameraId;
    auto msg = std_msgs::msg::Int32();
    msg.data = cameraId;
    camera_select_pub_->publish(msg);
}

void RobotController::selectRow(int row)
{
    qDebug() << "Select row:" << row;
    selected_row_ = row;
    emit selectedRowChanged();
    auto msg = std_msgs::msg::Int32();
    msg.data = row;
    command_row_pub_->publish(msg);
}

void RobotController::selectSlot(int slot)
{
    qDebug() << "Select slot:" << slot;
    selected_slot_ = slot;
    emit selectedSlotChanged();
    auto msg = std_msgs::msg::Int32();
    msg.data = slot;
    command_slot_pub_->publish(msg);
}

void RobotController::gotoState(const QString& state)
{
    qDebug() << "Goto state:" << state;
    auto msg = std_msgs::msg::String();
    msg.data = state.toStdString();
    goto_state_pub_->publish(msg);
}

// ═══════════════════════════════════════════════════════════════
// AUTO-POLL REALTIME STATE
// ═══════════════════════════════════════════════════════════════

void RobotController::pollRobotState()
{
    pollErrorID();
}

// ═══════════════════════════════════════════════════════════════
// JOG CONTROL
// ═══════════════════════════════════════════════════════════════

void RobotController::setIgnoreScale(bool ignore)
{
    if (ignore_scale_ != ignore) {
        ignore_scale_ = ignore;
        emit ignoreScaleChanged();
        
        auto msg = std_msgs::msg::Bool();
        msg.data = ignore;
        if (ignore_scale_pub_) ignore_scale_pub_->publish(msg);
        qDebug() << "Ignore Scale set to:" << ignore;
    }
}

// Helper: parse "{v1,v2,...}" into vector<double>
static std::vector<double> parseDobot(const std::string& raw) {
    std::string s = raw;
    s.erase(std::remove(s.begin(), s.end(), '{'), s.end());
    s.erase(std::remove(s.begin(), s.end(), '}'), s.end());
    std::istringstream iss(s);
    std::vector<double> v;
    std::string tok;
    while (std::getline(iss, tok, ',')) v.push_back(std::stod(tok));
    return v;
}

// Helper: get joint axis index from "j1"-"j6"
static int jointIdx(const QString& axis) {
    if (axis == "j1") return 0; if (axis == "j2") return 1;
    if (axis == "j3") return 2; if (axis == "j4") return 3;
    if (axis == "j5") return 4; if (axis == "j6") return 5;
    return -1;
}

// Helper: get Cartesian axis index from "x","y","z","rx","ry","rz"
static int cartIdx(const QString& axis) {
    if (axis == "x") return 0; if (axis == "y") return 1;
    if (axis == "z") return 2; if (axis == "rx") return 3;
    if (axis == "ry") return 4; if (axis == "rz") return 5;
    return -1;
}

void RobotController::setJogContinuous(bool c) {
    if (jog_continuous_ != c) { jog_continuous_ = c; emit jogContinuousChanged(); }
}

void RobotController::setJogStepSize(double s) {
    if (jog_step_size_ != s) { jog_step_size_ = s; emit jogStepSizeChanged(); }
}

void RobotController::jogStep(const QString& axisId, double stepSize)
{
    const QString id = axisId.trimmed().toLower();
    if (id.size() < 2 || (stepSize != 0.1 && stepSize != 1.0 &&
                          stepSize != 5.0 && stepSize != 10.0)) {
        qWarning() << "[JOG STEP] Invalid command:" << axisId << stepSize;
        return;
    }

    const bool positive = id.endsWith('+');
    const double delta = positive ? stepSize : -stepSize;
    const QString axis = id.left(id.size() - 1);

    if (axis.startsWith('j')) {
        const int idx = jointIdx(axis);
        if (idx < 0 || joint_angles_.size() < 6 || !joint_movj_client_->service_is_ready()) {
            qWarning() << "[JOG STEP] Joint command unavailable:" << axisId;
            return;
        }
        double target[6];
        for (int i = 0; i < 6; ++i) target[i] = joint_angles_[i].toDouble();
        target[idx] += delta;
        setJogStepSize(stepSize);
        moveJoint(target[0], target[1], target[2], target[3], target[4], target[5]);
        return;
    }

    const int idx = cartIdx(axis);
    if (idx < 0 || cartesian_pose_.size() < 6 || !movl_client_->service_is_ready()) {
        qWarning() << "[JOG STEP] Cartesian command unavailable:" << axisId;
        return;
    }
    double target[6];
    for (int i = 0; i < 6; ++i) target[i] = cartesian_pose_[i].toDouble();
    target[idx] += delta;
    setJogStepSize(stepSize);
    moveLinear(target[0], target[1], target[2], target[3], target[4], target[5]);
}

void RobotController::jogStart(const QString& axisId)
{
    const QString input = axisId.trimmed();
    if (input.size() < 2) {
        qWarning() << "[JOG] Invalid axis:" << axisId;
        return;
    }

    const QChar direction = input.back();
    const QString axis = input.left(input.size() - 1).toLower();
    if (direction != '+' && direction != '-') {
        qWarning() << "[JOG] Invalid direction:" << axisId;
        return;
    }

    // Dobot's MoveJog command is case-sensitive.  Always construct the exact
    // identifiers accepted by the controller instead of trusting QML casing.
    static const QHash<QString, QString> dobotAxes{
        {"j1", "J1"}, {"j2", "J2"}, {"j3", "J3"},
        {"j4", "J4"}, {"j5", "J5"}, {"j6", "J6"},
        {"x", "X"}, {"y", "Y"}, {"z", "Z"},
        {"rx", "Rx"}, {"ry", "Ry"}, {"rz", "Rz"}
    };
    const auto axisIt = dobotAxes.constFind(axis);
    if (axisIt == dobotAxes.cend()) {
        qWarning() << "[JOG] Unsupported axis:" << axisId;
        return;
    }

    const QString fixedId = axisIt.value() + direction;
    const bool isJoint = axis.startsWith('j');

    if (isJoint && (!jog_client_ || !jog_client_->service_is_ready())) {
        qWarning() << "[JOG] MoveJog service is not ready";
        return;
    }
    if (!isJoint && (!servo_p_client_ || !servo_p_client_->service_is_ready())) {
        qWarning() << "[JOG] ServoP service is not ready";
        return;
    }

    qDebug() << "Jog start:" << axisId << "→" << fixedId << (isJoint ? "JOINT" : "CART");
    jog_axis_ = fixedId;
    jog_moving_ = true;
    jog_start_time_ = std::chrono::steady_clock::now();

    if (isJoint) {
        auto req = std::make_shared<dobot_msgs_v3::srv::MoveJog::Request>();
        req->axis_id = fixedId.toStdString();
        req->param_value = {};
        jog_client_->async_send_request(req,
            [this, fixedId](rclcpp::Client<dobot_msgs_v3::srv::MoveJog>::SharedFuture f) {
                try {
                    const int result = f.get()->res;
                    if (result != 0) {
                        qWarning() << "[JOG] MoveJog rejected:" << fixedId << "error:" << result;
                        jog_moving_ = false;
                        jog_axis_.clear();
                    }
                } catch (const std::exception& e) {
                    qWarning() << "[JOG] MoveJog failed:" << fixedId << e.what();
                    jog_moving_ = false;
                    jog_axis_.clear();
                }
            });
        return;
    }

    // Restore the known-good Cartesian implementation from da6011f:
    // stream small absolute ServoP setpoints from the latest measured pose.
    jog_cart_idx_ = cartIdx(axis);
    jog_cart_positive_ = direction == '+';
    if (jog_cart_idx_ < 0 || cartesian_pose_.size() < 6) {
        qWarning() << "[JOG] Current Cartesian pose is unavailable";
        jog_moving_ = false;
        jog_axis_.clear();
        return;
    }
    for (int i = 0; i < 6; ++i) {
        jog_cart_target_[i] = cartesian_pose_[i].toDouble();
    }

    if (!jog_timer_) {
        jog_timer_ = new QTimer(this);
        connect(jog_timer_, &QTimer::timeout, this, &RobotController::sendCartesianStep);
    }
    // Tick dau tien phai dung chu ky danh nghia, khong phai khoang cach tu lan
    // jog truoc (co the hang phut).
    jog_step_clock_.invalidate();
    jog_timer_->start(33);
    sendCartesianStep();
}

void RobotController::sendCartesianStep()
{
    if (!jog_moving_ || jog_cart_idx_ < 0) {
        if (jog_timer_) jog_timer_->stop();
        return;
    }

    // Jog Cartesian phai stream ServoP chu khong dung MoveJog: firmware nay tu
    // choi jog theo he toa do Cartesian (do 14/08/2026 tren robot that):
    //     MoveJog(X+,CoordType=1,User=0,Tool=0) -> -50001  (sai cu phap)
    //     MoveJog(X+,1,0,0)                     -> -50001
    //     MoveJog(X+)                           -> -6      (truc khong hop le)
    //     MoveJog(J6+)                          -> 0       (chay that)
    // Bare la cu phap duy nhat firmware nhan, ma o do chi co ten truc khop.
    // Khong truyen duoc CoordType => khong co duong nao de SpeedFactor tac dong
    // len jog Cartesian. Vi vay chia toc do o phia GUI.
    //
    // Tran toc do o speed 100%. DOI HAI SO NAY LA DOI TOC DO JOG. Do bang
    // scripts/dobot_servop_sweep.sh roi lay muc cao nhat con bam >=95%: vuot
    // nguong bam thi robot tut lai sau setpoint, va luc nha nut GUI ngung
    // stream nhung robot chay not toi setpoint cuoi => vot qua diem nha tay.
    // Do 14/08/2026 bang scripts/dobot_servop_sweep.sh tren truc Z, 20mm/muc:
    //     muon  dat   so voi muon   vot
    //       24  26.6      111%      0.00mm
    //       60  63.6      106%      0.90mm   <- chon muc nay
    //       80  82.9      104%      1.49mm
    //      100  90.7       91%      1.37mm
    //      120 102.7       86%      1.55mm
    // Bao hoa quanh 90-105mm/s: xin 120 chi nhan duoc 102. Vot chung o ~1.5mm
    // chu khong tang mai. Vi tri dung CUOI CUNG luon dung (+-0.01mm) o moi muc
    // — vot chi la dao dong tam roi ve dung cho, khong lam sai diem day.
    // Chon 60: con trong vung bam tuyen tinh, vot duoi 1mm. Muon nhanh hon nua
    // thi 80 da do la an toan, doi lai vot ~1.5mm.
    constexpr double kJogTopLinear  = 60.0;   // mm/s
    // Truc xoay do rieng cung ngay, tren RZ, 10deg/muc:
    //     muon  dat   so voi muon   vot
    //       12  13.6      114%      0.02deg
    //       24  27.8      116%      0.05deg
    //       36  35.9      100%      0.08deg
    //       48  49.5      103%      0.08deg  <- chon muc nay
    // Xoay bam tot hon han truc thang: chua thay bao hoa trong dai da thu, vot
    // chi 0.08deg (thap hon nguong 0.5 toi 6 lan). Chon 48 la muc cao nhat da
    // do; con du du dia neu sau nay can nhanh hon, quet tiep bang:
    //     ./dobot_servop_sweep.sh RZ 20 48,72,96
    // (can travel lon hon 10deg, vi o toc do cao 10deg chi con ~3 tick).
    constexpr double kJogTopAngular = 48.0;   // deg/s

    // QTimer 33ms tre hon dang ke khi QML va view camera dang render. Tinh buoc
    // theo chu ky DO DUOC thay vi 33ms danh nghia, neu khong jog chay cham hon
    // toc do dat dung bang ti le tre — day la mot phan ly do speed 100% van
    // cham. Chan tren de mot khung bi nghen khong sinh ra cu nhay lon.
    constexpr double kJogNominalSec = 0.033;
    constexpr double kJogMaxSec     = 0.060;
    const double tickSec = jog_step_clock_.isValid()
        ? std::clamp(jog_step_clock_.nsecsElapsed() * 1.0e-9,
                     kJogNominalSec * 0.5, kJogMaxSec)
        : kJogNominalSec;
    jog_step_clock_.restart();

    const double speedScale = std::max(0.05, speed_ratio_ / 100.0);
    const double top = (jog_cart_idx_ < 3) ? kJogTopLinear : kJogTopAngular;
    double step = top * tickSec * speedScale;
    double delta = jog_cart_positive_ ? step : -step;

    jog_cart_target_[jog_cart_idx_] += delta;

    // Guard: nếu Dobot bringup offline, ServoP service không bao giờ ack →
    // mỗi tick 33ms tạo 1 pending future không bao giờ resolve → leak nặng.
    if (!servo_p_client_->service_is_ready()) {
        static qint64 last_warn = 0;
        qint64 now = QDateTime::currentMSecsSinceEpoch();
        if (now - last_warn > 2000) {
            qWarning() << "[JOG] ServoP service offline — skipping cartesian step";
            last_warn = now;
        }
        return;
    }

    auto req = std::make_shared<dobot_msgs_v3::srv::ServoP::Request>();
    req->x  = jog_cart_target_[0];
    req->y  = jog_cart_target_[1];
    req->z  = jog_cart_target_[2];
    req->rx = jog_cart_target_[3];
    req->ry = jog_cart_target_[4];
    req->rz = jog_cart_target_[5];

    servo_p_client_->async_send_request(req);
}

void RobotController::sendMoveJog(const QString&) {}
void RobotController::sendJogStep() {}

void RobotController::stopManualJogMotion()
{
    const bool wasJoint = jog_axis_.startsWith('J');
    if (jog_timer_) jog_timer_->stop();
    jog_cart_idx_ = -1;
    jog_moving_ = false;
    jog_axis_.clear();

    // ServoP Cartesian jog stops by ending the stream.  Sending MoveJog("")
    // here was introduced with the later MoveJog implementation and can
    // disturb the ServoP command mode on the controller.
    if (!wasJoint) {
        qDebug() << "[JOG] Cartesian ServoP stream stopped";
        return;
    }

    if (!jog_client_ || !jog_client_->service_is_ready()) {
        qWarning() << "[JOG] MoveJog service not ready; cannot send manual jog stop";
        return;
    }

    auto req = std::make_shared<dobot_msgs_v3::srv::MoveJog::Request>();
    req->axis_id = "";
    req->param_value = {};
    jog_client_->async_send_request(req,
        [](rclcpp::Client<dobot_msgs_v3::srv::MoveJog>::SharedFuture f) {
            try { qDebug() << "[JOG] MoveJog stop res:" << f.get()->res; }
            catch (...) { qWarning() << "[JOG] MoveJog stop failed"; }
        });
}

void RobotController::jogStop()
{
    qDebug() << "Jog stop";
    stopManualJogMotion();
}

void RobotController::stopMotionOnly()
{
    qDebug() << "Stop motion only";
    stopManualJogMotion();

    if (dobot_stop_script_client_ && dobot_stop_script_client_->service_is_ready()) {
        auto stopScriptReq = std::make_shared<dobot_msgs_v3::srv::StopScript::Request>();
        dobot_stop_script_client_->async_send_request(stopScriptReq,
            [](rclcpp::Client<dobot_msgs_v3::srv::StopScript>::SharedFuture f) {
                try { qDebug() << "[STOP-ONLY] StopScript res:" << f.get()->res; }
                catch (...) { qWarning() << "[STOP-ONLY] StopScript failed"; }
            });
    } else {
        qWarning() << "[STOP-ONLY] StopScript service not ready";
    }

    // [HOLD-TO-MOVE RELEASE] Halt with ResetRobot ONLY — never Pause().
    // Verified live on this firmware: Pause() enters PAUSE (RobotMode 10) and
    // ONLY Continue() exits it (Continue would resume toward the old target).
    // ResetRobot from a non-paused state stops the motion, clears the queue,
    // and leaves the robot at RobotMode 5 (ENABLE) — so the NEXT press's
    // MovL/MovJ runs again. This is what makes SEND behave like JOG: press →
    // move toward the entered target, release → stop where it is, repeatable.
    if (reset_robot_client_ && reset_robot_client_->service_is_ready()) {
        auto resetReq = std::make_shared<dobot_msgs_v3::srv::ResetRobot::Request>();
        reset_robot_client_->async_send_request(resetReq,
            [](rclcpp::Client<dobot_msgs_v3::srv::ResetRobot>::SharedFuture f) {
                try { qDebug() << "[STOP-ONLY] ResetRobot res:" << f.get()->res; }
                catch (...) { qWarning() << "[STOP-ONLY] ResetRobot failed"; }
            });
    } else {
        qWarning() << "[STOP-ONLY] ResetRobot service not ready — cannot stop on release";
    }

    auto softStopReq = std::make_shared<std_srvs::srv::SetBool::Request>();
    softStopReq->data = true;
    if (soft_stop_to_manual_client_ && soft_stop_to_manual_client_->service_is_ready()) {
        soft_stop_to_manual_client_->async_send_request(softStopReq,
            [](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    qDebug() << "[STOP-ONLY] Motion command cleared:"
                             << future.get()->message.c_str();
                } catch (...) {
                    qWarning() << "[STOP-ONLY] Failed to clear motion command";
                }
            });
    } else {
        qWarning() << "[STOP-ONLY] /robot/soft_stop_to_manual not available";
    }
}

void RobotController::getAngles()
{
    if (!get_angle_client_->service_is_ready()) return;  // service offline → bỏ qua, không spam pending future

    auto request = std::make_shared<dobot_msgs_v3::srv::GetAngle::Request>();

    get_angle_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::GetAngle>::SharedFuture future) {
            // Parse trên executor thread, sau đó marshal kết quả về GUI thread.
            QVariantList angles;
            try {
                auto result = future.get();
                const std::string& raw = result->angle;
                auto beg = raw.find('{');
                auto end = raw.find('}');
                if (beg == std::string::npos || end == std::string::npos || end <= beg) return;
                std::string inner = raw.substr(beg + 1, end - beg - 1);
                std::istringstream iss(inner);
                std::string token;
                while (std::getline(iss, token, ',')) {
                    try { angles.append(std::stod(token)); }
                    catch (...) { angles.append(0.0); }
                }
                while (angles.size() < 6) angles.append(0.0);
            } catch (const std::exception& e) {
                qWarning() << "GetAngle failed:" << e.what();
                return;
            }
            QMetaObject::invokeMethod(this, [this, angles]() {
                joint_angles_ = angles;
                emit jointAnglesChanged();
            }, Qt::QueuedConnection);
        });
}

void RobotController::getPose()
{
    if (!get_pose_client_->service_is_ready()) return;

    auto request = std::make_shared<dobot_msgs_v3::srv::GetPose::Request>();
    request->user = 0;
    request->tool = 0;

    get_pose_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::GetPose>::SharedFuture future) {
            QVariantList pose;
            try {
                auto result = future.get();
                const std::string& raw = result->pose;
                auto beg = raw.find('{');
                auto end = raw.find('}');
                if (beg == std::string::npos || end == std::string::npos || end <= beg) return;
                std::string inner = raw.substr(beg + 1, end - beg - 1);
                std::istringstream iss(inner);
                std::string token;
                while (std::getline(iss, token, ',')) {
                    try { pose.append(std::stod(token)); }
                    catch (...) { pose.append(0.0); }
                }
                while (pose.size() < 6) pose.append(0.0);
            } catch (const std::exception& e) {
                qWarning() << "GetPose failed:" << e.what();
                return;
            }
            QMetaObject::invokeMethod(this, [this, pose]() {
                cartesian_pose_ = pose;
                emit cartesianPoseChanged();
            }, Qt::QueuedConnection);
        });
}


// ═══════════════════════════════════════════════════════════════
// MOVE TO EXACT POSITION
// ═══════════════════════════════════════════════════════════════

void RobotController::moveJoint(double j1, double j2, double j3, double j4, double j5, double j6)
{
    qDebug() << "MoveJoint:" << j1 << j2 << j3 << j4 << j5 << j6;
    auto request = std::make_shared<dobot_msgs_v3::srv::JointMovJ::Request>();
    request->j1 = j1; request->j2 = j2; request->j3 = j3;
    request->j4 = j4; request->j5 = j5; request->j6 = j6;
    joint_movj_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::JointMovJ>::SharedFuture future) {
            try { auto r = future.get(); qDebug() << "MoveJoint result:" << r->res; }
            catch (const std::exception& e) { qWarning() << "MoveJoint failed:" << e.what(); }
        });
}

void RobotController::moveLinear(double x, double y, double z, double rx, double ry, double rz)
{
    qDebug() << "MoveLinear:" << x << y << z << rx << ry << rz;
    auto request = std::make_shared<dobot_msgs_v3::srv::MovL::Request>();
    request->x = x; request->y = y; request->z = z;
    request->rx = rx; request->ry = ry; request->rz = rz;
    movl_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::MovL>::SharedFuture future) {
            try { auto r = future.get(); qDebug() << "MoveLinear result:" << r->res; }
            catch (const std::exception& e) { qWarning() << "MoveLinear failed:" << e.what(); }
        });
}

// ═══════════════════════════════════════════════════════════════
// SEND as HOLD-TO-MOVE without queued point commands:
//   SEND MovL → straight Cartesian interpolation streamed through ServoP.
//   SEND MovJ → coordinated joint interpolation streamed through ServoJ.
// Releasing either button stops the stream, so there is no stored target that
// can resume later after ENABLE/STOP.
//
// MAINTAINER NOTE: Do not "simplify" this back to one native MovL/JointMovJ
// request. Tests on the deployed Nova controller showed that those targets
// remain queued after button release and can resume after STOP or ENABLE.
// Pause also retains the target by design. Stopping the Servo stream is the
// only reliable hold-to-run behavior with this firmware/ROS bridge while
// preserving linear Cartesian and coordinated joint interpolation.
// ═══════════════════════════════════════════════════════════════
void RobotController::startSendMoveL(double x, double y, double z, double rx, double ry, double rz)
{
    const double dst[6] = {x, y, z, rx, ry, rz};
    for (double value : dst) {
        if (!std::isfinite(value)) {
            qWarning() << "[SEND-L] Invalid Cartesian target";
            return;
        }
    }
    if (!servo_p_client_ || !servo_p_client_->service_is_ready() ||
        cartesian_pose_.size() < 6) {
        qWarning() << "[SEND-L] ServoP/current Cartesian pose is not ready";
        return;
    }

    stopManualJogMotion();
    if (send_timer_) send_timer_->stop();
    for (int i = 0; i < 6; ++i) {
        send_current_[i] = cartesian_pose_[i].toDouble();
        send_target_[i] = dst[i];
    }
    send_motion_kind_ = 1;
    send_button_held_ = true;
    send_point_motion_active_ = true;
    send_linear_ramp_ = 0.0;
    send_step_clock_.restart();
    if (!send_timer_) {
        send_timer_ = new QTimer(this);
        send_timer_->setTimerType(Qt::PreciseTimer);
        connect(send_timer_, &QTimer::timeout, this, &RobotController::sendHoldStep);
    }
    // 40 Hz gives ServoP smaller Cartesian increments than the legacy 30 Hz
    // stream without pushing the Python ROS service bridge excessively hard.
    send_timer_->start(25);
    qDebug() << "[SEND-L] ServoP linear target →" << x << y << z << rx << ry << rz;
    sendHoldStep();
}

void RobotController::startSendMoveJ(double j1, double j2, double j3, double j4, double j5, double j6)
{
    const double dst[6] = {j1, j2, j3, j4, j5, j6};
    for (double value : dst) {
        if (!std::isfinite(value)) {
            qWarning() << "[SEND-J] Invalid joint target";
            return;
        }
    }
    if (!servo_j_client_ || !servo_j_client_->service_is_ready() ||
        joint_angles_.size() < 6) {
        qWarning() << "[SEND-J] ServoJ/current joint pose is not ready";
        return;
    }

    stopManualJogMotion();
    if (send_timer_) send_timer_->stop();
    for (int i = 0; i < 6; ++i) {
        send_current_[i] = joint_angles_[i].toDouble();
        send_target_[i] = dst[i];
    }
    send_motion_kind_ = 2;
    send_button_held_ = true;
    send_point_motion_active_ = true;
    if (!send_timer_) {
        send_timer_ = new QTimer(this);
        send_timer_->setTimerType(Qt::PreciseTimer);
        connect(send_timer_, &QTimer::timeout, this, &RobotController::sendHoldStep);
    }
    send_timer_->start(33);
    qDebug() << "[SEND-J] ServoJ joint target →" << j1 << j2 << j3 << j4 << j5 << j6;
    sendHoldStep();
}

void RobotController::sendHoldStep()
{
    if (!send_button_held_ || !send_point_motion_active_ ||
        (send_motion_kind_ != 1 && send_motion_kind_ != 2)) {
        if (send_timer_) send_timer_->stop();
        return;
    }

    if (send_motion_kind_ == 1 &&
        (!servo_p_client_ || !servo_p_client_->service_is_ready())) {
        return;
    }
    if (send_motion_kind_ == 2 &&
        (!servo_j_client_ || !servo_j_client_->service_is_ready())) {
        return;
    }

    const double speedScale = std::max(0.05, speed_ratio_ / 100.0);
    double timingScale = 1.0;
    if (send_motion_kind_ == 1) {
        // Compensate for GUI/render scheduling jitter. Clamp long stalls so a
        // delayed frame can never create a large Cartesian catch-up jump.
        constexpr double legacyPeriodSeconds = 0.033;
        constexpr double nominalPeriodSeconds = 0.025;
        constexpr double maxSafePeriodSeconds = 0.040;
        const double measuredSeconds = send_step_clock_.isValid()
            ? send_step_clock_.nsecsElapsed() * 1.0e-9
            : nominalPeriodSeconds;
        send_step_clock_.restart();
        const double stepSeconds = std::clamp(
            measuredSeconds, nominalPeriodSeconds * 0.5, maxSafePeriodSeconds);

        // Short smooth-start ramp removes the initial velocity step while
        // reaching the exact legacy top speed after roughly 180 ms.
        send_linear_ramp_ = std::min(1.0, send_linear_ramp_ + stepSeconds / 0.18);
        const double smoothRamp =
            send_linear_ramp_ * send_linear_ramp_ * (3.0 - 2.0 * send_linear_ramp_);
        timingScale = (stepSeconds / legacyPeriodSeconds) *
                      (0.30 + 0.70 * smoothRamp);
    }

    double ticksRemaining = 1.0;
    for (int i = 0; i < 6; ++i) {
        const double step = send_motion_kind_ == 2
                                ? 0.6 * speedScale
                                : (i < 3 ? 0.8 : 0.4) * speedScale * timingScale;
        ticksRemaining =
            std::max(ticksRemaining, std::abs(send_target_[i] - send_current_[i]) / step);
    }

    const double fraction = std::min(1.0, 1.0 / ticksRemaining);
    for (int i = 0; i < 6; ++i) {
        send_current_[i] += (send_target_[i] - send_current_[i]) * fraction;
    }

    if (send_motion_kind_ == 1) {
        auto req = std::make_shared<dobot_msgs_v3::srv::ServoP::Request>();
        req->x = send_current_[0]; req->y = send_current_[1];
        req->z = send_current_[2]; req->rx = send_current_[3];
        req->ry = send_current_[4]; req->rz = send_current_[5];
        servo_p_client_->async_send_request(req);
    } else {
        auto req = std::make_shared<dobot_msgs_v3::srv::ServoJ::Request>();
        req->j1 = send_current_[0]; req->j2 = send_current_[1];
        req->j3 = send_current_[2]; req->j4 = send_current_[3];
        req->j5 = send_current_[4]; req->j6 = send_current_[5];
        req->t = 0.1;
        req->param_value = {};
        servo_j_client_->async_send_request(req);
    }

    if (fraction >= 1.0) {
        send_point_motion_active_ = false;
        if (send_timer_) send_timer_->stop();
        qDebug() << "[SEND] target reached";
    }
}

void RobotController::stopSendMove()
{
    send_button_held_ = false;
    send_point_motion_active_ = false;
    send_motion_kind_ = 0;
    send_linear_ramp_ = 0.0;
    if (send_timer_) send_timer_->stop();
    qDebug() << "[SEND] released; Servo stream stopped";
}

void RobotController::saveJointPose(const QString& name, double j1, double j2, double j3, double j4, double j5, double j6)
{
    // Path to YAML config
    QString yaml_path = QString::fromStdString(
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "/home/pi") +
        "/ros2_ws/src/robot_control_main/config/joint_pose_params.yaml");

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        emit jointPoseSaved(false, tr("Cannot open YAML: %1").arg(yaml_path));
        return;
    }
    QString content = QString::fromUtf8(file.readAll());
    file.close();

    // Build new pose line: "J,j1,j2,j3,j4,j5,j6   # name"
    QString comment = name.trimmed().isEmpty() ? "" : ("     # " + name.trimmed());
    QString newLine = QString("      - \"J,%1,%2,%3,%4,%5,%6\"%7")
        .arg(j1, 0, 'f', 4)
        .arg(j2, 0, 'f', 4)
        .arg(j3, 0, 'f', 4)
        .arg(j4, 0, 'f', 4)
        .arg(j5, 0, 'f', 4)
        .arg(j6, 0, 'f', 4)
        .arg(comment);

    // Find insertion point: last "- \"J," entry in the joints list
    int lastJIdx = content.lastIndexOf(QRegularExpression("      - \"J,"));
    if (lastJIdx < 0) {
        emit jointPoseSaved(false, tr("Could not find the joint pose list in YAML"));
        return;
    }
    // Find end of that line
    int lineEnd = content.indexOf('\n', lastJIdx);
    if (lineEnd < 0) lineEnd = content.length();

    // Insert new line after the last J, entry
    content.insert(lineEnd + 1, newLine + "\n");

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        emit jointPoseSaved(false, tr("Cannot write YAML: %1").arg(yaml_path));
        return;
    }
    file.write(content.toUtf8());
    file.close();

    const QString msg = tr("Saved: \"%1\" → J(%2, %3, %4, %5, %6, %7)")
        .arg(name)
        .arg(j1, 0, 'f', 2).arg(j2, 0, 'f', 2).arg(j3, 0, 'f', 2)
        .arg(j4, 0, 'f', 2).arg(j5, 0, 'f', 2).arg(j6, 0, 'f', 2);
    qDebug() << "saveJointPose:" << msg;
    emit jointPoseSaved(true, msg);
}

QVariantList RobotController::getSavedPoses()
{
    QVariantList list;
    QString yaml_path = QString::fromStdString(
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "/home/pi") +
        "/ros2_ws/src/robot_control_main/config/joint_pose_params.yaml");

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Cannot open YAML for reading poses:" << yaml_path;
        return list;
    }

    while (!file.atEnd()) {
        QString line = QString::fromUtf8(file.readLine());
        // Check for joint move string
        if (line.contains("- \"J,") || line.contains("- 'J,")) {
            // Find name comment
            QString name = "";
            int hashIdx = line.indexOf('#');
            if (hashIdx >= 0) {
                name = line.mid(hashIdx + 1).trimmed();
            }

            // Skip placeholders or entries that contain "placeholder"
            if (name.toLower().contains("placeholder")) {
                continue;
            }

            // Extract coordinate string between quotes
            int firstQuote = line.indexOf('"');
            int lastQuote = line.lastIndexOf('"');
            if (firstQuote < 0 || lastQuote < 0 || lastQuote <= firstQuote) {
                firstQuote = line.indexOf('\'');
                lastQuote = line.lastIndexOf('\'');
            }

            if (firstQuote >= 0 && lastQuote > firstQuote) {
                QString coordStr = line.mid(firstQuote + 1, lastQuote - firstQuote - 1);
                // coordStr looks like "J,104.2439,45.5457,-153.7780,62.7823,85.6854,184.4839"
                if (coordStr.startsWith("J,")) {
                    QStringList parts = coordStr.mid(2).split(',');
                    if (parts.size() >= 6) {
                        QVariantMap map;
                        if (name.isEmpty()) {
                            name = tr("Pose (index %1)").arg(list.size());
                        }
                        map["name"] = name;
                        map["j1"] = parts[0].toDouble();
                        map["j2"] = parts[1].toDouble();
                        map["j3"] = parts[2].toDouble();
                        map["j4"] = parts[3].toDouble();
                        map["j5"] = parts[4].toDouble();
                        map["j6"] = parts[5].toDouble();
                        list.append(map);
                    }
                }
            }
        }
    }
    file.close();
    return list;
}



// ═══════════════════════════════════════════════════════════════
// DIGITAL OUTPUT CONTROL
// ═══════════════════════════════════════════════════════════════

void RobotController::setDigitalOutput(int index, bool status)
{
    qDebug() << "SetDO (via Topic):" << index << status;
    auto msg = std_msgs::msg::Bool();
    msg.data = status;
    if (index == 1 && gripper_cmd_pub_) {
        gripper_cmd_pub_->publish(msg);
        if (last_gripper_state_ != status) {
            last_gripper_state_ = status;
            emit gripperOnChanged();
        }
    } else if (index == 2 && picker_cmd_pub_) {
        picker_cmd_pub_->publish(msg);
        if (last_picker_state_ != status) {
            last_picker_state_ = status;
            emit pickerOnChanged();
        }
    } else if (index == 6 && cyl_loadcell_cmd_pub_) {
        cyl_loadcell_cmd_pub_->publish(msg);
        if (last_cyl_loadcell_state_ != status) {
            last_cyl_loadcell_state_ = status;
            emit cylLoadcellOnChanged();
        }
    } else {
        qWarning() << "Invalid DO index for GUI control:" << index;
    }
}

// ═══════════════════════════════════════════════════════════════
// PAUSE / RESUME / CLEAR ERROR
// ═══════════════════════════════════════════════════════════════

void RobotController::pauseRobot()
{
    qDebug() << "PauseRobot -> /system/pause_button + /robot/pause_system true";
    // Instant PAUSE: Dobot giữ trajectory hiện tại; cartridge dừng servo/VFD
    // và giữ nguyên state/step để Resume tiếp tục.
    if (system_pause_pub_) {
        std_msgs::msg::Bool m;
        m.data = true;
        system_pause_pub_->publish(m);
    }
    // Publish first so MotionExecutor blocks the next primitive before the
    // robot service checks whether a native Dobot Pause is required.
    callServiceAsync(pause_system_client_, true);
}

void RobotController::resumeRobot()
{
    qDebug() << "ResumeRobot -> /system/resume_button + /robot/pause_system false";
    if (system_resume_pub_) {
        std_msgs::msg::Bool m;
        m.data = true;
        system_resume_pub_->publish(m);
    }
    callServiceAsync(pause_system_client_, false);
}


void RobotController::clearError()
{
    qDebug() << "ClearError";
    auto request = std::make_shared<dobot_msgs_v3::srv::ClearError::Request>();
    clear_error_client_->async_send_request(request,
        [](rclcpp::Client<dobot_msgs_v3::srv::ClearError>::SharedFuture future) {
            try { qDebug() << "ClearError result:" << future.get()->res; }
            catch (const std::exception& e) { qWarning() << "ClearError failed:" << e.what(); }
        });
}

// ═══════════════════════════════════════════════════════════════
// SPEED FACTOR
// ═══════════════════════════════════════════════════════════════

void RobotController::setSpeedRatio(int ratio)
{
    qDebug() << "SetSpeedRatio:" << ratio;
    if (!speed_factor_client_->service_is_ready()) {
        qWarning() << "[SPEED] SpeedFactor service offline — request dropped";
        return;
    }
    auto request = std::make_shared<dobot_msgs_v3::srv::SpeedFactor::Request>();
    request->ratio = ratio;
    speed_factor_client_->async_send_request(request,
        [this, ratio](rclcpp::Client<dobot_msgs_v3::srv::SpeedFactor>::SharedFuture future) {
            int res = -1;
            try {
                auto r = future.get();
                res = r->res;
                qDebug() << "SpeedFactor result:" << res;
            } catch (const std::exception& e) {
                qWarning() << "SpeedFactor failed:" << e.what();
                return;
            }
            if (res != 0) return;
            // QSettings + publisher đụng tới Qt object → marshal về GUI thread.
            QMetaObject::invokeMethod(this, [this, ratio]() {
                speed_ratio_ = ratio;
                emit speedRatioChanged();
                QSettings settings("RobotControl", "ManualMode");
                settings.setValue("speedRatio", ratio);
                settings.sync();
                std_msgs::msg::Int32 smsg;
                smsg.data = ratio;
                speed_ratio_pub_->publish(smsg);
                qDebug() << "[SPEED] Published speed_ratio to motion_executor:" << ratio;
            }, Qt::QueuedConnection);
        });
}

// ═══════════════════════════════════════════════════════════════
// ERROR ID POLLING
// ═══════════════════════════════════════════════════════════════

void RobotController::pollErrorID()
{
    if (!get_error_id_client_->service_is_ready()) return;

    auto request = std::make_shared<dobot_msgs_v3::srv::GetErrorID::Request>();
    get_error_id_client_->async_send_request(request,
        [this](rclcpp::Client<dobot_msgs_v3::srv::GetErrorID>::SharedFuture future) {
            QString newLog;
            try {
                auto r = future.get();
                newLog = QString::fromStdString(r->error_id);
            } catch (const std::exception&) {
                return;  // silently ignore poll errors
            }
            QMetaObject::invokeMethod(this, [this, newLog]() {
                if (newLog != error_log_) {
                    error_log_ = newLog;
                    emit errorLogChanged();
                }
            }, Qt::QueuedConnection);
        });
}

// ═══════════════════════════════════════════════════════════════
// SIMULATION TRIGGERS
// ═══════════════════════════════════════════════════════════════

void RobotController::simulateFeedChamber()
{
    qDebug() << "SimulateFeedChamber -> /revpi/feed_chamber = true";
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    feed_chamber_pub_->publish(msg);
    error_log_ = "[SIMULATION] ⚙️ PICK INPUT Signal Sent";
    emit errorLogChanged();
}

void RobotController::simulateFillDone()
{
    qDebug() << "SimulateFillDone -> /revpi/fill_done = true";
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    fill_done_pub_->publish(msg);
    error_log_ = "[SIMULATION] 💧 PICK CHAMBER (Fill Done) Sent";
    emit errorLogChanged();
}

void RobotController::simulateInputTrayReady()
{
    const bool next = !in_ready_;
    qDebug() << "SimulateInputTrayReady toggling to" << next;

    // Reflect the operator command immediately. The real cartridge status topic
    // remains authoritative and will correct this value if hardware disagrees.
    in_ready_ = next;
    emit inReadyChanged();

    auto msg = std_msgs::msg::Bool();
    msg.data = next;
    input_tray_ready_pub_->publish(msg);
    error_log_ = msg.data ? "[SIMULATION] 📥 INPUT TRAY READY (TRUE)" : "[SIMULATION] 📥 INPUT TRAY READY (FALSE)";
    emit errorLogChanged();
}

void RobotController::simulateOutputTrayReady()
{
    const bool next = !out_ready_;
    qDebug() << "SimulateOutputTrayReady toggling to" << next;

    // Reflect the operator command immediately. The real cartridge status topic
    // remains authoritative and will correct this value if hardware disagrees.
    out_ready_ = next;
    emit outReadyChanged();

    auto msg = std_msgs::msg::Bool();
    msg.data = next;
    output_tray_ready_pub_->publish(msg);
    error_log_ = msg.data ? "[SIMULATION] 📤 OUTPUT TRAY READY (TRUE)" : "[SIMULATION] 📤 OUTPUT TRAY READY (FALSE)";
    emit errorLogChanged();
}

void RobotController::publishScaleResult(bool pass)
{
    qDebug() << "Scale result:" << (pass ? "PASS" : "FAIL");
    auto msg = std_msgs::msg::Bool();
    msg.data = pass;
    scale_result_pub_->publish(msg);
    error_log_ = pass ? "[SCALE] ✅ PASS published" : "[SCALE] ❌ FAIL published";
    emit errorLogChanged();
}
