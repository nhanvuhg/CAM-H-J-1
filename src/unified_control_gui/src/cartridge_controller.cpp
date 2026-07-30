#include "unified_control_gui/cartridge_controller.hpp"
#include <QDebug>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>

CartridgeController::CartridgeController(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent), node_(node)
{
    auto qos = rclcpp::QoS(10);

    // ── UI hint auto-clear timer (5s sau khi set) ────────────────
    ui_hint_timer_ = new QTimer(this);
    ui_hint_timer_->setSingleShot(true);
    ui_hint_timer_->setInterval(5000);
    connect(ui_hint_timer_, &QTimer::timeout, this, [this]() {
        if (!ui_hint_.isEmpty()) {
            ui_hint_.clear();
            emit uiHintChanged();
        }
    });

    // ── Publishers ───────────────────────────────────────────────
    jog_pub_              = node_->create_publisher<std_msgs::msg::String>("/providesystem/jog_cmd", qos);
    set_mode_pub_         = node_->create_publisher<std_msgs::msg::String>("/providesystem/set_operation_mode", qos);
    goto_state_pub_       = node_->create_publisher<std_msgs::msg::String>("/providesystem/goto_state", qos);
    update_config_pub_    = node_->create_publisher<std_msgs::msg::String>("/providesystem/update_config", qos);
    get_config_pub_       = node_->create_publisher<std_msgs::msg::String>("/providesystem/get_config", qos);
    start_button_pub_     = node_->create_publisher<std_msgs::msg::Bool>("/system/start_button", qos);
    stop_button_pub_      = node_->create_publisher<std_msgs::msg::Bool>("/system/stop_button", qos);
    soft_stop_pub_        = node_->create_publisher<std_msgs::msg::Bool>("/system/soft_stop", qos);
    pause_button_pub_     = node_->create_publisher<std_msgs::msg::Bool>("/system/pause_button", qos);
    resume_button_pub_    = node_->create_publisher<std_msgs::msg::Bool>("/system/resume_button", qos);
    robot_pause_client_   = node_->create_client<std_srvs::srv::SetBool>("/robot/pause_system");
    gui_confirm_pub_      = node_->create_publisher<std_msgs::msg::String>("/providesystem/gui_confirm", qos);
    set_target_row_pub_   = node_->create_publisher<std_msgs::msg::String>("/providesystem/set_target_row", qos);
    cyl_cmd_pub_          = node_->create_publisher<std_msgs::msg::String>("/providesystem/cyl_cmd", qos);

    // ── Simulate robot signals (nút STATE 2 / STATE 4 trong GUI) ─
    done_tray_input_pub_  = node_->create_publisher<std_msgs::msg::Bool>("/robot/done_tray_input", qos);
    done_tray_output_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/done_tray_output", qos);

    // ── Subscribers ──────────────────────────────────────────────
    system_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/system_state", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString raw = QString::fromStdString(msg->data);
                // Format: "global|state_in|state_out"
                QStringList parts = raw.split('|');
                system_state_ = parts.value(0);
                state_in_     = parts.value(1);
                state_out_    = parts.value(2);
                emit systemStateChanged();
            }, Qt::QueuedConnection);
        });

    // current_mode từ Python: "jog" | "manual" | "auto"
    current_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/current_mode", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString mode = QString::fromStdString(msg->data);
                if (current_mode_ != mode) {
                    current_mode_ = mode;
                    emit currentModeChanged();
                }
            }, Qt::QueuedConnection);
        });

    config_data_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/config_data",
        rclcpp::QoS(10).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                config_data_ = QString::fromStdString(msg->data);
                emit configDataChanged();
            }, Qt::QueuedConnection);
        });

    gui_notify_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/gui_notify", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                last_notification_ = QString::fromStdString(msg->data);
                QJsonDocument doc  = QJsonDocument::fromJson(last_notification_.toUtf8());
                if (doc.isObject()) {
                    QJsonObject obj = doc.object();
                    QString level   = obj.value("level").toString("info");
                    QString title   = obj.value("title").toString();
                    QString detail  = obj.value("detail").toString();
                    QString hint    = obj.value("hint").toString();
                    QString logMsg  = detail.isEmpty() ? title : title + " — " + detail;
                    QString type    = (level == "error" || level == "warn") ? "err" : (level == "ok" || level == "silent_ok") ? "ok" : "info";
                    addLog(logMsg, type);
                    if (!level.startsWith("silent")) {
                        emit notificationReceived();
                    }
                    // UI hint: blink button tương ứng trong 5s
                    if (!hint.isEmpty() && hint != ui_hint_) {
                        ui_hint_ = hint;
                        emit uiHintChanged();
                        if (ui_hint_timer_) ui_hint_timer_->start();
                    }
                } else {
                    addLog(last_notification_, "info");
                    emit notificationReceived();
                }
            }, Qt::QueuedConnection);
        });

    servo_pos_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/servo_positions", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                servo_positions_ = QString::fromStdString(msg->data);
                emit servoPositionsChanged();
            }, Qt::QueuedConnection);
        });

    sensors_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/providesystem/sensors_state", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString s = QString::fromStdString(msg->data);
                if (sensor_state_ != s) {
                    sensor_state_ = s;
                    emit sensorStateChanged();
                }
            }, Qt::QueuedConnection);
        });

    qDebug() << "CartridgeController v8 initialized";
}

// ── Helpers ───────────────────────────────────────────────────────

void CartridgeController::publishString(
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
    const QString &data)
{
    auto msg = std_msgs::msg::String();
    msg.data = data.toStdString();
    pub->publish(msg);
}

void CartridgeController::publishBool(
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub,
    bool value)
{
    auto msg = std_msgs::msg::Bool();
    msg.data = value;
    pub->publish(msg);
}

// ── Servo JOG ─────────────────────────────────────────────────────

void CartridgeController::jogServo(int id, const QString &dir, int velocity)
{
    publishString(jog_pub_, QString("%1 %2 %3").arg(id).arg(dir).arg(velocity));
}

void CartridgeController::jogStop(int id)
{
    publishString(jog_pub_, QString("%1 stop").arg(id));
}

void CartridgeController::setJogVelocity(const QString &velocity_ms)
{
    publishString(jog_pub_, QString("0 set_jog_vel %1").arg(velocity_ms));
}

void CartridgeController::homeServo(int id)
{
    publishString(jog_pub_, QString("home %1").arg(id));
    addLog(QString("Homing servo %1").arg(id), "info");
}

void CartridgeController::clearServo(int id)
{
    publishString(jog_pub_, QString("clear %1").arg(id));
    addLog(QString("Clear fault servo %1").arg(id), "info");
}

void CartridgeController::moveServo(int id, double position)
{
    if (id == 1) {
        double min_val = -322.0;
        double max_val = 560.0;
        if (position < min_val || position > max_val) {
            addLog(QString("LỖI: Trục S1 (InX) vượt giới hạn [%2, %3] mm (Nhập: %1)").arg(position).arg(min_val).arg(max_val), "err");
            return;
        }
    } else if (id == 2) {
        double min_val = -80.0;
        double max_val = 1025.0;
        if (position < min_val || position > max_val) {
            addLog(QString("LỖI: Trục S2 (InY) vượt giới hạn [%2, %3] mm (Nhập: %1)").arg(position).arg(min_val).arg(max_val), "err");
            return;
        }
    }

    publishString(jog_pub_, QString("%1 move %2").arg(id).arg(position));
    addLog(QString("Move S%1 → %2mm").arg(id).arg(position), "ok");
}

// ── System Control ────────────────────────────────────────────────

void CartridgeController::setMode(const QString &mode)
{
    publishString(set_mode_pub_, mode);
    if (current_mode_ != mode) {
        current_mode_ = mode;
        emit currentModeChanged();
    }
    addLog(QString("Mode: %1").arg(mode.toUpper()), "ok");
}

void CartridgeController::gotoState(const QString &state)
{
    const QString normalized = state.trimmed().toUpper();
    const qint64 now = QDateTime::currentMSecsSinceEpoch();

    auto activeForState = [this](const QString &cmd) {
        const QString in = state_in_.toUpper();
        const QString out = state_out_.toUpper();
        if (cmd == "STATE1" || cmd == "S1" || cmd == "STATE_1")
            return in.startsWith("S1_") || in.contains("STATE1");
        if (cmd == "STATE2" || cmd == "S2" || cmd == "STATE_2" || cmd == "STATE2A" || cmd == "S2A")
            return in.startsWith("S2") || in.contains("STATE2");
        if (cmd == "STATE3" || cmd == "S3" || cmd == "STATE_3")
            return out.startsWith("S3_") || out.contains("STATE3");
        if (cmd == "STATE4" || cmd == "S4" || cmd == "STATE_4")
            return out.startsWith("S4_") || out.contains("STATE4");
        return false;
    };

    if (activeForState(normalized)) {
        addLog(QString("Ignore duplicate active state: %1").arg(normalized), "warn");
        return;
    }

    if (normalized == last_goto_state_ && now - last_goto_state_ms_ < 800) {
        addLog(QString("Debounce state: %1").arg(normalized), "warn");
        return;
    }

    last_goto_state_ = normalized;
    last_goto_state_ms_ = now;

    publishString(goto_state_pub_, normalized);
    addLog(QString("Goto state: %1").arg(normalized), "info");
}

void CartridgeController::setTargetRow(int row)
{
    publishString(set_target_row_pub_, QString::number(row));
    addLog(QString("Set Target Row: %1").arg(row), "ok");
}

void CartridgeController::startSystem()
{
    publishBool(start_button_pub_, true);
    addLog("System START", "ok");
}

void CartridgeController::stopSystem()
{
    publishBool(stop_button_pub_, true);
    addLog("System STOP", "err");
}

void CartridgeController::softStop()
{
    // Soft STOP: dừng motion + chuyển MANUAL, GIỮ NGUYÊN state + CPX
    publishBool(soft_stop_pub_, true);
    addLog("Soft STOP (keep state)", "warn");
}

void CartridgeController::pauseSystem()
{
    // Sync 2 chiều: cartridge dừng tại ranh giới state, robot dừng tại
    // ranh giới motion goal — cả 2 graceful, không halt giữa chừng.
    publishBool(pause_button_pub_, true);
    if (robot_pause_client_ && robot_pause_client_->service_is_ready()) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        robot_pause_client_->async_send_request(req);
        addLog("PAUSE → cartridge + robot (graceful)", "info");
    } else {
        addLog("PAUSE → cartridge (robot pause service offline)", "warn");
    }
}

void CartridgeController::resumeSystem()
{
    // RESUME cả cartridge + robot. State machine tự pick up từ trạng thái đang giữ.
    publishBool(resume_button_pub_, true);
    if (robot_pause_client_ && robot_pause_client_->service_is_ready()) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = false;
        robot_pause_client_->async_send_request(req);
        addLog("RESUME → cartridge + robot", "ok");
    } else {
        addLog("RESUME → cartridge (robot pause service offline)", "warn");
    }
}

void CartridgeController::hmiResume()
{
    // Legacy no-op — giữ để các QML chưa update không crash. New code dùng resumeSystem().
    addLog("HMI Resume (legacy no-op)", "info");
}

void CartridgeController::resetFaults()
{
    // Gửi ABORT_TO_JOG để về JOG an toàn
    publishString(goto_state_pub_, "ABORT_TO_JOG");
    addLog("Reset faults → ABORT_TO_JOG", "info");
}

void CartridgeController::abortToJog()
{
    publishString(goto_state_pub_, "ABORT_TO_JOG");
    addLog("ABORT → JOG mode", "info");
}

// ── Robot signal simulation (nút STATE 2 / STATE 4) ───────────────

void CartridgeController::simulateDoneTrayInput()
{
    publishBool(done_tray_input_pub_, true);
    addLog("Simulate: done_tray_input (trigger State 2)", "info");
}

void CartridgeController::simulateDoneTrayOutput()
{
    publishBool(done_tray_output_pub_, true);
    addLog("Simulate: done_tray_output (trigger State 4)", "info");
}

void CartridgeController::confirmOutput()
{
    // Giữ để tương thích QML cũ
    simulateDoneTrayOutput();
}

// ── Cylinder manual control ───────────────────────────────────────

void CartridgeController::cylinderCmd(int cylId, bool extend)
{
    QString cmd = QString("%1 %2").arg(cylId).arg(extend ? "extend" : "retract");
    publishString(cyl_cmd_pub_, cmd);
    addLog(QString("Cyl%1 %2").arg(cylId).arg(extend ? "EXTEND" : "RETRACT"), "info");
}

// ── Config ────────────────────────────────────────────────────────

void CartridgeController::getConfig()
{
    publishString(get_config_pub_, "request");
}

void CartridgeController::saveConfig(const QString &key, const QString &jsonData)
{
    QJsonObject obj;
    obj["key"]  = key;
    obj["data"] = jsonData;
    QString payload = QString::fromUtf8(
        QJsonDocument(obj).toJson(QJsonDocument::Compact));
    publishString(update_config_pub_, payload);
    addLog(QString("Config saved: %1").arg(key), "ok");
}

// ── Log ───────────────────────────────────────────────────────────

void CartridgeController::addLog(const QString &msg, const QString &type)
{
    QVariantMap entry;
    entry["time"] = QDateTime::currentDateTime().toString("HH:mm:ss");
    entry["msg"]  = msg;
    entry["type"] = type;
    log_entries_.prepend(entry);
    if (log_entries_.size() > 200)
        log_entries_.removeLast();
    emit logEntriesChanged();
}

void CartridgeController::clearLog()
{
    log_entries_.clear();
    emit logEntriesChanged();
}
