#include "unified_control_gui/system_alert_controller.hpp"

#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QRegularExpression>

#include <algorithm>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace {

QString compactText(const QString &value)
{
    QString result = value.simplified();
    return result;
}

bool isFeederLifecycleWarning(const QString &title)
{
    const QString value = title.trimmed().toUpper();
    return value == "STOP" || value == "SOFT STOP" || value == "PAUSED"
        || value == "ĐANG PAUSE" || value == "RESUMED";
}

} // namespace

SystemAlertController::SystemAlertController(
    rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent), node_(std::move(node))
{
    const auto qos = rclcpp::QoS(10);
    const auto transientQos = rclcpp::QoS(1).reliable().transient_local();

    auto subscribeString = [this](const QString &source, const std::string &topic,
                                  const rclcpp::QoS &topicQos) {
        auto subscription = node_->create_subscription<std_msgs::msg::String>(
            topic, topicQos,
            [this, source](const std_msgs::msg::String::SharedPtr message) {
                const QString value = QString::fromStdString(message->data);
                QMetaObject::invokeMethod(this, [this, source, value]() {
                    observeString(source, value);
                }, Qt::QueuedConnection);
            });
        subscriptions_.push_back(subscription);
    };

    auto subscribeBool = [this](const QString &source, const std::string &topic) {
        auto subscription = node_->create_subscription<std_msgs::msg::Bool>(
            topic, rclcpp::QoS(10),
            [this, source](const std_msgs::msg::Bool::SharedPtr message) {
                const bool value = message->data;
                QMetaObject::invokeMethod(this, [this, source, value]() {
                    observeBool(source, value);
                }, Qt::QueuedConnection);
            });
        subscriptions_.push_back(subscription);
    };

    subscribeString("robot_error", "/robot/error", qos);
    subscribeString("robot_system_status", "/robot/system_status", qos);
    subscribeString("robot_heartbeat", "/robot/system_uptime", qos);
    subscribeString("feeder_gui_notify", "/providesystem/gui_notify", qos);
    subscribeString("feeder_servo_positions", "/providesystem/servo_positions", qos);
    subscribeString("feeder_system_state", "/system_state", rclcpp::SensorDataQoS());
    subscribeString("hw_status", "/hw_status", qos);
    subscribeString("error_status", "/error_status", transientQos);
    subscribeString("scale_monitor_status", "/weight/monitor_status", qos);
    subscribeString("scale_status", "/loadcell/status", qos);
    subscribeString("scale_cal_status", "/loadcell/cal_status", qos);
    subscribeString("vfd_status", "/vfd/status", qos);
    subscribeString("camera_status", "/camera/status", qos);
    subscribeString("camera_cam0_health", "/camera/cam0/health", qos);
    subscribeString("camera_cam1_health", "/camera/cam1/health", qos);
    subscribeString("vision_roi_status", "/vision/roi_status", transientQos);

    auto robotFeedbackSubscription =
        node_->create_subscription<sensor_msgs::msg::JointState>(
            "/nova5/joint_states_robot", rclcpp::SensorDataQoS().keep_last(1),
            [this](const sensor_msgs::msg::JointState::SharedPtr) {
                QMetaObject::invokeMethod(this, [this]() {
                    markHeartbeat("robot_feedback");
                }, Qt::QueuedConnection);
            });
    subscriptions_.push_back(robotFeedbackSubscription);

    subscribeBool("scale_overload", "/loadcell/overload");
    subscribeBool("scale_zero_drift", "/loadcell/zero_drift_warning");

    heartbeat_watch_started_ms_ = QDateTime::currentMSecsSinceEpoch();
    heartbeat_timer_ = new QTimer(this);
    heartbeat_timer_->setInterval(1000);
    connect(heartbeat_timer_, &QTimer::timeout,
            this, &SystemAlertController::checkHeartbeats);
    heartbeat_timer_->start();
}

QVariantList SystemAlertController::activeAlerts() const
{
    QList<Alert> ordered = alerts_.values();
    std::sort(ordered.begin(), ordered.end(), [](const Alert &left, const Alert &right) {
        const int leftRank = left.level == "ERROR" ? 0 : (left.acknowledged ? 2 : 1);
        const int rightRank = right.level == "ERROR" ? 0 : (right.acknowledged ? 2 : 1);
        if (leftRank != rightRank)
            return leftRank < rightRank;
        return left.sequence > right.sequence;
    });

    QVariantList result;
    result.reserve(ordered.size());
    for (const Alert &alert : ordered) {
        QVariantMap item;
        item["id"] = alert.id;
        item["source"] = alert.source;
        item["level"] = alert.level;
        item["area"] = alert.area;
        item["title"] = localizedAlertText(alert.title);
        item["message"] = localizedAlertText(alert.message);
        item["action"] = actionForArea(alert.area);
        item["time"] = alert.time;
        item["acknowledged"] = alert.acknowledged;
        result.append(item);
    }
    return result;
}

int SystemAlertController::errorCount() const
{
    int count = 0;
    for (const Alert &alert : alerts_)
        count += alert.level == "ERROR" ? 1 : 0;
    return count;
}

int SystemAlertController::warningCount() const
{
    int count = 0;
    for (const Alert &alert : alerts_)
        count += alert.level == "WARNING" ? 1 : 0;
    return count;
}

int SystemAlertController::unacknowledgedWarningCount() const
{
    int count = 0;
    for (const Alert &alert : alerts_)
        count += alert.level == "WARNING" && !alert.acknowledged ? 1 : 0;
    return count;
}

bool SystemAlertController::canStart() const
{
    return errorCount() == 0 && unacknowledgedWarningCount() == 0;
}

QString SystemAlertController::startBlockReason() const
{
    const int errors = errorCount();
    const int warnings = unacknowledgedWarningCount();
    if (errors > 0)
        return tr("%1 active error(s) — resolve them before START").arg(errors);
    if (warnings > 0)
        return tr("%1 unacknowledged warning(s) — open alerts to acknowledge")
            .arg(warnings);
    return QString();
}

void SystemAlertController::setOperationMode(const QString &mode)
{
    QString next = mode.trimmed().toLower();
    if (next == "camera_ai")
        next = "ai";
    if (next.isEmpty())
        next = "manual";
    if (operation_mode_ == next)
        return;
    operation_mode_ = next;
    emit operationModeChanged();
    reclassifyConnectionAlerts();
}

void SystemAlertController::setScaleIgnored(bool ignored)
{
    if (scale_ignored_ == ignored)
        return;
    scale_ignored_ = ignored;
    if (scale_ignored_)
        clearArea("SCALE");
    emit scaleIgnoredChanged();
}

bool SystemAlertController::prepareStart(const QString &mode)
{
    setOperationMode(mode);
    if (canStart())
        return true;
    emit attentionRequested();
    return false;
}

bool SystemAlertController::acknowledgeWarning(const QString &id)
{
    auto it = alerts_.find(id);
    if (it == alerts_.end() || it->level != "WARNING" || it->acknowledged)
        return false;
    it->acknowledged = true;
    emit alertsChanged();
    return true;
}

int SystemAlertController::acknowledgeAllWarnings()
{
    int changed = 0;
    for (Alert &alert : alerts_) {
        if (alert.level == "WARNING" && !alert.acknowledged) {
            alert.acknowledged = true;
            ++changed;
        }
    }
    if (changed > 0)
        emit alertsChanged();
    return changed;
}

void SystemAlertController::requestAttention()
{
    emit attentionRequested();
}

void SystemAlertController::refreshTranslations()
{
    // Alert payloads keep canonical source text. Rebuild the QVariant model
    // and start interlock description after QTranslator changes.
    emit alertsChanged();
}

void SystemAlertController::observeString(const QString &source, const QString &rawValue)
{
    const QString value = compactText(rawValue);
    const QString upper = normalized(value);

    if (source == "robot_heartbeat" || source == "feeder_system_state"
            || source == "camera_cam0_health" || source == "camera_cam1_health")
        markHeartbeat(source);

    if (source == "robot_heartbeat")
        return;

    if (scale_ignored_ && source.startsWith("scale_")) {
        clearAlert(source);
        return;
    }

    if (source == "feeder_gui_notify") {
        observeFeederNotification(value);
        return;
    }

    if (source == "feeder_servo_positions") {
        const QJsonDocument document = QJsonDocument::fromJson(value.toUtf8());
        if (!document.isObject())
            return;

        const QJsonObject rootObject = document.object();
        const QJsonObject status = rootObject.value("_servo_status").toObject();
        if (status.isEmpty())
            return;  // Rolling-deploy compatibility with an older feeder node.
        markHeartbeat(source);
        servo_status_contract_seen_ = true;

        const bool hasRequiredList = rootObject.contains("_servo_required");
        const QJsonArray required = rootObject.value("_servo_required").toArray();

        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        for (auto it = status.constBegin(); it != status.constEnd(); ++it) {
            bool validServoId = false;
            const int servoId = it.key().toInt(&validServoId);
            if (!validServoId || servoId <= 0)
                continue;

            const QString alertId = "feeder_servo_connection_s" + QString::number(servoId);
            const bool servoRequired = !hasRequiredList
                || required.contains(QJsonValue(it.key()));
            if (!servoRequired) {
                servo_offline_since_ms_.remove(alertId);
                clearAlert(alertId);
                continue;
            }
            const QString state = normalized(it.value().toString());
            if (state == "LIVE") {
                servo_offline_since_ms_.remove(alertId);
                clearAlert(alertId);
                continue;
            }
            if (state != "OFFLINE")
                continue;

            // Debounce startup/reconnect hand-off so a normally connecting
            // servo does not flash a popup. A confirmed OFFLINE remains
            // continuously observable, including after a GUI-only restart.
            const qint64 firstSeen = servo_offline_since_ms_.value(alertId, now);
            if (!servo_offline_since_ms_.contains(alertId))
                servo_offline_since_ms_.insert(alertId, now);
            if (now - firstSeen < 4000)
                continue;

            if (!alerts_.contains(alertId)) {
                const QString label = "Servo S" + QString::number(servoId);
                upsertAlert(alertId, source, "WARNING", "FEEDER",
                            label + " offline",
                            label + " is not connected; the system is retrying automatically.",
                            true);
            }
        }
        return;
    }

    if (source == "robot_error") {
        if (value.isEmpty() || isHealthy(value)) {
            clearAlert(source);
            return;
        }
        const bool connection = isConnectionIssue(value);
        upsertAlert(source, source, "ERROR", "ROBOT", "Robot error", value, connection);
        return;
    }

    if (source == "robot_system_status") {
        const bool fault = upper.startsWith("ERROR") || upper.startsWith("FAULT")
            || hasCriticalToken(value);
        if (fault) {
            robot_fault_state_seen_ = true;
            upsertAlert(source, source, "ERROR", "ROBOT", "Robot system", value,
                        isConnectionIssue(value));
        } else {
            clearAlert(source);
            if (robot_fault_state_seen_) {
                clearAlert("robot_error");
                robot_fault_state_seen_ = false;
            }
        }
        return;
    }

    if (source == "feeder_system_state") {
        const QString global = value.section('|', 0, 0);
        const bool fault = normalized(global).startsWith("ERROR")
            || normalized(global).startsWith("FAULT") || hasCriticalToken(global);
        if (fault) {
            feeder_fault_state_seen_ = true;
            upsertAlert(source, source, "ERROR", "FEEDER", "Cartridge system", value,
                        isConnectionIssue(value));
        } else {
            clearAlert(source);
            if (feeder_fault_state_seen_) {
                clearAreaErrors("FEEDER");
                feeder_fault_state_seen_ = false;
            }
        }
        return;
    }

    if (source == "vision_roi_status") {
        if (isHealthy(value)) {
            clearAlert(source);
        } else {
            upsertAlert(source, source, "ERROR", "CAMERA", "ROI validation", value);
        }
        return;
    }

    if (source == "camera_cam0_health" || source == "camera_cam1_health") {
        const QString camera = source.contains("cam0") ? "CAM0" : "CAM1";
        const QString stable = stableCameraMessage(source, value);
        if (upper.startsWith("RECONNECT")) {
            upsertAlert(source, source, "WARNING", "CAMERA", camera + " reconnecting",
                        stable, true);
        } else if (!isHealthy(value) && hasCriticalToken(value)) {
            upsertAlert(source, source, "ERROR", "CAMERA", camera + " camera fault",
                        stable, isConnectionIssue(value));
        } else {
            clearAlert(source);
        }
        return;
    }

    if (source == "camera_status") {
        if (isHealthy(value) || !hasCriticalToken(value))
            clearAlert(source);
        else
            upsertAlert(source, source, "ERROR", "CAMERA", "Camera system", value,
                        isConnectionIssue(value));
        return;
    }

    if (source == "error_status") {
        if (isHealthy(value))
            clearAlert(source);
        else
            upsertAlert(source, source, "ERROR", "FILL_HP", "Fill HP error", value,
                        isConnectionIssue(value));
        return;
    }

    if (source == "hw_status") {
        if (isHealthy(value) || !hasCriticalToken(value))
            clearAlert(source);
        else
            upsertAlert(source, source, "ERROR", "FILL_HP", "Fill HP hardware", value,
                        isConnectionIssue(value));
        return;
    }

    if (source == "scale_monitor_status" && upper.startsWith("LAST:")) {
        clearAlert(source);
        return;
    }

    if (source == "scale_monitor_status" || source == "scale_status"
            || source == "scale_cal_status" || source == "vfd_status") {
        if (isHealthy(value) || !hasCriticalToken(value)) {
            clearAlert(source);
            return;
        }
        const bool vfd = source == "vfd_status";
        upsertAlert(source, source, "ERROR", vfd ? "VFD" : "SCALE",
                    vfd ? "VFD fault" : "Scale fault", value,
                    isConnectionIssue(value));
    }
}

void SystemAlertController::observeBool(const QString &source, bool value)
{
    if (scale_ignored_ && source.startsWith("scale_")) {
        clearAlert(source);
        return;
    }
    if (!value) {
        clearAlert(source);
        return;
    }
    if (source == "scale_overload") {
        upsertAlert(source, source, "ERROR", "SCALE", "Loadcell overload",
                    "The loadcell reports an overload.");
    } else if (source == "scale_zero_drift") {
        upsertAlert(source, source, "WARNING", "SCALE", "Loadcell zero drift",
                    "The loadcell reports a zero-point drift warning.");
    }
}

void SystemAlertController::observeFeederNotification(const QString &value)
{
    const QJsonDocument document = QJsonDocument::fromJson(value.toUtf8());
    if (!document.isObject())
        return;

    const QJsonObject object = document.object();
    const QString rawLevel = object.value("level").toString().trimmed().toLower();
    const QString title = compactText(object.value("title").toString());
    const QString detail = compactText(
        object.value("detail").toString(object.value("message").toString()));
    const QString code = object.value("code").toString().trimmed().toLower();

    static const QRegularExpression servoExpression("(?:SERVO\\s*)?S(\\d+)",
        QRegularExpression::CaseInsensitiveOption);
    const QRegularExpressionMatch servoMatch = servoExpression.match(title + " " + detail);
    const QString servoId = servoMatch.hasMatch() ? servoMatch.captured(1) : QString();
    const QString notificationText = title + " " + detail;
    const QString normalizedNotification = normalized(notificationText);
    const bool servoConnectionNotification = !servoId.isEmpty()
        && (isConnectionIssue(notificationText)
            || normalizedNotification.contains("KET NOI")
            || normalizedNotification.contains("CONNECTED"));
    const QString id = !servoId.isEmpty()
        ? (servoConnectionNotification
            ? "feeder_servo_connection_s" + servoId
            : "feeder_servo_s" + servoId)
        : (!code.isEmpty() ? "feeder_notify_" + code : "feeder_gui_alert");

    // Once the continuously published status contract is available it owns
    // connectivity alerts. The notification topic is still logged by
    // CartridgeController, but must not reset ACK or race clear/re-add here.
    if (servo_status_contract_seen_ && servoConnectionNotification)
        return;

    if (rawLevel == "ok" || rawLevel == "silent_ok" || rawLevel == "info") {
        if (!servoId.isEmpty() && (normalized(title).contains("KET NOI")
                || normalized(title).contains("CONNECTED")
                || normalized(detail).endsWith(" OK")))
            clearAlert(id);
        if (code.endsWith("_clear"))
            clearPrefix("feeder_notify_" + code.left(code.size() - 6));
        return;
    }

    if (rawLevel != "warn" && rawLevel != "warning" && rawLevel != "error")
        return;
    if (isFeederLifecycleWarning(title))
        return;

    const QString message = detail.isEmpty() ? title : title + " — " + detail;
    const bool warning = rawLevel != "error";
    upsertAlert(id, "feeder_gui_notify", warning ? "WARNING" : "ERROR", "FEEDER",
                title.isEmpty() ? "Cartridge notification" : title,
                message, isConnectionIssue(message));
}

void SystemAlertController::upsertAlert(
    const QString &id, const QString &source, const QString &baseLevel,
    const QString &area, const QString &title, const QString &message,
    bool connectionIssue)
{
    if (message.trimmed().isEmpty())
        return;
    const QString level = effectiveLevel(baseLevel, connectionIssue);
    auto it = alerts_.find(id);
    if (it != alerts_.end() && it->level == level && it->message == message)
        return;

    Alert alert;
    if (it != alerts_.end())
        alert = *it;
    alert.id = id;
    alert.source = source;
    alert.level = level;
    alert.area = area;
    alert.title = title;
    alert.message = message;
    alert.action = actionForArea(area);
    alert.time = QDateTime::currentDateTime().toString("HH:mm:ss");
    alert.acknowledged = false;
    alert.connection_issue = connectionIssue;
    alert.sequence = next_sequence_++;
    alerts_[id] = alert;
    emit alertsChanged();
}

void SystemAlertController::clearAlert(const QString &id)
{
    if (alerts_.remove(id) > 0)
        emit alertsChanged();
}

void SystemAlertController::clearAreaErrors(const QString &area)
{
    bool changed = false;
    for (auto it = alerts_.begin(); it != alerts_.end();) {
        if (it->area == area && it->level == "ERROR") {
            it = alerts_.erase(it);
            changed = true;
        } else {
            ++it;
        }
    }
    if (changed)
        emit alertsChanged();
}

void SystemAlertController::clearArea(const QString &area)
{
    bool changed = false;
    for (auto it = alerts_.begin(); it != alerts_.end();) {
        if (it->area == area) {
            it = alerts_.erase(it);
            changed = true;
        } else {
            ++it;
        }
    }
    if (changed)
        emit alertsChanged();
}

void SystemAlertController::clearPrefix(const QString &prefix)
{
    bool changed = false;
    for (auto it = alerts_.begin(); it != alerts_.end();) {
        if (it.key().startsWith(prefix)) {
            it = alerts_.erase(it);
            changed = true;
        } else {
            ++it;
        }
    }
    if (changed)
        emit alertsChanged();
}

void SystemAlertController::reclassifyConnectionAlerts()
{
    bool changed = false;
    for (Alert &alert : alerts_) {
        if (!alert.connection_issue)
            continue;
        const QString next = effectiveLevel("ERROR", true);
        if (alert.level == next)
            continue;
        alert.level = next;
        alert.acknowledged = false;
        alert.sequence = next_sequence_++;
        changed = true;
    }
    if (changed)
        emit alertsChanged();
}

void SystemAlertController::markHeartbeat(const QString &source)
{
    heartbeat_seen_ms_[source] = QDateTime::currentMSecsSinceEpoch();
    // A live sample is the recovery condition for a missing-topic alert. For
    // camera/feeder sources observeString() may immediately replace it with a
    // more precise active fault from the payload.
    clearAlert("watchdog_" + source);
}

void SystemAlertController::checkHeartbeats()
{
    const qint64 now = QDateTime::currentMSecsSinceEpoch();

    struct Watch {
        const char *source;
        const char *area;
        const char *title;
        const char *message;
        qint64 timeout_ms;
    };
    static const Watch watches[] = {
        {"robot_heartbeat", "ROBOT", "Robot status offline",
         "No /robot/system_uptime heartbeat was received for 12 seconds.", 12000},
        {"robot_feedback", "ROBOT", "Robot hardware disconnected",
         "No hardware feedback was received from /nova5/joint_states_robot for 5 seconds.", 5000},
        {"feeder_system_state", "FEEDER", "Cartridge status offline",
         "No /system_state update was received from the cartridge system for 12 seconds.", 12000},
        {"feeder_servo_positions", "FEEDER", "Servo feedback offline",
         "No /providesystem/servo_positions feedback was received for 6 seconds.", 6000},
        {"camera_cam0_health", "CAMERA", "CAM0 status offline",
         "No /camera/cam0/health update was received for 12 seconds.", 12000},
        {"camera_cam1_health", "CAMERA", "CAM1 status offline",
         "No /camera/cam1/health update was received for 12 seconds.", 12000},
    };

    for (const Watch &watch : watches) {
        const QString source = QString::fromLatin1(watch.source);
        const qint64 lastSeen = heartbeat_seen_ms_.value(source, heartbeat_watch_started_ms_);
        if (now - lastSeen <= watch.timeout_ms)
            continue;
        upsertAlert("watchdog_" + source, source, "ERROR", QString::fromLatin1(watch.area),
                    QString::fromUtf8(watch.title), QString::fromUtf8(watch.message), true);
    }
}

QString SystemAlertController::effectiveLevel(
    const QString &baseLevel, bool connectionIssue) const
{
    // A missing driver is acknowledgeable while servicing in MANUAL, but it
    // must become a hard interlock before AUTO/AI can start. Apply this rule
    // even when the source topic itself labels reconnect as WARN.
    if (connectionIssue)
        return isProductionMode(operation_mode_) ? "ERROR" : "WARNING";
    return baseLevel.trimmed().toUpper() == "WARN" ? "WARNING"
        : baseLevel.trimmed().toUpper();
}

QString SystemAlertController::actionForArea(const QString &area) const
{
    if (area == "ROBOT")
        return tr("Stop motion, inspect the robot, and resume only after the fault is resolved.");
    if (area == "FEEDER")
        return tr("Inspect the cartridge feeder, servos, and sensors before continuing.");
    if (area == "SCALE")
        return tr("Inspect the scale/loadcell, RS485 connection, and the load on the scale.");
    if (area == "VFD")
        return tr("Stop the conveyor and inspect the VFD, RS485, and fault signal before restarting.");
    if (area == "CAMERA")
        return tr("Inspect the camera, CSI/VI, and image topics before continuing.");
    return tr("Inspect the Fill HP hardware and confirm a safe state before restarting.");
}

QString SystemAlertController::localizedAlertText(const QString &text) const
{
    // Incoming driver/ROS diagnostics remain verbatim. Only canonical GUI
    // phrases owned by this controller are translated here.
    if (text == "Robot error") return tr("Robot error");
    if (text == "Robot system") return tr("Robot system");
    if (text == "Cartridge system") return tr("Cartridge system");
    if (text == "ROI validation") return tr("ROI validation");
    if (text == "Camera system") return tr("Camera system");
    if (text == "Fill HP error") return tr("Fill HP error");
    if (text == "Fill HP hardware") return tr("Fill HP hardware");
    if (text == "VFD fault") return tr("VFD fault");
    if (text == "Scale fault") return tr("Scale fault");
    if (text == "Loadcell overload") return tr("Loadcell overload");
    if (text == "Loadcell zero drift") return tr("Loadcell zero drift");
    if (text == "Cartridge notification") return tr("Cartridge notification");
    if (text == "Robot status offline") return tr("Robot status offline");
    if (text == "Robot hardware disconnected") return tr("Robot hardware disconnected");
    if (text == "Cartridge status offline") return tr("Cartridge status offline");
    if (text == "Servo feedback offline") return tr("Servo feedback offline");
    if (text == "CAM0 status offline") return tr("CAM0 status offline");
    if (text == "CAM1 status offline") return tr("CAM1 status offline");
    if (text == "The loadcell reports an overload.")
        return tr("The loadcell reports an overload.");
    if (text == "The loadcell reports a zero-point drift warning.")
        return tr("The loadcell reports a zero-point drift warning.");
    if (text == "No /robot/system_uptime heartbeat was received for 12 seconds.")
        return tr("No /robot/system_uptime heartbeat was received for 12 seconds.");
    if (text == "No hardware feedback was received from /nova5/joint_states_robot for 5 seconds.")
        return tr("No hardware feedback was received from /nova5/joint_states_robot for 5 seconds.");
    if (text == "No /system_state update was received from the cartridge system for 12 seconds.")
        return tr("No /system_state update was received from the cartridge system for 12 seconds.");
    if (text == "No /providesystem/servo_positions feedback was received for 6 seconds.")
        return tr("No /providesystem/servo_positions feedback was received for 6 seconds.");
    if (text == "No /camera/cam0/health update was received for 12 seconds.")
        return tr("No /camera/cam0/health update was received for 12 seconds.");
    if (text == "No /camera/cam1/health update was received for 12 seconds.")
        return tr("No /camera/cam1/health update was received for 12 seconds.");

    static const QRegularExpression servoTitle("^Servo S(\\d+) offline$");
    QRegularExpressionMatch match = servoTitle.match(text);
    if (match.hasMatch())
        return tr("Servo S%1 offline").arg(match.captured(1));

    static const QRegularExpression servoMessage(
        "^Servo S(\\d+) is not connected; the system is retrying automatically\\.$");
    match = servoMessage.match(text);
    if (match.hasMatch())
        return tr("Servo S%1 is not connected; the system is retrying automatically.")
            .arg(match.captured(1));

    static const QRegularExpression cameraReconnect("^(CAM[01]) reconnecting$");
    match = cameraReconnect.match(text);
    if (match.hasMatch())
        return tr("%1 reconnecting").arg(match.captured(1));

    static const QRegularExpression cameraFault("^(CAM[01]) camera fault$");
    match = cameraFault.match(text);
    if (match.hasMatch())
        return tr("%1 camera fault").arg(match.captured(1));

    return text;
}

QString SystemAlertController::stableCameraMessage(
    const QString &source, const QString &value)
{
    const QString label = source.contains("cam0") ? "CAM0" : "CAM1";
    QString state = value.section(" device=", 0, 0);
    static const QRegularExpression deviceExpression("(?:^|\\s)device=([^\\s]+)");
    const QRegularExpressionMatch match = deviceExpression.match(value);
    if (match.hasMatch())
        state += " device=" + match.captured(1);
    return label + " " + state;
}

QString SystemAlertController::normalized(const QString &value)
{
    QString result = value.simplified().toUpper();
    result.replace('_', ' ');
    return result;
}

bool SystemAlertController::isHealthy(const QString &value)
{
    const QString text = normalized(value);
    static const QStringList exact = {
        "", "-", "OK", "READY", "IDLE", "RUNNING", "STANDBY", "STREAMING",
        "CONNECTED", "ONLINE", "HEALTHY", "DONE", "NORMAL", "ACTIVE"
    };
    if (exact.contains(text))
        return true;
    return text.startsWith("OK:") || text.startsWith("READY:")
        || text.startsWith("STREAMING ") || text.contains("NO ERROR")
        || text.contains("ERROR NONE");
}

bool SystemAlertController::hasCriticalToken(const QString &value)
{
    if (isHealthy(value))
        return false;
    static const QRegularExpression expression(
        "(?:^|[^A-Z0-9])(?:ERROR|FAULT|FATAL|FAILED?|FAILURE|TIMEOUT|OFFLINE|"
        "DISCONNECT(?:ED)?|NO[ ]SIGNAL|CORR[ ]ERR|OVERLOAD|EMERGENCY)"
        "(?:$|[^A-Z0-9])");
    return expression.match(normalized(value)).hasMatch();
}

bool SystemAlertController::isConnectionIssue(const QString &value)
{
    static const QRegularExpression expression(
        "(?:OFFLINE|DISCONNECT(?:ED)?|NO[ ]SIGNAL|RECONNECT|CONNECTION|RS485|CSI|VI)");
    return expression.match(normalized(value)).hasMatch();
}

bool SystemAlertController::isProductionMode(const QString &mode)
{
    const QString value = mode.trimmed().toLower();
    return value == "auto" || value == "ai" || value == "camera_ai"
        || value == "1" || value == "2";
}
