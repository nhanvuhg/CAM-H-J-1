#include "unified_control_gui/scale_controller.hpp"
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>
#include <QVariantMap>
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>
#include <QDateTime>
#include <QMetaObject>
#include <sys/types.h>
#include <pwd.h>
#include <unistd.h>

namespace {

QStringList parseCsvLine(const QString& line)
{
    QStringList fields;
    QString current;
    bool inQuotes = false;

    for (int i = 0; i < line.size(); ++i) {
        const QChar ch = line.at(i);
        if (ch == '"') {
            if (inQuotes && i + 1 < line.size() && line.at(i + 1) == '"') {
                current.append('"');
                ++i;
            } else {
                inQuotes = !inQuotes;
            }
        } else if (ch == ',' && !inQuotes) {
            fields.append(current.trimmed());
            current.clear();
        } else {
            current.append(ch);
        }
    }
    fields.append(current.trimmed());
    return fields;
}

int csvColumnIndex(const QStringList& headers, const QString& name)
{
    const QString wanted = name.trimmed().toLower();
    for (int i = 0; i < headers.size(); ++i) {
        if (headers.at(i).trimmed().toLower() == wanted) return i;
    }
    return -1;
}

QString csvValue(const QStringList& row, int index)
{
    return (index >= 0 && index < row.size()) ? row.at(index).trimmed() : QString();
}

float csvFloatValue(const QString& raw, float fallback = 0.0f)
{
    bool ok = false;
    const float value = raw.trimmed().replace(',', '.').toFloat(&ok);
    return ok ? value : fallback;
}

} // namespace

ScaleController::ScaleController(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent), node_(node)
{
    last_weight_time_ = 0;
    scale_node_connected_ = false;

    // Connection watchdog timer
    connection_timer_ = new QTimer(this);
    connect(connection_timer_, &QTimer::timeout, this, [this]() {
        qint64 now = QDateTime::currentMSecsSinceEpoch();
        bool connected = (last_weight_time_ > 0 && (now - last_weight_time_ < 2000));
        if (connected != scale_node_connected_) {
            scale_node_connected_ = connected;
            emit scaleNodeConnectedChanged();
            if (!scale_node_connected_) {
                loadcell_status_ = "OFFLINE";
                emit loadcellStatusChanged();
            }
        }
    });
    connection_timer_->start(1000);

    // Publishers
    pub_active_profile_ = node_->create_publisher<std_msgs::msg::String>("/weight/active_profile", 10);
    pub_target_weight_ = node_->create_publisher<std_msgs::msg::Float32>("/loadcell/target_weight", 10);
    pub_target_min_ = node_->create_publisher<std_msgs::msg::Float32>("/loadcell/target_min", 10);
    pub_target_max_ = node_->create_publisher<std_msgs::msg::Float32>("/loadcell/target_max", 10);
    pub_cal_weight_ = node_->create_publisher<std_msgs::msg::Float32>("/loadcell/cal_weight", 10);
    pub_tare_cmd_ = node_->create_publisher<std_msgs::msg::Bool>("/loadcell/tare_cmd", 10);
    pub_tare_reset_ = node_->create_publisher<std_msgs::msg::Bool>("/loadcell/tare_reset", 10);
    pub_overload_ack_ = node_->create_publisher<std_msgs::msg::Bool>("/loadcell/overload_ack", 10);
    pub_batch_reset_ = node_->create_publisher<std_msgs::msg::Bool>("/loadcell/batch_reset", 10);
    pub_ink_capacity_ = node_->create_publisher<std_msgs::msg::Float32>("/Fill_HP1/ink_capacity", 10);

    // Subscribers
    // INVARIANT: tất cả callback chạy trên ROS executor thread (rosThread trong main.cpp).
    //   Mọi thao tác state Qt + emit signal PHẢI marshal về GUI thread qua
    //   QMetaObject::invokeMethod(this, ..., Qt::QueuedConnection). Bug history:
    //   crash random khi QML đọc property cùng lúc executor đang write.
    sub_weight_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/loadcell/weight", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            const float w = msg->data;
            const qint64 t = QDateTime::currentMSecsSinceEpoch();
            QMetaObject::invokeMethod(this, [this, w, t]() {
                current_weight_ = w;
                last_weight_time_ = t;
                if (!scale_node_connected_) {
                    scale_node_connected_ = true;
                    emit scaleNodeConnectedChanged();
                }
                emit currentWeightChanged();
            }, Qt::QueuedConnection);
        });

    sub_monitor_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/weight/monitor_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            const QString s = QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(this, [this, s]() {
                monitor_status_ = s;
                emit monitorStatusChanged();
            }, Qt::QueuedConnection);
        });

    sub_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/loadcell/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            const QString s = QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(this, [this, s]() {
                loadcell_status_ = s;
                emit loadcellStatusChanged();
            }, Qt::QueuedConnection);
        });

    sub_cal_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/loadcell/cal_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            const QString s = QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(this, [this, s]() {
                cal_status_ = s;
                emit calStatusChanged();
                if (cal_status_ == "ERROR") {
                    emit calErrorAlarm();
                } else if (cal_status_ == "DONE") {
                    emit calDoneAlarm();
                }
            }, Qt::QueuedConnection);
        });

    sub_batch_stats_ = node_->create_subscription<std_msgs::msg::String>(
        "/loadcell/batch_stats", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            QByteArray data = QByteArray::fromStdString(msg->data);
            QJsonDocument doc = QJsonDocument::fromJson(data);
            if (doc.isNull() || !doc.isObject()) return;
            QJsonObject obj = doc.object();
            const int total = obj["total"].toInt();
            const int pass  = obj["pass"].toInt();
            const int fail  = obj["fail"].toInt();
            QMetaObject::invokeMethod(this, [this, total, pass, fail]() {
                total_batch_ = total;
                pass_batch_  = pass;
                fail_batch_  = fail;
                emit batchStatsChanged();
            }, Qt::QueuedConnection);
        });

    sub_consec_fails_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/loadcell/consecutive_fails", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            const int n = msg->data;
            QMetaObject::invokeMethod(this, [this, n]() {
                consec_fails_ = n;
                emit consecFailsChanged();
            }, Qt::QueuedConnection);
        });

    sub_overload_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/loadcell/overload", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (!msg->data) return;
            QMetaObject::invokeMethod(this, [this]() {
                emit overloadAlarm();
            }, Qt::QueuedConnection);
        });

    sub_zero_drift_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/loadcell/zero_drift_warning", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            const bool current = msg->data;
            QMetaObject::invokeMethod(this, [this, current]() {
                if (current && !last_zero_drift_) {
                    emit zeroDriftAlarm();
                }
                last_zero_drift_ = current;
            }, Qt::QueuedConnection);
        });

    sub_ink_capacity_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/Fill_HP1/ink_capacity_ack", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            const float v = msg->data;
            QMetaObject::invokeMethod(this, [this, v]() {
                current_ml_fill_ = v;
                emit currentMlFillChanged();
            }, Qt::QueuedConnection);
        });

    // Services
    client_cal_start_ = node_->create_client<std_srvs::srv::Trigger>("/loadcell/cal_start");
    client_cal_set_known_ = node_->create_client<std_srvs::srv::Trigger>("/loadcell/cal_set_known");
}

void ScaleController::confirmTarget(const QString& inkName, float inkDensity, const QString& cartName, float cartDensity, float relativeError, float inkCapacity)
{
    active_ink_name_ = inkName;
    active_cart_name_ = cartName;
    ink_capacity_ = inkCapacity;
    current_ml_fill_ = inkCapacity;
    
    // Publish ml fill back to machine just in case
    auto mlMsg = std_msgs::msg::Float32();
    mlMsg.data = ink_capacity_;
    pub_ink_capacity_->publish(mlMsg);

    // Calculate total batch weight for 8 cartridges: (density * mlFill + density cartridge) * 8
    total_batch_weight_ = (inkDensity * inkCapacity + cartDensity) * 8.0f;
    min_weight_ = total_batch_weight_ - relativeError;
    max_weight_ = total_batch_weight_ + relativeError;

    emit targetChanged();
    emit inkCapacityChanged();
    emit currentMlFillChanged();

    auto fmsg = std_msgs::msg::Float32();
    fmsg.data = total_batch_weight_;
    pub_target_weight_->publish(fmsg);

    auto minMsg = std_msgs::msg::Float32();
    minMsg.data = min_weight_;
    pub_target_min_->publish(minMsg);

    auto maxMsg = std_msgs::msg::Float32();
    maxMsg.data = max_weight_;
    pub_target_max_->publish(maxMsg);

    qDebug() << "Target sent to loadcell (RevPi A):" << total_batch_weight_ 
             << "g (Min:" << min_weight_ << " Max:" << max_weight_ << ")";
}

void ScaleController::setInkCapacity(float capacity)
{
    if (ink_capacity_ != capacity) {
        ink_capacity_ = capacity;
        emit inkCapacityChanged();
        
        auto mlMsg = std_msgs::msg::Float32();
        mlMsg.data = ink_capacity_;
        pub_ink_capacity_->publish(mlMsg);
    }
}

void ScaleController::tare()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    pub_tare_cmd_->publish(msg);
    if (zero_drift_pending_) {
        zero_drift_pending_ = false;
        emit zeroDriftPendingChanged();
    }
}

void ScaleController::dismissZeroDrift()
{
    if (!zero_drift_pending_) {
        zero_drift_pending_ = true;
        emit zeroDriftPendingChanged();
    }
}

void ScaleController::resetTare()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    pub_tare_reset_->publish(msg);
}

void ScaleController::ackOverload()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    pub_overload_ack_->publish(msg);
}

void ScaleController::resetBatch()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    pub_batch_reset_->publish(msg);
}

void ScaleController::startCalibration()
{
    if (!client_cal_start_->wait_for_service(std::chrono::seconds(1))) {
        qWarning() << "cal_start service not available";
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_cal_start_->async_send_request(request);
}

void ScaleController::setLastKnownCalWeight(float weight)
{
    last_known_cal_weight_ = weight;
    emit lastKnownCalWeightChanged();
}

void ScaleController::setKnownCalibration(float weight)
{
    if (!client_cal_set_known_->service_is_ready()) {
        qWarning() << "cal_set_known service not ready!";
        return;
    }
    
    // Publish the calibration weight value first
    auto msg = std_msgs::msg::Float32();
    msg.data = weight;
    pub_cal_weight_->publish(msg);

    // Call trigger service to execute scale calibration based on that value
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_cal_set_known_->async_send_request(request);
}

QVariantList ScaleController::getInkProfiles()
{
    QVariantList list;
    QString csvPath = QString::fromLocal8Bit(qgetenv("FILL_HP_INK_CODE_FILE"));
    if (csvPath.trimmed().isEmpty()) {
        csvPath = "/home/pi/ink_codes.csv";
    }

    QFile file(csvPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return list;

    QTextStream stream(&file);
    if (stream.atEnd()) return list;

    const QStringList headers = parseCsvLine(stream.readLine());
    const int scanIdx = csvColumnIndex(headers, "scan_code");
    const int inkIdx = csvColumnIndex(headers, "ink_name");
    const int totalIdx = csvColumnIndex(headers, "total_kg");
    const int densityIdx = csvColumnIndex(headers, "density_g_ml");
    const int lotPiIdx = csvColumnIndex(headers, "lot_pi");

    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) continue;

        const QStringList row = parseCsvLine(line);
        const QString scanCode = csvValue(row, scanIdx);
        if (scanCode.isEmpty()) continue;

        const QString inkName = csvValue(row, inkIdx).isEmpty() ? scanCode : csvValue(row, inkIdx);
        const float totalKg = csvFloatValue(csvValue(row, totalIdx));
        float density = csvFloatValue(csvValue(row, densityIdx), 0.89f);
        if (density <= 0.0f) density = 0.89f;
        const QString lotPi = csvValue(row, lotPiIdx);

        QVariantMap map;
        map["scan_code"] = scanCode;
        map["name"] = inkName;
        map["ink_name"] = inkName;
        map["display"] = QString("%1 - %2").arg(scanCode, inkName);
        map["total_kg"] = totalKg;
        map["density"] = density;
        map["density_g_ml"] = density;
        map["lot_pi"] = lotPi;
        map["lot_ci"] = "";
        list.append(map);
    }
    return list;
}

QVariantList ScaleController::getCartProfiles()
{
    QVariantList list;
    const char *homedir = getenv("HOME") ? getenv("HOME") : getpwuid(getuid())->pw_dir;
    QString yaml_path = QString("%1/.ros/cartridge_profiles.yaml").arg(homedir);

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return list;
    QString content = file.readAll();
    file.close();

    QRegularExpression re("\\s*([A-Za-z0-9_ -]+):\\s*\\n\\s*density_g:\\s*([0-9.]+)");
    QRegularExpressionMatchIterator i = re.globalMatch(content);
    while (i.hasNext()) {
        QRegularExpressionMatch match = i.next();
        if (match.captured(1) == "profiles") continue;
        QVariantMap map;
        map["name"] = match.captured(1).trimmed();
        map["density"] = match.captured(2).toFloat();
        list.append(map);
    }
    return list;
}

bool ScaleController::createInkProfile(const QString& name, float density)
{
    return createInkProfileWithBatch(name, density, QString(), QString());
}

bool ScaleController::createInkProfileWithBatch(const QString& idInk, float density, const QString& lotPi, const QString& lotCi)
{
    QString name = idInk.trimmed();
    if (name.isEmpty()) return false;
    const char *homedir = getenv("HOME") ? getenv("HOME") : getpwuid(getuid())->pw_dir;
    QString yaml_path = QString("%1/.ros/ink_profiles_new.yaml").arg(homedir);

    QFile file(yaml_path);
    QString content;
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        content = QString::fromUtf8(file.readAll());
        file.close();
    }
    if (!content.contains("profiles:")) content = "profiles:\n";
    QString safeLotPi = lotPi.trimmed();
    QString safeLotCi = lotCi.trimmed();
    safeLotPi.replace('\n', ' ');
    safeLotCi.replace('\n', ' ');

    QString escaped = QRegularExpression::escape(name);
    QRegularExpression existing(QString("  %1:\\s*\\n(?:    [A-Za-z0-9_]+:\\s*[^\\n]*\\n)+").arg(escaped));
    content.remove(existing);

    QString newProfile = QString("  %1:\n    density_g: %2\n    lot_pi: %3\n    lot_ci: %4\n")
                             .arg(name,
                                  QString::number(density, 'f', 2),
                                  safeLotPi,
                                  safeLotCi);
    if (!content.endsWith('\n')) content += "\n";
    content += newProfile;

    if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        file.write(content.toUtf8());
        file.close();
        emit profilesChanged();
        return true;
    }
    return false;
}

bool ScaleController::createCartProfile(const QString& name, float density)
{
    if (name.isEmpty()) return false;
    const char *homedir = getenv("HOME") ? getenv("HOME") : getpwuid(getuid())->pw_dir;
    QString yaml_path = QString("%1/.ros/cartridge_profiles.yaml").arg(homedir);

    QFile file(yaml_path);
    QString content;
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        content = QString::fromUtf8(file.readAll());
        file.close();
    }
    if (!content.contains("profiles:")) content = "profiles:\n";
    QString newProfile = QString("  %1:\n    density_g: %2\n").arg(name, QString::number(density, 'f', 2));
    if (!content.endsWith('\n')) content += "\n";
    content += newProfile;

    if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        file.write(content.toUtf8());
        file.close();
        emit profilesChanged();
        return true;
    }
    return false;
}

bool ScaleController::deleteInkProfile(const QString& name)
{
    if (name.isEmpty()) return false;
    const char *homedir = getenv("HOME") ? getenv("HOME") : getpwuid(getuid())->pw_dir;
    QString yaml_path = QString("%1/.ros/ink_profiles_new.yaml").arg(homedir);

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    QString content = QString::fromUtf8(file.readAll());
    file.close();

    QString escaped = QRegularExpression::escape(name.trimmed());
    QRegularExpression re(QString("  %1:\\s*\\n(?:    [A-Za-z0-9_]+:\\s*[^\\n]+\\n)+").arg(escaped));
    
    QString newContent = content;
    newContent.remove(re);
    if (newContent == content) return false;

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return false;
    file.write(newContent.toUtf8());
    file.close();
    emit profilesChanged();
    return true;
}

bool ScaleController::deleteCartProfile(const QString& name)
{
    if (name.isEmpty()) return false;
    const char *homedir = getenv("HOME") ? getenv("HOME") : getpwuid(getuid())->pw_dir;
    QString yaml_path = QString("%1/.ros/cartridge_profiles.yaml").arg(homedir);

    QFile file(yaml_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    QString content = QString::fromUtf8(file.readAll());
    file.close();

    QString escaped = QRegularExpression::escape(name.trimmed());
    QRegularExpression re(QString("  %1:\\s*\\n(?:    [A-Za-z0-9_]+:\\s*[^\\n]+\\n)+").arg(escaped));
    
    QString newContent = content;
    newContent.remove(re);
    if (newContent == content) return false;

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return false;
    file.write(newContent.toUtf8());
    file.close();
    emit profilesChanged();
    return true;
}
