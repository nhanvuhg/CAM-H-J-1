#include "unified_control_gui/hp_controller.hpp"
#include <QDebug>
#include <QMetaObject>
#include <QTime>

HpController::HpController(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent), node_(node)
{
    auto qos = rclcpp::QoS(10);
    auto qos_latched = rclcpp::QoS(1).transient_local().reliable();

    // ── Publishers ───────────────────────────────────────────────────────────
    pub_manual_ = node_->create_publisher<std_msgs::msg::String>("manual_command", qos);
    pub_mode_ = node_->create_publisher<std_msgs::msg::Int32>("mode_switch", qos);
    pub_screen_control_ = node_->create_publisher<std_msgs::msg::String>("screen_control", qos);

    // String topics
    std::vector<std::string> str_topics = {
        "reconnect_cmd", "servo_command", "servo_jog", "pressure_thresholds_set",
        "cr_parameters", "error_control", "base_pwm_recommend", "parameters_control",
        "ink_batch_code"
    };
    for (const auto &topic : str_topics) {
        pub_strings_[topic] = node_->create_publisher<std_msgs::msg::String>(topic, qos);
    }

    // Int32 topics
    std::vector<std::string> int_topics = {
        "base_pwm", "chamber_vent_pwm", "cr_valve10_pwm", "cr_cycles"
    };
    for (const auto &topic : int_topics) {
        pub_ints_[topic] = node_->create_publisher<std_msgs::msg::Int32>(topic, qos);
    }

    // Float32 topics
    std::vector<std::string> float_topics = {
        "dosing_volume", "dosing_flow_rate", "dosing_loading_rate", "fill_compensation",
        "cr_volume", "cr_flow_rate", "cr_loading_rate", "tank_min", "tank_max",
        "cr_valve10_duration", "cr_valve5_duration", "cr_return_duration",
        "waste_ink_drain_duration"
    };
    for (const auto &topic : float_topics) {
        pub_floats_[topic] = node_->create_publisher<std_msgs::msg::Float32>(topic, qos);
    }

    // ── Subscribers ──────────────────────────────────────────────────────────
    sub_system_status_ = node_->create_subscription<std_msgs::msg::String>(
        "system_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (system_status_ != val) {
                    system_status_ = val;
                    emit systemStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_dosing_status_ = node_->create_subscription<std_msgs::msg::String>(
        "dosing_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (dosing_status_ != val) {
                    dosing_status_ = val;
                    emit dosingStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_fill_status_ = node_->create_subscription<std_msgs::msg::String>(
        "fill_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (fill_status_ != val) {
                    fill_status_ = val;
                    emit fillStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_ball_cycle_time_ = node_->create_subscription<std_msgs::msg::String>(
        "ball_cycle_time", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (ball_cycle_time_ != val) {
                    ball_cycle_time_ = val;
                    emit ballCycleTimeChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_fix_status_ = node_->create_subscription<std_msgs::msg::String>(
        "fix_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (fix_status_ != val) {
                    fix_status_ = val;
                    emit fixStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_cr_status_ = node_->create_subscription<std_msgs::msg::String>(
        "cr_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (cr_status_ != val) {
                    cr_status_ = val;
                    emit crStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_hw_status_ = node_->create_subscription<std_msgs::msg::String>(
        "hw_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (hw_status_ != val) {
                    hw_status_ = val;
                    emit hwStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_input_state_ = node_->create_subscription<std_msgs::msg::String>(
        "input_state", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (input_state_ != val) {
                    input_state_ = val;
                    emit inputStateChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_valve_state_ = node_->create_subscription<std_msgs::msg::String>(
        "valve_state", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (valve_state_ != val) {
                    valve_state_ = val;
                    emit valveStateChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_manual_response_ = node_->create_subscription<std_msgs::msg::String>(
        "manual_response", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (manual_response_ != val) {
                    manual_response_ = val;
                    emit manualResponseChanged();
                    if (!val.isEmpty()) {
                        addAlert("Manual Response", val, "info");
                    }
                }
            }, Qt::QueuedConnection);
        });

    // Latched topics
    sub_mode_status_ = node_->create_subscription<std_msgs::msg::String>(
        "mode_status", qos_latched, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (mode_status_ != val) {
                    mode_status_ = val;
                    emit modeStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_pressure_thresholds_ = node_->create_subscription<std_msgs::msg::String>(
        "pressure_thresholds", qos_latched, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (pressure_thresholds_ != val) {
                    pressure_thresholds_ = val;
                    emit pressureThresholdsChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_error_status_ = node_->create_subscription<std_msgs::msg::String>(
        "error_status", qos_latched, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (error_status_ != val) {
                    error_status_ = val;
                    emit errorStatusChanged();
                    if (!val.isEmpty() && val != "OK" && val != "-") {
                        addAlert("Error Alert", val, "error");
                    } else if (val == "OK") {
                        addAlert("Error Cleared", "System error has been cleared.", "info");
                    }
                }
            }, Qt::QueuedConnection);
        });

    sub_base_pwm_advice_ = node_->create_subscription<std_msgs::msg::String>(
        "base_pwm_advice", qos_latched, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (base_pwm_advice_ != val) {
                    base_pwm_advice_ = val;
                    emit basePwmAdviceChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_ink_status_ = node_->create_subscription<std_msgs::msg::String>(
        "ink_status", qos_latched, [this](const std_msgs::msg::String::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QString val = QString::fromStdString(msg->data);
                if (ink_status_ != val) {
                    ink_status_ = val;
                    emit inkStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    // Numeric topics
    sub_pressure_s1_ = node_->create_subscription<std_msgs::msg::Float32>(
        "pressure_s1", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                double val = msg->data;
                if (pressure_s1_ != val) {
                    pressure_s1_ = val;
                    emit pressureS1Changed();
                }
            }, Qt::QueuedConnection);
        });

    sub_pressure_s2_ = node_->create_subscription<std_msgs::msg::Float32>(
        "pressure_s2", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                double val = msg->data;
                if (pressure_s2_ != val) {
                    pressure_s2_ = val;
                    emit pressureS2Changed();
                }
            }, Qt::QueuedConnection);
        });

    sub_pressure_s3_ = node_->create_subscription<std_msgs::msg::Float32>(
        "pressure_s3", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                double val = msg->data;
                if (pressure_s3_ != val) {
                    pressure_s3_ = val;
                    emit pressureS3Changed();
                }
            }, Qt::QueuedConnection);
        });

    sub_servo_position_ = node_->create_subscription<std_msgs::msg::Float32>(
        "servo_position", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                double val = msg->data;
                if (servo_position_ != val) {
                    servo_position_ = val;
                    emit servoPositionChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_servo_position_raw_ = node_->create_subscription<std_msgs::msg::Float32>(
        "servo_position_raw", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                double val = msg->data;
                if (servo_position_raw_ != val) {
                    servo_position_raw_ = val;
                    emit servoPositionRawChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_pwm_debug_ = node_->create_subscription<std_msgs::msg::Float32>(
        "pwm_debug", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                double val = msg->data;
                if (pwm_debug_ != val) {
                    pwm_debug_ = val;
                    emit pwmDebugChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_base_pwm_status_ = node_->create_subscription<std_msgs::msg::Int32>(
        "base_pwm_status", qos, [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                int val = msg->data;
                if (base_pwm_status_ != val) {
                    base_pwm_status_ = val;
                    emit basePwmStatusChanged();
                }
            }, Qt::QueuedConnection);
        });

    sub_cartridge_pressures_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "cartridge_pressures", qos, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                QVariantList list;
                for (float val : msg->data) {
                    list.append(val);
                }
                cartridge_pressures_ = list;
                emit cartridgePressuresChanged();
            }, Qt::QueuedConnection);
        });

    qDebug() << "HpController initialized successfully.";
    addAlert("System Status", "HpController initialized successfully.", "info");
}

// ── Control Slots ────────────────────────────────────────────────────────────

void HpController::publishManual(const QString &name, const QString &action)
{
    auto msg = std_msgs::msg::String();
    msg.data = (name + ":" + action).toStdString();
    pub_manual_->publish(msg);
    addAlert("Manual Command", QString("%1: %2").arg(name).arg(action), "info");
}

void HpController::publishMode(int mode)
{
    auto msg = std_msgs::msg::Int32();
    msg.data = mode;
    pub_mode_->publish(msg);
    QString modeStr = (mode == 0) ? "AUTO" : (mode == 1) ? "CLEAN" : "MANUAL";
    addAlert("Mode Switch", QString("Change mode to %1").arg(modeStr), "info");
}

void HpController::publishScreenControl(const QString &action)
{
    auto msg = std_msgs::msg::String();
    msg.data = action.toStdString();
    pub_screen_control_->publish(msg);
    addAlert("Screen Control", QString("Command: %1").arg(action.toUpper()), "info");
}

void HpController::publishInt(const QString &topic, int value)
{
    auto it = pub_ints_.find(topic.toStdString());
    if (it != pub_ints_.end()) {
        auto msg = std_msgs::msg::Int32();
        msg.data = value;
        it->second->publish(msg);
        addAlert("Parameter Update", QString("%1 = %2").arg(topic).arg(value), "info");
    } else {
        qWarning() << "Unsupported HP int topic:" << topic;
    }
}

void HpController::publishFloat(const QString &topic, double value)
{
    auto it = pub_floats_.find(topic.toStdString());
    if (it != pub_floats_.end()) {
        auto msg = std_msgs::msg::Float32();
        msg.data = static_cast<float>(value);
        it->second->publish(msg);
        addAlert("Parameter Update", QString("%1 = %2").arg(topic).arg(value), "info");
    } else {
        qWarning() << "Unsupported HP float topic:" << topic;
    }
}

void HpController::publishString(const QString &topic, const QString &value)
{
    auto it = pub_strings_.find(topic.toStdString());
    if (it != pub_strings_.end()) {
        auto msg = std_msgs::msg::String();
        msg.data = value.toStdString();
        it->second->publish(msg);
        addAlert("Parameter Update", QString("%1 = %2").arg(topic).arg(value), "info");
    } else {
        qWarning() << "Unsupported HP string topic:" << topic;
    }
}

void HpController::addAlert(const QString &title, const QString &text, const QString &sev)
{
    QVariantMap item;
    item["title"] = title;
    item["text"] = text;
    item["sev"] = sev;
    item["time"] = QTime::currentTime().toString("HH:mm:ss");

    alert_history_.prepend(item);
    while (alert_history_.size() > 50) {
        alert_history_.removeLast();
    }
    emit alertHistoryChanged();
}
