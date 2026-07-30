#ifndef HP_CONTROLLER_HPP
#define HP_CONTROLLER_HPP

#include <QObject>
#include <QString>
#include <QVariantList>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class HpController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString systemStatus READ systemStatus NOTIFY systemStatusChanged)
    Q_PROPERTY(QString inkStatus READ inkStatus NOTIFY inkStatusChanged)
    Q_PROPERTY(QString dosingStatus READ dosingStatus NOTIFY dosingStatusChanged)
    Q_PROPERTY(QString fillStatus READ fillStatus NOTIFY fillStatusChanged)
    Q_PROPERTY(QString ballCycleTime READ ballCycleTime NOTIFY ballCycleTimeChanged)
    Q_PROPERTY(QString fixStatus READ fixStatus NOTIFY fixStatusChanged)
    Q_PROPERTY(QString crStatus READ crStatus NOTIFY crStatusChanged)
    Q_PROPERTY(QString hwStatus READ hwStatus NOTIFY hwStatusChanged)
    Q_PROPERTY(QString inputState READ inputState NOTIFY inputStateChanged)
    Q_PROPERTY(QString valveState READ valveState NOTIFY valveStateChanged)
    Q_PROPERTY(QString manualResponse READ manualResponse NOTIFY manualResponseChanged)
    Q_PROPERTY(QString modeStatus READ modeStatus NOTIFY modeStatusChanged)
    Q_PROPERTY(QString pressureThresholds READ pressureThresholds NOTIFY pressureThresholdsChanged)
    Q_PROPERTY(QString errorStatus READ errorStatus NOTIFY errorStatusChanged)
    Q_PROPERTY(QString basePwmAdvice READ basePwmAdvice NOTIFY basePwmAdviceChanged)
    Q_PROPERTY(double pressureS1 READ pressureS1 NOTIFY pressureS1Changed)
    Q_PROPERTY(double pressureS2 READ pressureS2 NOTIFY pressureS2Changed)
    Q_PROPERTY(double pressureS3 READ pressureS3 NOTIFY pressureS3Changed)
    Q_PROPERTY(double servoPosition READ servoPosition NOTIFY servoPositionChanged)
    Q_PROPERTY(double servoPositionRaw READ servoPositionRaw NOTIFY servoPositionRawChanged)
    Q_PROPERTY(double pwmDebug READ pwmDebug NOTIFY pwmDebugChanged)
    Q_PROPERTY(int basePwmStatus READ basePwmStatus NOTIFY basePwmStatusChanged)
    Q_PROPERTY(QVariantList cartridgePressures READ cartridgePressures NOTIFY cartridgePressuresChanged)
    Q_PROPERTY(QVariantList alertHistory READ alertHistory NOTIFY alertHistoryChanged)

public:
    explicit HpController(rclcpp::Node::SharedPtr node, QObject *parent = nullptr);

    // Getters
    QString systemStatus() const { return system_status_; }
    QString inkStatus() const { return ink_status_; }
    QString dosingStatus() const { return dosing_status_; }
    QString fillStatus() const { return fill_status_; }
    QString ballCycleTime() const { return ball_cycle_time_; }
    QString fixStatus() const { return fix_status_; }
    QString crStatus() const { return cr_status_; }
    QString hwStatus() const { return hw_status_; }
    QString inputState() const { return input_state_; }
    QString valveState() const { return valve_state_; }
    QString manualResponse() const { return manual_response_; }
    QString modeStatus() const { return mode_status_; }
    QString pressureThresholds() const { return pressure_thresholds_; }
    QString errorStatus() const { return error_status_; }
    QString basePwmAdvice() const { return base_pwm_advice_; }
    double pressureS1() const { return pressure_s1_; }
    double pressureS2() const { return pressure_s2_; }
    double pressureS3() const { return pressure_s3_; }
    double servoPosition() const { return servo_position_; }
    double servoPositionRaw() const { return servo_position_raw_; }
    double pwmDebug() const { return pwm_debug_; }
    int basePwmStatus() const { return base_pwm_status_; }
    QVariantList cartridgePressures() const { return cartridge_pressures_; }
    QVariantList alertHistory() const { return alert_history_; }

    void addAlert(const QString &title, const QString &text, const QString &sev = "info");

public slots:
    // Control interfaces
    void publishManual(const QString &name, const QString &action);
    void publishMode(int mode);
    void publishScreenControl(const QString &action);
    void publishInt(const QString &topic, int value);
    void publishFloat(const QString &topic, double value);
    void publishString(const QString &topic, const QString &value);

signals:
    void systemStatusChanged();
    void inkStatusChanged();
    void dosingStatusChanged();
    void fillStatusChanged();
    void ballCycleTimeChanged();
    void fixStatusChanged();
    void crStatusChanged();
    void hwStatusChanged();
    void inputStateChanged();
    void valveStateChanged();
    void manualResponseChanged();
    void modeStatusChanged();
    void pressureThresholdsChanged();
    void errorStatusChanged();
    void basePwmAdviceChanged();
    void pressureS1Changed();
    void pressureS2Changed();
    void pressureS3Changed();
    void servoPositionChanged();
    void servoPositionRawChanged();
    void pwmDebugChanged();
    void basePwmStatusChanged();
    void cartridgePressuresChanged();
    void alertHistoryChanged();

private:
    rclcpp::Node::SharedPtr node_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_manual_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_mode_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_screen_control_;
    
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pub_strings_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> pub_ints_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> pub_floats_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_system_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_dosing_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_fill_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ball_cycle_time_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_fix_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cr_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_hw_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_input_state_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_valve_state_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_manual_response_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_pressure_thresholds_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_error_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_base_pwm_advice_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ink_status_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_pressure_s1_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_pressure_s2_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_pressure_s3_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_servo_position_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_servo_position_raw_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_pwm_debug_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_base_pwm_status_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_cartridge_pressures_;

    // Local state variables
    QString system_status_{""};
    QString dosing_status_{""};
    QString fill_status_{""};
    QString ball_cycle_time_{""};
    QString fix_status_{""};
    QString cr_status_{""};
    QString hw_status_{""};
    QString input_state_{""};
    QString valve_state_{""};
    QString manual_response_{""};
    QString mode_status_{""};
    QString pressure_thresholds_{""};
    QString error_status_{""};
    QString base_pwm_advice_{""};
    QString ink_status_{""};
    double pressure_s1_{0.0};
    double pressure_s2_{0.0};
    double pressure_s3_{0.0};
    double servo_position_{0.0};
    double servo_position_raw_{0.0};
    double pwm_debug_{0.0};
    int base_pwm_status_{0};
    QVariantList cartridge_pressures_;
    QVariantList alert_history_;
};

#endif // HP_CONTROLLER_HPP
