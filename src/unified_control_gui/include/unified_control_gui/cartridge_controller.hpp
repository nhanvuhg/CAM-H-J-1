#ifndef CARTRIDGE_CONTROLLER_HPP
#define CARTRIDGE_CONTROLLER_HPP

#include <QObject>
#include <QString>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <memory>

class CartridgeController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString systemState      READ systemState      NOTIFY systemStateChanged)
    Q_PROPERTY(QString currentMode      READ currentMode      NOTIFY currentModeChanged)
    Q_PROPERTY(QString servoPositions   READ servoPositions   NOTIFY servoPositionsChanged)
    Q_PROPERTY(QString configData       READ configData       NOTIFY configDataChanged)
    Q_PROPERTY(QString lastNotification READ lastNotification NOTIFY notificationReceived)
    Q_PROPERTY(QVariantList logEntries  READ logEntries       NOTIFY logEntriesChanged)
    Q_PROPERTY(QString sensorState      READ sensorState      NOTIFY sensorStateChanged)
    Q_PROPERTY(QString stateIn          READ stateIn          NOTIFY systemStateChanged)
    Q_PROPERTY(QString stateOut         READ stateOut         NOTIFY systemStateChanged)
    Q_PROPERTY(QString uiHint           READ uiHint           NOTIFY uiHintChanged)

public:
    explicit CartridgeController(rclcpp::Node::SharedPtr node, QObject *parent = nullptr);

    QString systemState()      const { return system_state_; }
    QString currentMode()      const { return current_mode_; }
    QString servoPositions()   const { return servo_positions_; }
    QString configData()       const { return config_data_; }
    QString lastNotification() const { return last_notification_; }
    QVariantList logEntries()  const { return log_entries_; }
    QString sensorState()      const { return sensor_state_; }
    QString stateIn()          const { return state_in_; }
    QString stateOut()         const { return state_out_; }
    QString uiHint()           const { return ui_hint_; }

public slots:
    // Servo control
    void jogServo(int id, const QString &dir, int velocity);
    void jogStop(int id);
    void homeServo(int id);
    void clearServo(int id);
    void moveServo(int id, double position);
    Q_INVOKABLE void setJogVelocity(const QString &velocity_ms);

    // System control
    void setMode(const QString &mode);
    void gotoState(const QString &state);
    void setTargetRow(int row);
    void startSystem();
    void stopSystem();
    void softStop();          // Soft STOP — keep state + CPX, only switch to MANUAL
    void pauseSystem();    // Graceful: cartridge + robot dừng tại ranh giới state
    void resumeSystem();   // Resume cả cartridge + robot, tiếp tục state hiện tại
    void hmiResume();      // Legacy no-op (giữ để tương thích QML cũ)
    void resetFaults();
    void abortToJog();
    Q_INVOKABLE void simulateDoneTrayInput();
    Q_INVOKABLE void simulateDoneTrayOutput();
    void confirmOutput();

    // Cylinder manual control (MANUAL + IDLE only) — cyl1: extend/retract, cyl2: extend/retract
    Q_INVOKABLE void cylinderCmd(int cylId, bool extend);

    // Config
    void getConfig();
    void saveConfig(const QString &key, const QString &jsonData);

    // Log
    void clearLog();

signals:
    void systemStateChanged();
    void currentModeChanged();
    void servoPositionsChanged();
    void configDataChanged();
    void notificationReceived();
    void logEntriesChanged();
    void sensorStateChanged();
    void uiHintChanged();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gui_confirm_pub_;
    rclcpp::Node::SharedPtr node_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jog_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_to_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goto_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_target_row_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cyl_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reset_faults_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_config_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr update_config_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   hmi_resume_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   start_button_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   stop_button_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   soft_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   pause_button_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   resume_button_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr   robot_pause_client_;  // /robot/pause_system (sync)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   confirm_button_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   done_tray_input_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   done_tray_output_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_data_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gui_notify_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensors_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_mode_sub_;

    // State
    QString system_state_{"UNKNOWN"};
    QString state_in_;
    QString state_out_;
    QString current_mode_{"idle"};
    QString servo_positions_{"{}"};
    QString config_data_{"{}"};
    QString last_notification_;
    QVariantList log_entries_;
    QString sensor_state_{"000000000000000000"};
    QString ui_hint_;                // 'press_homing' | 'press_stop' | 'switch_manual' | ''
    QTimer* ui_hint_timer_{nullptr}; // auto-clear hint sau 5s
    QString last_goto_state_;
    qint64 last_goto_state_ms_{0};

    void publishString(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub, const QString &data);
    void publishBool(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, bool value);
    void addLog(const QString &msg, const QString &type = "info");
};

#endif // CARTRIDGE_CONTROLLER_HPP
