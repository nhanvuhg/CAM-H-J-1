#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dobot_msgs_v3/msg/tool_vector_actual.hpp"
#include "dobot_msgs_v3/srv/move_jog.hpp"
#include "dobot_msgs_v3/srv/get_angle.hpp"
#include "dobot_msgs_v3/srv/get_pose.hpp"
#include "dobot_msgs_v3/srv/joint_mov_j.hpp"
#include "dobot_msgs_v3/srv/mov_l.hpp"
#include "dobot_msgs_v3/srv/servo_j.hpp"
#include "dobot_msgs_v3/srv/servo_p.hpp"
#include "dobot_msgs_v3/srv/do.hpp"
#include "dobot_msgs_v3/srv/pause.hpp"
#include "dobot_msgs_v3/srv/continues.hpp"
#include "dobot_msgs_v3/srv/emergency_stop.hpp"
#include "dobot_msgs_v3/srv/stop_script.hpp"
#include "dobot_msgs_v3/srv/disable_robot.hpp"
#include "dobot_msgs_v3/srv/enable_robot.hpp"
#include "dobot_msgs_v3/srv/clear_error.hpp"
#include "dobot_msgs_v3/srv/reset_robot.hpp"
#include "dobot_msgs_v3/srv/speed_factor.hpp"
#include "dobot_msgs_v3/srv/get_error_id.hpp"
#include <memory>
#include <mutex>


class RobotController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString systemStatus READ systemStatus NOTIFY systemStatusChanged)
    Q_PROPERTY(QString errorMessage READ errorMessage NOTIFY errorMessageChanged)
    Q_PROPERTY(int selectedRow READ selectedRow NOTIFY selectedRowChanged)
    Q_PROPERTY(int selectedSlot READ selectedSlot NOTIFY selectedSlotChanged)
    Q_PROPERTY(QString systemUptime READ systemUptime NOTIFY systemUptimeChanged)
    Q_PROPERTY(int trayCount READ trayCount NOTIFY trayCountChanged)
    // Jog control properties
    Q_PROPERTY(QVariantList jointAngles READ jointAngles NOTIFY jointAnglesChanged)
    Q_PROPERTY(QVariantList cartesianPose READ cartesianPose NOTIFY cartesianPoseChanged)
    Q_PROPERTY(int speedRatio READ speedRatio NOTIFY speedRatioChanged)
    Q_PROPERTY(int hwSpeedRatio READ hwSpeedRatio NOTIFY hwSpeedRatioChanged)
    Q_PROPERTY(bool jogContinuous READ jogContinuous WRITE setJogContinuous NOTIFY jogContinuousChanged)
    Q_PROPERTY(double jogStepSize READ jogStepSize WRITE setJogStepSize NOTIFY jogStepSizeChanged)
    Q_PROPERTY(bool inReady READ inReady NOTIFY inReadyChanged)
    Q_PROPERTY(bool outReady READ outReady NOTIFY outReadyChanged)
    Q_PROPERTY(bool ignoreScale READ ignoreScale WRITE setIgnoreScale NOTIFY ignoreScaleChanged)
    Q_PROPERTY(QVariantList rowReady READ rowReady NOTIFY rowReadyChanged)
    Q_PROPERTY(QVariantList slotReady READ slotReady NOTIFY slotReadyChanged)
    Q_PROPERTY(QString errorLog READ errorLog NOTIFY errorLogChanged)
    // Rong = ROI OK. Non-empty = vision decision dang bi khoa/thieu ROI.
    Q_PROPERTY(QString roiError READ roiError NOTIFY roiErrorChanged)
    Q_PROPERTY(bool gripperOn READ gripperOn NOTIFY gripperOnChanged)
    Q_PROPERTY(bool pickerOn READ pickerOn NOTIFY pickerOnChanged)
    Q_PROPERTY(bool cylLoadcellOn READ cylLoadcellOn NOTIFY cylLoadcellOnChanged)

public:
    explicit RobotController(rclcpp::Node::SharedPtr node, QObject *parent = nullptr);
    
    QString systemStatus() const { return system_status_; }
    QString errorMessage() const { return error_message_; }
    QString roiError() const { return roi_error_; }
    int selectedRow() const { return selected_row_; }
    int selectedSlot() const { return selected_slot_; }
    QString systemUptime() const { return system_uptime_; }
    int trayCount() const { return tray_count_; }
    QVariantList jointAngles() const { return joint_angles_; }
    QVariantList cartesianPose() const { return cartesian_pose_; }
    int speedRatio() const { return speed_ratio_; }
    int hwSpeedRatio() const { return hw_speed_ratio_; }
    QString errorLog() const { return error_log_; }
    bool jogContinuous() const { return jog_continuous_; }
    double jogStepSize() const { return jog_step_size_; }
    bool inReady() const { return in_ready_; }
    bool outReady() const { return out_ready_; }
    bool ignoreScale() const { return ignore_scale_; }
    QVariantList rowReady() const { return row_ready_; }
    QVariantList slotReady() const { return slot_ready_; }
    bool gripperOn() const { return last_gripper_state_; }
    bool pickerOn() const { return last_picker_state_; }
    bool cylLoadcellOn() const { return last_cyl_loadcell_state_; }

public slots:
    // System control
    void enableSystem(bool enable);
    void stopAndResetRobot();  // Reset motion command, disable Dobot, switch MANUAL
    void softStopAndManual();  // Soft STOP: Pause Dobot + cancel motion + switch MANUAL (keep state)
    void startSystem(bool start);
    void emergencyStop(bool stop);
    void setManualMode(bool enable);
    void setAiMode(bool enable);
    void setAutoMode(bool enable);
    void switchCamera(int cameraId);
    void selectRow(int row);
    void selectSlot(int slot);
    void gotoState(const QString& state);
    
    // Jog control (Manual mode)
    void jogStart(const QString& axisId);  // "j1+" "j1-" "x+" "z-" etc
    void jogStep(const QString& axisId, double stepSize);  // one relative step (mm or deg)
    void jogStop();
    void stopMotionOnly();
    void sendJogStep();  // callback-chaining step for continuous JOG
    void sendCartesianStep();  // ServoP streaming tick for Cartesian jog
    void setJogContinuous(bool continuous);
    void setJogStepSize(double size);
    void getAngles();
    void getPose();
    
    // Move to exact position
    void moveJoint(double j1, double j2, double j3, double j4, double j5, double j6);
    void moveLinear(double x, double y, double z, double rx, double ry, double rz);

    // IMPORTANT — KEEP SEND QUEUE-FREE:
    // SEND is hold-to-move (like JOG): stream ServoP/ServoJ toward an absolute
    // target while the button is held; release stops the stream so the robot
    // halts in place. Do NOT replace this with MovL/JointMovJ. On the deployed
    // Nova firmware those are queued commands: release/ResetRobot/DisableRobot
    // do not reliably discard the target, and a later ENABLE/Continue can make
    // the robot resume the old point unexpectedly.
    Q_INVOKABLE void startSendMoveL(double x, double y, double z, double rx, double ry, double rz);
    Q_INVOKABLE void startSendMoveJ(double j1, double j2, double j3, double j4, double j5, double j6);
    Q_INVOKABLE void stopSendMove();
    void saveJointPose(const QString& name, double j1, double j2, double j3, double j4, double j5, double j6);
    Q_INVOKABLE QVariantList getSavedPoses();
    
    // IO control
    void setDigitalOutput(int index, bool status);
    
    // Robot control
    void pauseRobot();
    void resumeRobot();
    void clearError();
    void setSpeedRatio(int ratio);
    void pollErrorID();
    
    // Simulation triggers
    void simulateFeedChamber();
    void simulateFillDone();
    void simulateInputTrayReady();
    void simulateOutputTrayReady();
    void setIgnoreScale(bool ignore);

    // Scale result
    void publishScaleResult(bool pass);

    Q_INVOKABLE QString captureScreenshot();
    Q_INVOKABLE QString restartSystemNodes();
    Q_INVOKABLE QString restartGui();

signals:
    void systemStatusChanged();
    void errorMessageChanged();
    void selectedRowChanged();
    void selectedSlotChanged();
    void systemUptimeChanged();
    void trayCountChanged();
    void serviceCallResult(bool success, QString message);
    void jointAnglesChanged();
    void cartesianPoseChanged();
    void speedRatioChanged();
    void hwSpeedRatioChanged();
    void errorLogChanged();
    void jogContinuousChanged();
    void jogStepSizeChanged();
    void inReadyChanged();
    void outReadyChanged();
    void ignoreScaleChanged();
    void rowReadyChanged();
    void slotReadyChanged();
    void roiErrorChanged();
    void jointPoseSaved(bool success, QString message);
    void gripperOnChanged();
    void pickerOnChanged();
    void cylLoadcellOnChanged();

private:
    rclcpp::Node::SharedPtr node_;
    
    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_system_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_stop_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_system_client_;  // NEW: /robot/pause_system
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr manual_mode_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ai_mode_client_;
    
    // Dobot service clients (jog)
    rclcpp::Client<dobot_msgs_v3::srv::MoveJog>::SharedPtr jog_client_;
    rclcpp::Client<dobot_msgs_v3::srv::GetAngle>::SharedPtr get_angle_client_;
    rclcpp::Client<dobot_msgs_v3::srv::GetPose>::SharedPtr get_pose_client_;
    rclcpp::Client<dobot_msgs_v3::srv::JointMovJ>::SharedPtr joint_movj_client_;
    rclcpp::Client<dobot_msgs_v3::srv::MovL>::SharedPtr movl_client_;
    rclcpp::Client<dobot_msgs_v3::srv::ServoJ>::SharedPtr servo_j_client_;
    rclcpp::Client<dobot_msgs_v3::srv::ServoP>::SharedPtr servo_p_client_;
    rclcpp::Client<dobot_msgs_v3::srv::DO>::SharedPtr do_client_;
    rclcpp::Client<dobot_msgs_v3::srv::Pause>::SharedPtr pause_client_;
    rclcpp::Client<dobot_msgs_v3::srv::Continues>::SharedPtr continue_client_;
    rclcpp::Client<dobot_msgs_v3::srv::EmergencyStop>::SharedPtr dobot_emergency_stop_client_;
    rclcpp::Client<dobot_msgs_v3::srv::StopScript>::SharedPtr dobot_stop_script_client_;
    rclcpp::Client<dobot_msgs_v3::srv::DisableRobot>::SharedPtr disable_robot_client_;
    rclcpp::Client<dobot_msgs_v3::srv::EnableRobot>::SharedPtr dobot_enable_robot_client_;
    rclcpp::Client<dobot_msgs_v3::srv::ClearError>::SharedPtr clear_error_client_;

    // [STOP-PRESERVE] Snapshot/restore picker+gripper state khi STOP de dam bao
    // CPX coil khong bi reset boi side-effect cua Pause/ResetRobot/EnableRobot.
    // (gripper_cmd_pub_ + picker_cmd_pub_ da duoc declare o tren — dung lai)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr picker_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cyl_loadcell_status_sub_;
    bool last_gripper_state_{false};
    bool last_picker_state_{false};
    // DO6 wiring convention: true = RELEASING. Use the physical default for
    // the initial GUI indication until the first status feedback arrives.
    bool last_cyl_loadcell_state_{true};
    rclcpp::Client<dobot_msgs_v3::srv::ResetRobot>::SharedPtr reset_robot_client_;
    rclcpp::Client<dobot_msgs_v3::srv::SpeedFactor>::SharedPtr speed_factor_client_;
    rclcpp::Client<dobot_msgs_v3::srv::GetErrorID>::SharedPtr get_error_id_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr reset_state_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr soft_stop_to_manual_client_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr camera_select_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_row_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_slot_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goto_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr set_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feed_chamber_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fill_done_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr input_tray_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr output_tray_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scale_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_ratio_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr system_start_pub_;  // /system/start_button — shared with cartridge
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr system_pause_pub_;  // /system/pause_button — sync sang cartridge
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr system_resume_pub_; // /system/resume_button — sync sang cartridge
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ignore_scale_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr picker_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cyl_loadcell_cmd_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr error_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr roi_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_slot_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_uptime_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tray_count_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr in_ready_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr out_ready_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hw_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr row_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr slot_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<dobot_msgs_v3::msg::ToolVectorActual>::SharedPtr tool_vector_sub_;
    
    // State
    QString system_status_{"UNKNOWN"};
    QString error_message_;
    int selected_row_{-1};
    int selected_slot_{-1};
    QString system_uptime_{"00:00:00"};
    int tray_count_{0};
    QVariantList joint_angles_;
    QVariantList cartesian_pose_;
    int speed_ratio_{100};
    int hw_speed_ratio_{0};
    QString error_log_;
    QString roi_error_;
    QTimer *poll_timer_;
    
    // JOG state
    bool jog_continuous_{true};    // true=continuous, false=step
    double jog_step_size_{1.0};   // step size (degrees or mm)
    
    bool in_ready_{false};
    bool out_ready_{false};
    bool ignore_scale_{false};
    QVariantList row_ready_;
    QVariantList slot_ready_;
    
    QTimer *jog_timer_{nullptr};
    QString jog_axis_;
    bool jog_moving_{false};
    std::chrono::steady_clock::time_point jog_start_time_;
    int jog_cart_idx_{-1};
    bool jog_cart_positive_{true};
    double jog_cart_target_[6]{0};

    // Queue-free ServoP/ServoJ stream used by the press-and-hold SEND buttons.
    QTimer *send_timer_{nullptr};
    bool send_point_motion_active_{false};
    bool send_button_held_{false};
    int send_motion_kind_{0};  // 0=none, 1=Cartesian ServoP, 2=joint ServoJ
    double send_current_[6]{0};
    double send_target_[6]{0};

    void callServiceAsync(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value);
    void pollRobotState();
    void sendMoveJog(const QString& axisId);  // native MoveJog for continuous
    void stopManualJogMotion();
    void sendHoldStep();

    // Thread-safe high-frequency GUI updates
    std::mutex status_mutex_;
    QVariantList next_joint_angles_;
    QVariantList next_cartesian_pose_;
    bool has_new_joints_{false};
    bool has_new_pose_{false};
};

#endif // ROBOT_CONTROLLER_HPP
