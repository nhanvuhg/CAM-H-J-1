#ifndef CARTRIDGE_SYSTEM_HPP
#define CARTRIDGE_SYSTEM_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>

namespace system_feed_cartridge {

enum class SystemState {
    IDLE,
    HOMING,
    WAIT_FOR_TRAY,
    CONVEYOR_RUNNING,
    INX_MOVING_TO_TARGET2,
    INY_SEARCHING,
    INY_MOVING_TO_ROW,
    CYLINDER1_EXTEND,
    INY_RETURN_HOME,
    INY_MOVING_TO_TARGET2_RETURN,
    CYLINDER1_RETRACT,
    INX_RETURN_HOME,
    TRAY_LOADED,
    
    // State 2: Retrieve processed tray
    WAIT_FOR_DONE_TRAY,
    RETRIEVE_INX_TO_TARGET2,
    RETRIEVE_INY_TO_TARGET2,
    RETRIEVE_CYLINDER_EXTEND,
    RETRIEVE_INY_TO_TARGET1,
    RETRIEVE_INX_TO_TARGET1,
    RETRIEVE_INY_SEARCH_ROW,
    RETRIEVE_INY_TO_ROW,
    RETRIEVE_CYLINDER_RETRACT,
    RETRIEVE_INY_RETURN,
    
    // State 3: Output tray
    WAIT_FOR_OUTPUT_SIGNAL,
    OUTPUT_SERVO3_PUSH_TRAY,
    OUTPUT_WAIT_CARTRIDGE_FILL,
    OUTPUT_OUTX_TO_TARGET2,
    OUTPUT_OUTY_TO_TARGET2,
    OUTPUT_CYLINDER_EXTEND,
    OUTPUT_OUTY_TO_TARGET1,
    OUTPUT_OUTX_TO_TARGET1,
    OUTPUT_OUTY_SEARCH_ROW,
    OUTPUT_OUTY_TO_ROW,
    OUTPUT_CYLINDER_RETRACT,
    OUTPUT_OUTY_TO_HOME,
    OUTPUT_SERVO3_RETRACT,
    OUTPUT_COMPLETE,
    
    ERROR
};

// Convert state to string for logging
std::string stateToString(SystemState state);

struct Config {
    // Hardware IPs
    std::map<int, std::string> servo_ips;
    std::string io_ip;

    // Cylinder channels
    int cylinder1_extend_channel;
    int cylinder1_retract_channel;
    int cylinder2_extend_channel;
    int cylinder2_retract_channel;

    // Positions
    double inx_home;
    double inx_target2;
    double iny_home;
    double iny_target2;
    double servo3_home;
    double outx_home;
    double outx_target1;
    double outx_target2;
    double outy_home;
    double outy_target1;
    double outy_target2;
    
    std::map<int, double> row_positions;

    // Safety & Tolerances
    double position_tolerance;
    double iny_safe_position_threshold;
    double outx_safe_position_threshold;

    // Velocities
    double servo3_jog_velocity;
    double iny_search_velocity;
    
    // Timeouts
    double homing_timeout;
    double move_timeout;
    double cylinder_timeout;
    
    void loadFromFile(const std::string& config_file);
};

class CartridgeSystem : public rclcpp::Node {
public:
    explicit CartridgeSystem(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CartridgeSystem();

private:
    // ROS Communication
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_new_tray_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_run_conveyor_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_last_batch_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_button_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_done_tray_input_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_done_tray_output_;
    
    // Vision node tray change subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_vision_change_input_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_vision_change_output_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    // Configuration
    Config config_;
    
    // State Machine
    SystemState current_state_;
    std::mutex state_mutex_;
    void processState();
    void changeState(SystemState new_state);
    
    // Callbacks
    void startButtonCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void doneTrayInputCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void doneTrayOutputCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void timerCallback();

    // Logic Variables
    bool done_tray_signal_;
    bool done_output_signal_;
    bool start_button_pressed_;
    
    int detected_trays_;
    int current_input_row_;
    int current_output_row_;
    int current_slot_;
    int max_trays_;
    int max_slots_per_output_tray_;
    
    // Hardware Abstraction (Mocked for now)
    bool connectHardware();
    bool homeAllServos();
    bool moveServo(int servo_id, double position, bool wait = true);
    double getServoPosition(int servo_id);
    void setCylinder(int cylinder_id, bool extend); // 1 or 2
    bool getSensor(int sensor_id);
    
    // Safety Checks
    bool isInYSafeForInXMove();
    bool isOutXSafeForOutYMove();
    
    // Simulated/Mocked hardware state (remove when real hardware lib is available)
    std::map<int, double> mocked_servo_positions_;
    std::map<int, bool> mocked_sensors_;
    
    // Helper to simulate sensor updates based on state/position
    void updateMockHardware(); 
};

} // namespace system_feed_cartridge

#endif // CARTRIDGE_SYSTEM_HPP
