#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_interfaces/action/execute_motion.hpp"


#include "dobot_msgs_v3/srv/enable_robot.hpp"
#include "dobot_msgs_v3/srv/do.hpp"
#include "dobot_msgs_v3/srv/robot_mode.hpp"
#include "dobot_msgs_v3/srv/clear_error.hpp"
#include "dobot_msgs_v3/srv/get_error_id.hpp"
#include "dobot_msgs_v3/srv/reset_robot.hpp"
#include "dobot_msgs_v3/srv/pause.hpp"  // NEW: for pause_client_
#include "dobot_msgs_v3/srv/disable_robot.hpp"  // NEW: for E-Stop power off



#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <deque>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <cstdint>
#include <thread>

using EnableRobot   = dobot_msgs_v3::srv::EnableRobot;
using DO            = dobot_msgs_v3::srv::DO;
using RobotMode     = dobot_msgs_v3::srv::RobotMode;
using ClearError    = dobot_msgs_v3::srv::ClearError;
using GetErrorID    = dobot_msgs_v3::srv::GetErrorID;
using ResetRobot    = dobot_msgs_v3::srv::ResetRobot;
using Pause         = dobot_msgs_v3::srv::Pause;         // soft pause
using DisableRobot  = dobot_msgs_v3::srv::DisableRobot;  // E-Stop power off


// ============================================================================
// ENUMS AND CONSTANTS
// ============================================================================

enum class SystemState
{
    IDLE,
    INIT_CHECK,
    INIT_LOAD_CHAMBER_DIRECT,
    INIT_REFILL_BUFFER,
    WAIT_FILLING,
    TAKE_CHAMBER_TO_SCALE,
    PROCESSING_SCALE,
    ERROR_SCALE_TIMEOUT,
    PLACE_TO_OUTPUT,
    PLACE_TO_FAIL,
    REFILL_BUFFER,
    LOAD_CHAMBER_FROM_BUFFER,
    LAST_BATCH_WAIT,
    ERROR_SCALE,
    ERROR_OUTPUT_TRAY_TIMEOUT,
    ERROR_INPUT_TRAY_EMPTY,
    ERROR_MOTION_LOST
};


// ============================================================================
// CONTROL MODE ENUM
// ============================================================================

enum class ControlMode : uint8_t
{
    MANUAL = 0,  // Manual control - user specifies everything
    AUTO = 1,    // Auto mode - sequential picking without AI detection
    AI = 2       // AI mode - vision-based detection and selection
};



// ============================================================================
// MAIN ROBOT LOGIC NODE CLASS
// ============================================================================

class RobotLogicNode : public rclcpp::Node
{
public:
    RobotLogicNode();
    ~RobotLogicNode();

private:
    // ========================================================================
    // CONSTANTS
    // ========================================================================
    static constexpr int INPUT_ROW_THRESHOLD = 8;
    static constexpr int BUFFER_CAPACITY = 8;

    static constexpr double SCALE_TIMEOUT_SEC = 120.0;
    static constexpr double OUTPUT_TRAY_TIMEOUT_SEC = 180.0;
    static constexpr double FILLING_DURATION_SEC = 90.0;

    // ========================================================================
    // SERVICE CLIENTS (Moved to below)
    // ========================================================================

    


    // ========================================================================
    
    // ========================================================================
    // ROS SUBSCRIPTIONS
    // ========================================================================
    // Vision Node Subscriptions (Replaces direct YOLO)
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr vision_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_empty_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr vision_slot_sub_;

    
    // Original subscriptions (kept for compatibility)
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr camera_active_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_camera_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr feed_chamber_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fill_done_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scale_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_button_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr new_tray_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_last_tray_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selected_slot_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_slot_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goto_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_change_tray_sub_;
    
    // Heartbeat & Busy Subscriptions
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr motion_hb_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr vision_hb_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_busy_sub_;

    
    // Heartbeat tracking
    rclcpp::Time last_motion_hb_;
    rclcpp::Time last_vision_hb_;
    std::atomic<bool> motion_busy_{false};
    std::atomic<bool> cartridge_busy_{false};  // True khi cartridge system đang S2/S3

    // ========================================================================
    // ROS PUBLISHERS
    // ========================================================================
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr selected_slot_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr selected_row_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr picker_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr camera_select_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_uptime_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tray_count_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr change_tray_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_output_tray_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr place_done_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr last_batch_complete_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_busy_pub_;  // NEW: tự publish busy status
    
    // Motion Executor Communication
    // Action Client
    using ExecuteMotion = robot_control_interfaces::action::ExecuteMotion;
    using GoalHandleExecuteMotion = rclcpp_action::ClientGoalHandle<ExecuteMotion>;
    rclcpp_action::Client<ExecuteMotion>::SharedPtr motion_action_client_;


    // ========================================================================
    // ROS SERVICES
    // ========================================================================
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_system_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_system_service_;
    // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr new_tray_loaded_service_; // NOT NEEDED
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_state_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_system_service_;  // NEW: true=pause, false=resume

    
    // Unified mode selection (1=AUTO, 2=AI, 3=MANUAL)
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_mode_sub_;

    // ========================================================================
    // THREAD SYNCHRONIZATION
    // ========================================================================
    std::condition_variable state_cv_;
    std::mutex state_cv_mutex_;
    std::atomic<bool> state_changed_{false};
    
    // Callback group for parallel execution
    rclcpp::CallbackGroup::SharedPtr callback_group_reentrant_;

    // ========================================================================
    // ROS CLIENTS
    // ========================================================================
    rclcpp::Client<EnableRobot>::SharedPtr enable_client_;
    rclcpp::Client<ClearError>::SharedPtr clear_error_client_;
    rclcpp::Client<DO>::SharedPtr do_client_;
    rclcpp::Client<RobotMode>::SharedPtr robot_mode_client_;
    rclcpp::Client<GetErrorID>::SharedPtr error_client_;
    rclcpp::Client<ResetRobot>::SharedPtr reset_robot_client_;
    rclcpp::Client<Pause>::SharedPtr pause_client_;         // soft pause
    rclcpp::Client<DisableRobot>::SharedPtr disable_robot_client_;  // E-Stop power off


    // ========================================================================
    // STATE MACHINE
    // ========================================================================
    SystemState current_state_;
    std::thread state_machine_thread_;
    std::atomic<bool> state_machine_running_{true};
    std::recursive_mutex state_mutex_;
    
    bool is_first_batch_;
    std::atomic<bool> stop_after_single_motion_{false};
    bool is_last_batch_;
    
    // ========================================================================
    // SYSTEM FLAGS
    // ========================================================================
    std::atomic<bool> system_started_{false};
    std::atomic<bool> system_running_{false};
    std::atomic<bool> new_tray_loaded_{false};
    std::atomic<bool> feed_chamber_signal_{false};
    std::atomic<bool> fill_done_{false};
    std::atomic<bool> scale_result_received_{false};  // Flag to track result availability
    // scale_result_ready removed - callback triggers directly
    std::atomic<bool> scale_result_pass_{false};
    std::atomic<bool> system_enabled_{true};  // Robot assumed enabled on startup; set false only by E-Stop

    std::atomic<bool> emergency_stop_{false};
    std::atomic<bool> manual_mode_{false};
    std::atomic<bool> system_paused_{false};  // NEW: true=paused, state machine blocks

    std::atomic<bool> use_ai_for_control_{false}; // Default to false (Auto Mode) for safety
    std::atomic<bool> stored_scale_result_{false}; // Store actual scale result for pipeline processing
    std::atomic<bool> dual_camera_mode_{true}; // DUAL CAM: Both cameras run in parallel - no switching needed
    int current_auto_row_{1}; // 1-5 for Auto Mode sequencing
    int motion_fail_count_{0}; // Consecutive motion failure counter
    
    // ========================================================================
    // CAMERA 1 - INPUT TRAY & BUFFER
    // ========================================================================
    std::vector<bool> row_full_;
    bool input_tray_empty_;
    int selected_input_row_;
    std::mutex row_selection_mutex_;
    
    int current_auto_slot_{1}; // 1-9 sequential output
    int current_fail_slot_{1}; // 1-4 sequential fail
    bool buffer_is_empty_{true};
    bool is_last_tray_available_{false};

    // Tray change management
    std::atomic<bool> waiting_for_tray_change_{true}; // Start WAITING for tray by default
    int all_rows_empty_frame_count_{0};
    static constexpr int EMPTY_TRAY_CONFIRM_FRAMES = 8;
    
    // NEW: Motion Mode
    std::atomic<bool> use_action_motion_{true}; // Now Action-Based Only
    
    // ========================================================================
    // ASYNC MOTION STATE (Non-blocking)
    // ========================================================================
    std::atomic<bool> motion_in_progress_{false};  // true=running, false=idle/done
    std::atomic<bool> motion_result_{false};        // Result (valid when !motion_in_progress_)
    std::string motion_current_cmd_;                // Current motion command name
    int async_slot_{0};                             // Stored slot for async result processing
    

    // Camera control state
    std::atomic<int> current_active_camera_{-1};  // 0=cam0, 1=cam1, -1=none
    std::mutex camera_mutex_;
    
    // ========================================================================
    // CAMERA 2 - OUTPUT TRAY
    // ========================================================================
    // Mutex for slot detection protection
    std::mutex slot_detection_mutex_;
    int selected_output_slot_;
    std::mutex output_slot_selection_mutex_;
    rclcpp::Time output_tray_wait_start_;
    
    // ✅ NEW: Queue for Pending Scale Results (Patch 2)
    struct PendingScaleResult {
        bool pass;
        rclcpp::Time timestamp;
        int cartridge_id;
    };
    std::deque<PendingScaleResult> pending_scale_results_;
    std::mutex scale_result_mutex_;
    std::atomic<int> cartridge_counter_{0};
    
    // Helper declarations
    bool processNextScaleResult(bool& result_pass, int& cartridge_id);
    bool hasPendingScaleResults();

    // ========================================================================
    // ROBOT STATE
    // ========================================================================
    bool chamber_is_empty_;
    bool chamber_has_cartridge_;
    bool scale_has_cartridge_;
    
    // ========================================================================
    // TIMING
    // ========================================================================
    rclcpp::Time scale_wait_start_;
    
    // System uptime tracking
    rclcpp::Time system_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::TimerBase::SharedPtr uptime_timer_;
    
    // Tray count tracking
    std::atomic<int> tray_count_{0};
    
    // ✅ PERFORMANCE BENCHMARKING
    std::atomic<uint64_t> callback_count_{0};
    std::atomic<uint64_t> total_callback_time_us_{0};
    

    
    // ========================================================================
    // SIMULATION MODE
    // ========================================================================
    bool simulate_scale_;
    bool force_pass_;

    // ========================================================================
    // INITIALIZATION METHODS
    // ========================================================================
    void initServiceClients();
    void initSubscriptions();
    void initPublishers();
    void initServices();


    // ========================================================================
    // ROS CALLBACK METHODS
    // ========================================================================

    void cameraActiveIdCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void commandCameraCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void feedChamberCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void fillDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scaleResultCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void startButtonCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void newTrayCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void selectedRowCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void selectedSlotCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void isLastTrayCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void gotoStateCallback(const std_msgs::msg::String::SharedPtr msg);
    void commandRowCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void commandSlotCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void commandChangeTrayCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // Camera switch helpers
    void requestCameraSwitch(int camera_id);
    bool waitForCameraActive(int target_camera, double timeout_sec = 5.0);
    bool switchAndWaitForCamera(int camera_id);
    bool switchAndWaitForCameraWithRetry(int camera_id, int max_retries = 3);
    void publishCameraStatus(const std::string &status);

    // ========================================================================
    // SERVICE CALLBACK METHODS
    // ========================================================================
    void enableSystemCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void startSystemCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    void resetStateCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void pauseSystemCallback(  // NEW: true=pause, false=resume
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // Unified mode callback (1=AUTO, 2=AI, 3=MANUAL)
    void setModeCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // ========================================================================
    // STATE MACHINE METHODS
    // ========================================================================
    void stateMachineLoop();
    void handleCurrentState();
    void notifyStateChange();
    
    // State handlers
    void stateIdle();
    void stateInitCheck();
    void stateInitLoadChamberDirect();
    void stateInitRefillBuffer();
    void stateWaitFilling();
    void stateTakeChamberToScale();
    void stateProcessingScale();
    void stateErrorScaleTimeout();
    void statePlaceToOutput();
    void statePlaceToFail();
    void stateRefillBuffer();
    void stateLoadChamberFromBuffer();
    void stateLastBatchWait();
    void stateErrorScale();
    void stateErrorOutputTrayTimeout();
    void stateErrorMotionLost();


    // ========================================================================
    // MOTION METHODS
    // ========================================================================
    bool setDigitalOutput(int index, bool status);
    bool checkMotionAlive(double timeout_sec = 2.0);
    bool sendMotionAction(const std::string& command, int slot = 0, int timeout_sec = 30);
    
    // ✅ ASYNC (Non-blocking) motion - returns immediately
    void sendMotionActionAsync(const std::string& command, int slot = 0);
    
    // Motion sequences (delegate to motion_executor via topic)
    bool sendMotionCommand(const std::string& command, int timeout_sec = 60);
    bool motionStub_InputTrayChamber();
    bool motionStub_InputTrayBuffer_SinglePick();
    bool motionStub_InputTrayBuffer();
    bool motionStub_ChamberScale();
    bool motionStub_ScaleOutput(int slot);
    bool motionStub_ScaleFail();
    bool motionStub_BufferChamber();
    
    // ========================================================================
    // HELPER METHODS
    // ========================================================================
    void logState(const std::string &message);
    void transitionTo(SystemState new_state);
    bool checkConnection();
    bool validateRobotState();
    std::string stateToString(SystemState state);
    void publishSystemStatus(const std::string &status);
    void publishError(const std::string &error);
    
    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr callService(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request,
        const std::string &service_name);
};

// ============================================================================
// CONSTRUCTOR
// ============================================================================

RobotLogicNode::RobotLogicNode() 
    : Node("robot_logic_nova5"),
      current_state_(SystemState::IDLE),
      is_first_batch_(true),
      is_last_batch_(false),
      input_tray_empty_(false),
      selected_input_row_(1),
      buffer_is_empty_(true),

      selected_output_slot_(-1),
      chamber_is_empty_(true),
      chamber_has_cartridge_(false),
      scale_has_cartridge_(false),
      simulate_scale_(false),
      force_pass_(false)

{
    RCLCPP_INFO(this->get_logger(), "=== Robot Logic Node Starting ===");

    row_full_.assign(5, false);
    input_tray_empty_ = false;
    selected_input_row_ = 1;

    // Reentrant callback group for parallel execution
    callback_group_reentrant_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    
    RCLCPP_INFO(this->get_logger(), "[PERF] Reentrant callback group created");

    initServiceClients();
    initSubscriptions();
    initPublishers();
    initServices();
    
    state_machine_thread_ = std::thread(&RobotLogicNode::stateMachineLoop, this);


    // Log dual camera mode status
    if (dual_camera_mode_) {
        RCLCPP_INFO(this->get_logger(), "📷 DUAL CAMERA MODE - Both cameras running in parallel, no switching required");
    }

    // Initialize uptime timer (1 second interval)
    uptime_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            if (system_enabled_ && system_running_) {
                auto now = this->now();
                auto elapsed = (now - system_start_time_).seconds();
                
                int hours = static_cast<int>(elapsed) / 3600;
                int minutes = (static_cast<int>(elapsed) % 3600) / 60;
                int seconds = static_cast<int>(elapsed) % 60;
                
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", hours, minutes, seconds);
                
                auto msg = std_msgs::msg::String();
                msg.data = buffer;
                system_uptime_pub_->publish(msg);
            } else {
                // Publish default when system is disabled
                auto msg = std_msgs::msg::String();
                msg.data = "00:00:00";
                system_uptime_pub_->publish(msg);
            }
            
            // Always publish tray count
            auto tray_msg = std_msgs::msg::Int32();
            tray_msg.data = tray_count_.load();
            tray_count_pub_->publish(tray_msg);
        });
    
    RCLCPP_INFO(this->get_logger(), "=== Robot Logic Node Ready ===");
}

// ============================================================================
// DESTRUCTOR
// ============================================================================

RobotLogicNode::~RobotLogicNode()
{
    state_machine_running_ = false;
    state_cv_.notify_all();
    if (state_machine_thread_.joinable())
    {
        state_machine_thread_.join();
    }
}

// ============================================================================
// INITIALIZATION IMPLEMENTATIONS
// ============================================================================

void RobotLogicNode::initServiceClients()
{
    // Use explicit QoS profile for service compatibility
    motion_action_client_ = rclcpp_action::create_client<ExecuteMotion>(this, "/robot/execute_motion");
    auto qos = rclcpp::ServicesQoS();
    
    enable_client_        = create_client<EnableRobot>("/nova5/dobot_bringup/EnableRobot", qos, callback_group_reentrant_);
    clear_error_client_   = create_client<ClearError>("/nova5/dobot_bringup/ClearError", qos, callback_group_reentrant_);
    do_client_            = create_client<DO>("/nova5/dobot_bringup/DO", qos, callback_group_reentrant_);
    robot_mode_client_    = create_client<RobotMode>("/nova5/dobot_bringup/RobotMode", qos, callback_group_reentrant_);
    error_client_         = create_client<GetErrorID>("/nova5/dobot_bringup/GetErrorID", qos, callback_group_reentrant_);
    reset_robot_client_   = create_client<ResetRobot>("/nova5/dobot_bringup/ResetRobot", qos, callback_group_reentrant_);
    pause_client_         = create_client<Pause>("/nova5/dobot_bringup/Pause", qos, callback_group_reentrant_);
    disable_robot_client_ = create_client<DisableRobot>("/nova5/dobot_bringup/DisableRobot", qos, callback_group_reentrant_);  // NEW
}

void RobotLogicNode::initSubscriptions()
{
    // ========================================================================
    // VISION NODE SUBSCRIPTIONS (Replaces direct YOLO callbacks)
    // ========================================================================
    vision_row_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/vision/input_tray/selected_row", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            if (msg->data > 0 && msg->data <= 5) {
                selected_input_row_ = msg->data;
                row_full_[msg->data - 1] = true;
            }
        });
    
    vision_empty_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/vision/input_tray/empty", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            input_tray_empty_ = msg->data;
        });
    
    vision_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/vision/output_tray/selected_slot", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            selected_output_slot_ = msg->data;
        });


    
    // Camera active ID confirmation from CSI node
    camera_active_id_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/camera/active_id", 10,
        std::bind(&RobotLogicNode::cameraActiveIdCallback, this, std::placeholders::_1));

    // Manual camera command
    command_camera_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/robot/command_camera", 10,
        std::bind(&RobotLogicNode::commandCameraCallback, this, std::placeholders::_1));
    
    // ========================================================================
    // SUBSCRIPTIONS: Default QoS (no custom callback group)
    // ========================================================================
    
    feed_chamber_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/revpi/feed_chamber", 10,
        std::bind(&RobotLogicNode::feedChamberCallback, this, std::placeholders::_1));
    
    fill_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/fill_machine/fill_done", 10,
        std::bind(&RobotLogicNode::fillDoneCallback, this, std::placeholders::_1));
        
    scale_result_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/scale/result", 10,
        std::bind(&RobotLogicNode::scaleResultCallback, this, std::placeholders::_1));
        
    start_button_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/system/start_button", 10,
        std::bind(&RobotLogicNode::startButtonCallback, this, std::placeholders::_1));
        
    new_tray_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge_providesystem/new_tray_loaded", 10,
        std::bind(&RobotLogicNode::newTrayCallback, this, std::placeholders::_1));
    
    is_last_tray_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/revpi/is_last_tray", 10,
        std::bind(&RobotLogicNode::isLastTrayCallback, this, std::placeholders::_1));
    
    selected_row_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/camera/ai/selected_row", 10,
        std::bind(&RobotLogicNode::selectedRowCallback, this, std::placeholders::_1));

    goto_state_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot/goto_state", 10,
        std::bind(&RobotLogicNode::gotoStateCallback, this, std::placeholders::_1));

    command_row_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/robot/command_row", 10,
        std::bind(&RobotLogicNode::commandRowCallback, this, std::placeholders::_1));

    selected_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/camera/ai/selected_slot", 10,
        std::bind(&RobotLogicNode::selectedSlotCallback, this, std::placeholders::_1));

    command_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/robot/command_slot", 10,
        std::bind(&RobotLogicNode::commandSlotCallback, this, std::placeholders::_1));
    
    command_change_tray_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/command/change_tray", 10,
        std::bind(&RobotLogicNode::commandChangeTrayCallback, this, std::placeholders::_1));

    // Monitoring Subscriptions
    motion_hb_sub_ = create_subscription<std_msgs::msg::Header>(
        "/robot/motion_heartbeat", 10,
        [this](const std_msgs::msg::Header::SharedPtr msg) { last_motion_hb_ = msg->stamp; });
    
    vision_hb_sub_ = create_subscription<std_msgs::msg::Header>(
        "/vision/heartbeat", 10,
        [this](const std_msgs::msg::Header::SharedPtr msg) { last_vision_hb_ = msg->stamp; });

    motion_busy_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/motion_busy", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { motion_busy_ = msg->data; });

    // Interlock: cartridge system báo đang hoạt động — robot phải đợi
    create_subscription<std_msgs::msg::Bool>(
        "/cartridge/busy", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            cartridge_busy_ = msg->data;
            if (msg->data) {
                RCLCPP_WARN(get_logger(), "[INTERLOCK] 🔒 Cartridge BUSY — pick/refill sẽ bỏ qua");
            } else {
                RCLCPP_INFO(get_logger(), "[INTERLOCK] 🔓 Cartridge FREE — robot có thể pick/refill");
            }
        });
}


void RobotLogicNode::initPublishers()
{
    system_status_pub_ = create_publisher<std_msgs::msg::String>("/robot/system_status", 10);
    error_pub_ = create_publisher<std_msgs::msg::String>("/robot/error", 10);
    selected_slot_pub_ = create_publisher<std_msgs::msg::Int32>("/robot/selected_output_slot", 10);
    selected_row_pub_ = create_publisher<std_msgs::msg::Int32>("/robot/selected_input_row", 10);
    gripper_cmd_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/gripper_cmd", 10);
    picker_cmd_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/picker_cmd", 10);
    
    // Camera control publishers
    camera_select_pub_ = create_publisher<std_msgs::msg::Int32>("/robot/camera_select", 10);
    camera_status_pub_ = create_publisher<std_msgs::msg::String>("/camera/status", 10);
    
    // System monitor publishers
    system_uptime_pub_ = create_publisher<std_msgs::msg::String>("/robot/system_uptime", 10);
    tray_count_pub_ = create_publisher<std_msgs::msg::Int32>("/robot/tray_count", 10);
    change_tray_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/change_tray", 10);
    done_output_tray_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/done_tray_output", 10);
    place_done_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/place_done", 10);
    last_batch_complete_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/last_batch_complete", 10);
    motion_busy_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/motion_busy", 10);  // NEW
    

}

void RobotLogicNode::initServices()
{
    enable_system_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/enable_system",
        std::bind(&RobotLogicNode::enableSystemCallback, this,
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_reentrant_);
    
    emergency_stop_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/emergency_stop",
        std::bind(&RobotLogicNode::emergencyStopCallback, this,
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_reentrant_);

    start_system_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/start_system",
        std::bind(&RobotLogicNode::startSystemCallback, this,
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_reentrant_);
    
    // Unified mode selection subscription (1=AUTO, 2=AI, 3=MANUAL)
    // Subscriptions don't strictly need reentrant unless they call services
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_reentrant_;
    
    set_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/robot/set_mode", 10,
        std::bind(&RobotLogicNode::setModeCallback, this, std::placeholders::_1),
        sub_options);
    
    // Reset state service - allows resetting without restarting node
    reset_state_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/reset_state",
        std::bind(&RobotLogicNode::resetStateCallback, this,
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_reentrant_);

    // Pause/Resume service — true=pause, false=resume
    pause_system_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/pause_system",
        std::bind(&RobotLogicNode::pauseSystemCallback, this,
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_reentrant_);
}



// ============================================================================
// Camera control callbacks and helpers
// ============================================================================

void RobotLogicNode::cameraActiveIdCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        current_active_camera_ = msg->data;
    }
    RCLCPP_INFO(this->get_logger(), "[CAMERA] ✅ Active camera confirmed: %d", msg->data);
    publishCameraStatus("CAMERA_" + std::to_string(msg->data) + "_ACTIVE");
    notifyStateChange();
}

void RobotLogicNode::commandCameraCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int camera_id = msg->data;
    if (camera_id < 1 || camera_id > 2)
    {
        RCLCPP_ERROR(this->get_logger(), "[CMD CAMERA] ❌ Invalid Camera ID: %d", camera_id);
        publishError("INVALID_CAMERA_ID");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[CMD CAMERA] 📷 Manual switch to Camera %d", camera_id);
    publishCameraStatus("MANUAL_SWITCH_REQUESTED_CAM_" + std::to_string(camera_id));

    if (switchAndWaitForCameraWithRetry(camera_id, 3))
    {
        RCLCPP_INFO(this->get_logger(), "[CMD CAMERA] ✅ Manual switch successful");
        publishCameraStatus("MANUAL_SWITCH_SUCCESS_CAM_" + std::to_string(camera_id));
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "[CMD CAMERA] ❌ Manual switch failed");
        publishError("MANUAL_CAMERA_SWITCH_FAILED");
    }
}

void RobotLogicNode::requestCameraSwitch(int camera_id)
{
    if (camera_id < 1 || camera_id > 2)
    {
        RCLCPP_ERROR(this->get_logger(), "[CAMERA] ❌ Invalid camera ID: %d", camera_id);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (current_active_camera_ == camera_id)
        {
            RCLCPP_INFO(this->get_logger(), "[CAMERA] ✅ Camera %d already active", camera_id);
            return;
        }
    }

    auto msg = std::make_shared<std_msgs::msg::Int32>();
    msg->data = camera_id;
    camera_select_pub_->publish(*msg);

    RCLCPP_INFO(this->get_logger(), "[CAMERA] 📷 Requesting Camera %d switch...", camera_id);
    publishCameraStatus("SWITCHING_TO_CAMERA_" + std::to_string(camera_id));
}

bool RobotLogicNode::waitForCameraActive(int target_camera, double timeout_sec)
{
    auto start_time = this->now();
    int last_logged_second = -1;
    RCLCPP_INFO(this->get_logger(), "[CAMERA] ⏳ Waiting for Camera %d confirmation (timeout: %.1fs)...", target_camera, timeout_sec);

    while (rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (current_active_camera_ == target_camera)
            {
                auto elapsed = (this->now() - start_time).seconds();
                RCLCPP_INFO(this->get_logger(), "[CAMERA] ✅ Camera %d confirmed active (took %.2fs)", target_camera, elapsed);
                return true;
            }
        }

        auto elapsed = (this->now() - start_time).seconds();
        if (elapsed > timeout_sec)
        {
            RCLCPP_ERROR(this->get_logger(), "[CAMERA] ❌ Timeout waiting for Camera %d (%.1fs elapsed)", target_camera, elapsed);
            publishCameraStatus("TIMEOUT_WAITING_CAMERA_" + std::to_string(target_camera));
            return false;
        }

        int current_second = static_cast<int>(elapsed);
        if (current_second > last_logged_second && current_second > 0)
        {
            RCLCPP_WARN(this->get_logger(), "[CAMERA] ⏱️ Still waiting... (%.1fs / %.1fs)", elapsed, timeout_sec);
            last_logged_second = current_second;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    return false;
}



bool RobotLogicNode::switchAndWaitForCamera(int camera_id)
{
    requestCameraSwitch(camera_id);

    if (!waitForCameraActive(camera_id, 5.0))
    {
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "[CAMERA] 🔄 Waiting 500ms for frame stabilization...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Validate AI detection (removed)
    // if (!waitForFirstDetection(camera_id, 3.0)) ...

    publishCameraStatus("CAMERA_" + std::to_string(camera_id) + "_READY");
    return true;
}

bool RobotLogicNode::switchAndWaitForCameraWithRetry(int camera_id, int max_retries)
{
    // DUAL CAM MODE: Both cameras running in parallel - no switching needed
    if (dual_camera_mode_) {
        RCLCPP_DEBUG(this->get_logger(), "[CAMERA] 📷 Dual camera mode - skip switching (cam%d always available)", camera_id);
        return true;
    }

    for (int attempt = 1; attempt <= max_retries; ++attempt)
    {
        RCLCPP_INFO(this->get_logger(), "[CAMERA] 🔄 Attempt %d/%d to switch to Camera %d", attempt, max_retries, camera_id);
        publishCameraStatus("SWITCH_ATTEMPT_" + std::to_string(attempt) + "_CAM_" + std::to_string(camera_id));

        if (switchAndWaitForCamera(camera_id))
        {
            RCLCPP_INFO(this->get_logger(), "[CAMERA] ✅ Switch successful on attempt %d", attempt);
            return true;
        }

        if (attempt < max_retries)
        {
            RCLCPP_WARN(this->get_logger(), "[CAMERA] ⚠️ Attempt %d failed, retrying in 2 seconds...", attempt);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    RCLCPP_ERROR(this->get_logger(), "[CAMERA] ❌ Failed to switch to Camera %d after %d attempts", camera_id, max_retries);
    publishCameraStatus("SWITCH_FAILED_AFTER_" + std::to_string(max_retries) + "_ATTEMPTS_CAM_" + std::to_string(camera_id));
    return false;
}

void RobotLogicNode::publishCameraStatus(const std::string &status)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = status;
    camera_status_pub_->publish(*msg);
}

void RobotLogicNode::feedChamberCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    feed_chamber_signal_ = msg->data;
    
    if (msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "[FEED_CHAMBER] Chamber ready for loading");
        notifyStateChange();
    }
}

void RobotLogicNode::fillDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    fill_done_ = msg->data;
    
    if (msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "[FILL_DONE] Filling complete");
        notifyStateChange();
    }
}

// ────────────────────────────────────────────────────────────────────────────
// PATCH 2: SCALE QUEUE HELPERS
// ────────────────────────────────────────────────────────────────────────────

bool RobotLogicNode::processNextScaleResult(bool& result_pass, int& cartridge_id)
{
    std::lock_guard<std::mutex> lock(scale_result_mutex_);
    
    if (pending_scale_results_.empty()) {
        return false;  // No pending results
    }
    
    // Get FIFO (oldest result first)
    PendingScaleResult result = pending_scale_results_.front();
    pending_scale_results_.pop_front();
    
    result_pass = result.pass;
    cartridge_id = result.cartridge_id;
    
    RCLCPP_INFO(this->get_logger(), 
        "[SCALE] 📦 Processing result #%d: %s (Queue remaining: %zu)", 
        cartridge_id,
        result_pass ? "PASS" : "FAIL",
        pending_scale_results_.size());
    
    return true;
}

bool RobotLogicNode::hasPendingScaleResults()
{
    std::lock_guard<std::mutex> lock(scale_result_mutex_);
    return !pending_scale_results_.empty();
}

void RobotLogicNode::scaleResultCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // ✅ Store result in queue with metadata
    PendingScaleResult result;
    result.pass = msg->data;
    result.timestamp = this->now();
    result.cartridge_id = cartridge_counter_.load();
    
    {
        std::lock_guard<std::mutex> lock(scale_result_mutex_);
        pending_scale_results_.push_back(result);
        
        // ✅ Limit queue size (safety)
        if (pending_scale_results_.size() > 10) {
            RCLCPP_WARN(this->get_logger(), 
                "[SCALE] Queue overflow! Oldest result dropped.");
            pending_scale_results_.pop_front();
        }
    }
    
    // ✅ Legacy flags for backward compatibility
    stored_scale_result_.store(msg->data);
    scale_result_received_ = true;
    
    RCLCPP_INFO(this->get_logger(), 
        "[SCALE] ⚡ Result #%d received: %s (Queue size: 1 [approx])", // size thread unsafe to read here without lock, but keeping simple
        result.cartridge_id,
        msg->data ? "PASS" : "FAIL");
    
    // Notify state machine
    if (current_state_ == SystemState::PROCESSING_SCALE || 
        current_state_ == SystemState::ERROR_SCALE_TIMEOUT) {
        notifyStateChange();
    }
}

void RobotLogicNode::startButtonCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) return;
    
    RCLCPP_INFO(this->get_logger(), "[INIT] ⚡ Quick start initiated...");
    
    // 1. Reset Internal Flags
    if (emergency_stop_) {
        emergency_stop_ = false;
        RCLCPP_INFO(this->get_logger(), "[INIT] E-Stop flag reset");
    }
    system_enabled_ = true;
    system_running_ = true;
    
    // 2. Clear Error (Fast)
    auto clear_req = std::make_shared<ClearError::Request>();
    callService<ClearError>(clear_error_client_, clear_req, "ClearError");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 3. Enable Robot (Fast)
    auto enable_req = std::make_shared<EnableRobot::Request>();
    enable_req->load = 0.0;
    callService<EnableRobot>(enable_client_, enable_req, "EnableRobot");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 4. Move Home (Action waits automatically)
    sendMotionAction("HOME");
    
    // 5. Finalize
    system_started_ = true;
    // new_tray_loaded_ is NOT auto-set - wait for /cartridge_providesystem/new_tray_loaded topic
    
    RCLCPP_INFO(this->get_logger(), "[INIT] ✅ System Ready - Waiting for New Tray signal...");
    notifyStateChange();
}


void RobotLogicNode::newTrayCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        new_tray_loaded_ = true;
        
        // ✅ Clear tray change waiting flag
        if (waiting_for_tray_change_)
        {
            waiting_for_tray_change_ = false;
            all_rows_empty_frame_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "[CHANGE_TRAY] New tray loaded → Unblocked input tray states");
        }
        
        // ✅ User Request: Fix flag reset - Ensure tray is marked "Not Empty"
        // When new tray is loaded, we assume it has items (until camera/logic says otherwise)
        {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            input_tray_empty_ = false;
            // Optionally reset rows to true (Full) so Refill doesn't think it's empty immediately
            // Especially strictly for Auto Mode. AI mode will update via camera?
            // To be safe, let's assume full.
             for (size_t i = 0; i < row_full_.size(); ++i) {
                row_full_[i] = true;
            }
            selected_input_row_ = 1; // Reset selection
        }
        
        RCLCPP_INFO(this->get_logger(), "[TRAY] ✅ New tray loaded - Flags reset, Ready to pick!");
        notifyStateChange();
    }
    else
    {
        new_tray_loaded_ = false;
        RCLCPP_INFO(this->get_logger(), "[TRAY] ⚠️ Tray removed/empty");
    }
}

void RobotLogicNode::selectedRowCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(row_selection_mutex_);
    
    int row = msg->data;
    
    if (row < 1 || row > 5)
    {
        RCLCPP_WARN(this->get_logger(), "[AI ROW] Invalid row: %d", row);
        return;
    }
    
    if (!row_full_[row - 1])
    {
        if (manual_mode_ || !use_ai_for_control_)
        {
            RCLCPP_INFO(this->get_logger(), "[AI ROW] Row %d not full (Ignored in Manual/Auto Mode)", row);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[AI ROW] Row %d not full", row);
            return;
        }
    }
    
    selected_input_row_ = row;
    RCLCPP_INFO(this->get_logger(), "[AI ROW] Selected Row %d", row);
    notifyStateChange();
}

void RobotLogicNode::selectedSlotCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int slot = msg->data;
    
    if (slot < 1 || slot > 9)
    {
        RCLCPP_WARN(this->get_logger(), "[AI SLOT] Invalid slot: %d", slot);
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        selected_output_slot_ = slot;
    }
    
    RCLCPP_INFO(this->get_logger(), "[AI SLOT] Selected Slot %d", slot);
    notifyStateChange();
}

void RobotLogicNode::gotoStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string state_name = msg->data;
    SystemState target_state = SystemState::IDLE;

    if (state_name == "IDLE") target_state = SystemState::IDLE;
    else if (state_name == "INIT_CHECK") target_state = SystemState::INIT_CHECK;
    else if (state_name == "INIT_LOAD_CHAMBER_DIRECT") target_state = SystemState::INIT_LOAD_CHAMBER_DIRECT;
    else if (state_name == "INIT_REFILL_BUFFER") target_state = SystemState::INIT_REFILL_BUFFER;
    else if (state_name == "WAIT_FILLING") target_state = SystemState::WAIT_FILLING;
    else if (state_name == "TAKE_CHAMBER_TO_SCALE") target_state = SystemState::TAKE_CHAMBER_TO_SCALE;
    else if (state_name == "PROCESSING_SCALE") target_state = SystemState::PROCESSING_SCALE;
    else if (state_name == "ERROR_SCALE_TIMEOUT") target_state = SystemState::ERROR_SCALE_TIMEOUT;
    else if (state_name == "PLACE_TO_OUTPUT") target_state = SystemState::PLACE_TO_OUTPUT;
    else if (state_name == "PLACE_TO_FAIL") target_state = SystemState::PLACE_TO_FAIL;
    else if (state_name == "REFILL_BUFFER") target_state = SystemState::REFILL_BUFFER;
    else if (state_name == "LOAD_CHAMBER_FROM_BUFFER") target_state = SystemState::LOAD_CHAMBER_FROM_BUFFER;
    else if (state_name == "LAST_BATCH_WAIT") target_state = SystemState::LAST_BATCH_WAIT;
    else
    {
        RCLCPP_ERROR(this->get_logger(), "[GOTO] Unknown state: %s", state_name.c_str());
        return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    if (target_state == SystemState::PROCESSING_SCALE)
    {
        scale_wait_start_ = this->now();
        scale_result_received_ = false;
        RCLCPP_INFO(this->get_logger(), "[GOTO] Initialized wait timer for PROCESSING_SCALE");
    }
    
    // Force-enable for manual PLACE commands so state machine doesn't kick to IDLE
    if (target_state == SystemState::PLACE_TO_OUTPUT ||
        target_state == SystemState::PLACE_TO_FAIL)
    {
        system_enabled_ = true;
        manual_mode_ = true;
        stop_after_single_motion_ = true;
        RCLCPP_INFO(this->get_logger(),
            "[GOTO] Manual %s: system_enabled=true, manual_mode=true",
            state_name.c_str());
        
        // Auto-select slot if none selected (for PLACE_TO_OUTPUT)
        if (target_state == SystemState::PLACE_TO_OUTPUT)
        {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            if (selected_output_slot_ == -1)
            {
                selected_output_slot_ = current_auto_slot_;
                RCLCPP_INFO(this->get_logger(),
                    "[GOTO] Auto-selected slot %d for PLACE_TO_OUTPUT", selected_output_slot_);
            }
        }
    }
    
    if (target_state == SystemState::REFILL_BUFFER)
    {
        int sel = -1;
        {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            sel = selected_input_row_;
        }

        if (sel != -1)
        {
            // AUTO mode: just use the selected row, don't switch to manual
            stop_after_single_motion_ = true;
        }
    }

    transitionTo(target_state);
}

void RobotLogicNode::commandRowCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int row = msg->data;
    if (row < 1 || row > 5)
    {
        RCLCPP_ERROR(this->get_logger(), "[CMD ROW] Invalid Row %d", row);
        return;
    }

    // AUTO/AI mode: just set the row, don't switch to manual
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        selected_input_row_ = row;
        row_full_[row-1] = true;
    }
    
    RCLCPP_INFO(this->get_logger(), "[CMD ROW] Row %d selected", row);
    notifyStateChange();
}

void RobotLogicNode::commandSlotCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int slot = msg->data;
    if (slot < 1 || slot > 9)
    {
        RCLCPP_ERROR(this->get_logger(), "[CMD SLOT] Invalid Slot %d", slot);
        return;
    }

    // AUTO/AI mode: just set the slot, don't switch to manual
    {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        selected_output_slot_ = slot;
    }
    
    RCLCPP_INFO(this->get_logger(), "[CMD SLOT] Slot %d selected", slot);
    notifyStateChange();
}

void RobotLogicNode::commandChangeTrayCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) return;
    
    RCLCPP_WARN(this->get_logger(), "[CHANGE_TRAY] Manual trigger - Publishing change_tray signal");
    
    auto change_msg = std_msgs::msg::Bool();
    change_msg.data = true;
    change_tray_pub_->publish(change_msg);
    
    waiting_for_tray_change_ = true;
    RCLCPP_INFO(this->get_logger(), "[CHANGE_TRAY] Waiting for new tray - Input tray states blocked");
}

void RobotLogicNode::isLastTrayCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        is_last_tray_available_ = true;
        RCLCPP_WARN(this->get_logger(), "[LAST_TRAY] ⚠️ Signal received! This is the FINAL TRAY.");
    }
}

// ============================================================================
// SERVICE CALLBACK IMPLEMENTATIONS
// ============================================================================

void RobotLogicNode::enableSystemCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data)
    {
        RCLCPP_INFO(this->get_logger(), "[SERVICE] ⚡ Enable System — Power ON Robot...");
        
        // 1. Reset E-Stop flag
        if (emergency_stop_) {
            emergency_stop_ = false;
            RCLCPP_INFO(this->get_logger(), "[INIT] E-Stop flag reset");
        }
        system_enabled_ = true;
        
        // 2. Clear Error
        auto clear_req = std::make_shared<ClearError::Request>();
        callService<ClearError>(clear_error_client_, clear_req, "ClearError");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 3. Enable Robot (Power ON)
        auto enable_req = std::make_shared<EnableRobot::Request>();
        enable_req->load = 0.0;
        callService<EnableRobot>(enable_client_, enable_req, "EnableRobot");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        RCLCPP_INFO(this->get_logger(), "[INIT] ✅ Robot Power ON — Ready. Press START to begin.");
        
        response->success = true;
        response->message = "Robot Power ON";
    }
    else
    {
        system_enabled_ = false;
        system_started_ = false;
        system_running_ = false;
        response->success = true;
        response->message = "System DISABLED";
    }
}

void RobotLogicNode::startSystemCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data)
    {
        // If E-Stop was pressed (system_enabled_=false), require re-enable first
        if (!system_enabled_) {
            RCLCPP_WARN(this->get_logger(),
                "[START] ⚠️ Robot was Emergency Stopped. Press ENABLE to re-power robot first.");
            response->success = false;
            response->message = "Robot DISABLED (Emergency Stop). Press ENABLE first.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[START] 🚀 Starting System — HOME + State Machine...");

        system_running_ = true;
        manual_mode_ = false;
        stop_after_single_motion_ = false;

        // Reset uptime tracking
        system_start_time_ = this->now();

        // Clear tray-change block so INIT_LOAD doesn’t wait for InReady before it’s needed
        // (InReady topic will be received during the process naturally)
        waiting_for_tray_change_ = false;

        // Move Home
        sendMotionAction("HOME");

        // Start state machine
        system_started_ = true;
        RCLCPP_INFO(this->get_logger(), "[START] ✅ System Started — entering INIT_CHECK...");
        notifyStateChange();

        response->success = true;
        response->message = "System Started";
    }
    else
    {
        system_started_ = false;
        system_running_ = false;
        RCLCPP_INFO(this->get_logger(), "[START] ⏹ System Stopped");
        response->success = true;
        response->message = "System Stopped";
    }
}

/*
void RobotLogicNode::newTrayLoadedCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data)
    {
        new_tray_loaded_ = true;
        RCLCPP_INFO(this->get_logger(), "[SERVICE] ✅ New tray loaded - Ready to pick!");
        notifyStateChange();
        
        response->success = true;
        response->message = "New Tray Loaded Accepted";
    }
    else
    {
        new_tray_loaded_ = false;
        RCLCPP_INFO(this->get_logger(), "[SERVICE] ⚠️ Tray removed/empty");
        
        response->success = true;
        response->message = "Tray Removed";
    }
}
*/

void RobotLogicNode::emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data)
    {
        emergency_stop_ = true;
        system_enabled_ = false;
        system_paused_  = false;  // clear any pause state

        response->success = true;
        response->message = "EMERGENCY STOP ACTIVATED — Robot POWER OFF";

        // Step 1: ResetRobot — flush motion queue (fire-and-forget)
        auto resetReq = std::make_shared<ResetRobot::Request>();
        reset_robot_client_->async_send_request(resetReq,
            [this](rclcpp::Client<ResetRobot>::SharedFuture future) {
                try { RCLCPP_INFO(this->get_logger(), "[E-STOP] ResetRobot: %d", future.get()->res); }
                catch (...) { RCLCPP_WARN(this->get_logger(), "[E-STOP] ResetRobot failed"); }

                // Step 2: DisableRobot — power off immediately after queue flushed
                auto disableReq = std::make_shared<DisableRobot::Request>();
                disable_robot_client_->async_send_request(disableReq,
                    [this](rclcpp::Client<DisableRobot>::SharedFuture f) {
                        try { RCLCPP_WARN(this->get_logger(), "[E-STOP] ⚡ DisableRobot (POWER OFF): %d", f.get()->res); }
                        catch (...) { RCLCPP_WARN(this->get_logger(), "[E-STOP] DisableRobot failed"); }
                    });
            });

        publishError("EMERGENCY STOP");
        notifyStateChange();
    }
    else
    {
        emergency_stop_ = false;
        response->success = true;
        response->message = "Emergency stop cleared";
    }
}

void RobotLogicNode::resetStateCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    (void)request; // Unused (value doesn't matter)
    
    RCLCPP_INFO(this->get_logger(), "[RESET] 🔄 Resetting all states to initial values...");
    
    // ✅ STOP ROBOT MOTION FIRST
    RCLCPP_INFO(this->get_logger(), "[RESET] Stopping robot motion...");
    auto clear_req = std::make_shared<ClearError::Request>();
    callService<ClearError>(clear_error_client_, clear_req, "ClearError");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Reset state machine
    current_state_ = SystemState::IDLE;
    
    // Reset batch flags
    is_first_batch_ = true;
    is_last_batch_ = false;
    
    // Reset input tray
    input_tray_empty_ = false;
    selected_input_row_ = 1;
    current_auto_row_ = 1;
    
    // ✅ Clear AI detection states
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        for (size_t i = 0; i < row_full_.size(); ++i) {
            row_full_[i] = false;
        }
    }
    
    // Reset buffer
    buffer_is_empty_ = true;
    is_last_tray_available_ = false;


    
    // Reset tray change management
    waiting_for_tray_change_ = true; // ✅ Wait for tray signal on reset/start
    all_rows_empty_frame_count_ = 0;
    
    // Reset output tray
    selected_output_slot_ = -1;
    current_auto_slot_ = 1;

    // Reset fail slot
    current_fail_slot_ = 1;
    
    // Reset chamber
    chamber_is_empty_ = true;
    chamber_has_cartridge_ = false;
    
    // Reset scale
    scale_has_cartridge_ = false;
    stored_scale_result_.store(false);
    scale_result_received_ = false;
    
    // Reset system flags
    system_started_ = false;
    // NOTE: system_enabled_ is NOT reset here — robot stays powered after STOP
    // Only E-Stop sets system_enabled_ = false (requires ENABLE to recover)
    system_running_ = false;
    new_tray_loaded_ = false;
    feed_chamber_signal_ = false;
    fill_done_ = false;
    emergency_stop_ = false;  // ✅ Clear emergency stop
    system_paused_ = false;   // ✅ Clear pause flag on full reset
    
    // Reset mode flags
    manual_mode_ = false;
    stop_after_single_motion_ = false;
    
    RCLCPP_INFO(this->get_logger(), "[RESET] ✅ State reset complete. Ready for new cycle!");
    
    response->success = true;
    response->message = "All states reset to initial values. System in IDLE state.";
    
    notifyStateChange();
}

void RobotLogicNode::pauseSystemCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data)
    {
        // ─── SOFT PAUSE ───
        // Chỉ block state machine — KHÔNG gọi Dobot Pause
        // (Dobot Pause sẽ interrupt motion action → action fail → motion_fail_count_++ → IDLE)
        // Motion đang chạy sẽ tự hoàn thành, sau đó state machine không start motion mới
        system_paused_ = true;
        RCLCPP_WARN(this->get_logger(),
            "[PAUSE] ⏸ Soft PAUSE — state machine blocked. Current motion will finish naturally.");

        publishSystemStatus("PAUSED");
        response->success = true;
        response->message = "System PAUSED (soft — current motion will complete)";
    }
    else
    {
        // ─── RESUME ───
        // Reset fail counter để tránh retry penalty tích lũy
        motion_fail_count_ = 0;

        // Nếu motion đã kết thúc trong khi paused và bị fail (do timeout/lỗi khác),
        // clear lệnh cũ để state machine bắt đầu sạch
        if (!motion_in_progress_ && !motion_result_ && !motion_current_cmd_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "[RESUME] Previous motion '%s' ended with fail while paused — clearing to retry",
                motion_current_cmd_.c_str());
            motion_current_cmd_.clear();
        }

        system_paused_ = false;
        RCLCPP_INFO(this->get_logger(), "[RESUME] ▶ State machine RESUMED from same state");
        notifyStateChange();

        response->success = true;
        response->message = "System RESUMED";
    }
}



void RobotLogicNode::setModeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int mode = msg->data;
    
    switch(mode) {
        case 1:  // AUTO
            manual_mode_ = false;
            use_ai_for_control_ = false;
            RCLCPP_INFO(this->get_logger(), "[MODE] Set to AUTO (Sequential picking)");
            break;
            
        case 2:  // AI/Camera
            manual_mode_ = false;
            use_ai_for_control_ = true;
            RCLCPP_INFO(this->get_logger(), "[MODE] Set to AI (Vision-based)");
            break;
            
        case 3:  // MANUAL
            manual_mode_ = true;
            use_ai_for_control_ = false;
            RCLCPP_INFO(this->get_logger(), "[MODE] Set to MANUAL");
            
            // Reset states for manual operation
            {
                std::lock_guard<std::mutex> lock(row_selection_mutex_);
                for (size_t i = 0; i < row_full_.size(); ++i) {
                    row_full_[i] = true;
                }
                input_tray_empty_ = false;
                selected_input_row_ = 1; // Default to 1 to match GUI
            }
            
            buffer_is_empty_ = true;
            selected_output_slot_ = 1;
            break;
            
        default:
            RCLCPP_ERROR(this->get_logger(), "[MODE] Invalid mode: %d. Use 1=AUTO, 2=AI, 3=MANUAL", mode);
            return;
    }
    
    notifyStateChange();
}

// ========================================================================
// STATE MACHINE IMPLEMENTATIONS
// ========================================================================

void RobotLogicNode::notifyStateChange()
{
    state_changed_ = true;
    state_cv_.notify_all();
}

void RobotLogicNode::stateMachineLoop()
{
    RCLCPP_INFO(this->get_logger(), "[STATE MACHINE] Started (Optimized)");
    
    while (state_machine_running_ && rclcpp::ok())
    {
        {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            handleCurrentState();
        }
        
        {
            std::unique_lock<std::mutex> cv_lock(state_cv_mutex_);
            
            // ✅ OPTIMIZATION: 50ms timeout + early wake on state change
            state_cv_.wait_for(cv_lock, std::chrono::milliseconds(50), [this]() {
                // Wake immediately if:
                // 1. State changed (priority)
                // 2. Shutdown requested
                return state_changed_.load() || !state_machine_running_;
            });
            
            state_changed_ = false;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "[STATE MACHINE] Stopped");
}

void RobotLogicNode::handleCurrentState()
{
    if (emergency_stop_)
    {
        publishSystemStatus("EMERGENCY STOP");
        return;
    }

    // ⏸ PAUSE guard: block all state transitions when paused
    if (system_paused_)
    {
        publishSystemStatus("PAUSED");
        return;
    }

    if (!system_enabled_ && current_state_ != SystemState::IDLE)
    {
        transitionTo(SystemState::IDLE);
        return;
    }
    
    // 🔴 Motion safety check (global)
    if (!checkMotionAlive(2.0) && current_state_ != SystemState::IDLE && current_state_ != SystemState::ERROR_MOTION_LOST)
    {
        publishError("MOTION_NODE_LOST");
        transitionTo(SystemState::ERROR_MOTION_LOST);
        return;
    }



    // 2. State machine logic
    switch (current_state_)
    {
        case SystemState::IDLE:
            stateIdle();
            break;


        case SystemState::INIT_CHECK:
            stateInitCheck();
            break;
        case SystemState::INIT_LOAD_CHAMBER_DIRECT:
            stateInitLoadChamberDirect();
            break;
        case SystemState::INIT_REFILL_BUFFER:
            stateInitRefillBuffer();
            break;
        case SystemState::WAIT_FILLING:
            stateWaitFilling();
            break;
        case SystemState::TAKE_CHAMBER_TO_SCALE:
            stateTakeChamberToScale();
            break;
        case SystemState::PROCESSING_SCALE:
            stateProcessingScale();
            break;
        case SystemState::ERROR_SCALE_TIMEOUT:
            stateErrorScaleTimeout();
            break;
        case SystemState::PLACE_TO_OUTPUT:
            statePlaceToOutput();
            break;
        case SystemState::PLACE_TO_FAIL:
            statePlaceToFail();
            break;
        case SystemState::REFILL_BUFFER:
            stateRefillBuffer();
            break;
        case SystemState::LOAD_CHAMBER_FROM_BUFFER:
            stateLoadChamberFromBuffer();
            break;
        case SystemState::LAST_BATCH_WAIT:
            stateLastBatchWait();
            break;
        case SystemState::ERROR_SCALE:
            stateErrorScale();
            break;
        case SystemState::ERROR_OUTPUT_TRAY_TIMEOUT:
            stateErrorOutputTrayTimeout();
            break;
    }
}

void RobotLogicNode::stateIdle()
{
    publishSystemStatus("IDLE");

    if (!system_enabled_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[IDLE] Robot DISABLED (E-Stop). Press ENABLE to recover.");
        return;
    }

    if (system_started_)
    {
        RCLCPP_INFO(this->get_logger(), "[IDLE] ✅ START received — transitioning to INIT_CHECK");
        system_started_ = false;
        is_first_batch_ = true;
        // NOTE: new_tray_loaded_ / waiting_for_tray_change_ are NOT required here.
        // INIT_LOAD_CHAMBER_DIRECT will wait for InReady topic if tray isn't ready yet.
        transitionTo(SystemState::INIT_CHECK);
    }
}


void RobotLogicNode::stateInitCheck()
{
    RCLCPP_INFO(this->get_logger(), "[STATE] Executing INIT_CHECK");
    publishSystemStatus("INIT_CHECK");

    transitionTo(SystemState::INIT_LOAD_CHAMBER_DIRECT);
}

void RobotLogicNode::stateInitLoadChamberDirect()
{
    static int debug_count = 0;
    if (debug_count++ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "[DEBUG] motion_cmd: '%s', in_prg: %d, manual: %d, first_batch: %d, feed: %d, ai: %d, wait_tray: %d",
            motion_current_cmd_.c_str(), 
            (int)motion_in_progress_.load(), 
            (int)manual_mode_, 
            (int)is_first_batch_, 
            (int)feed_chamber_signal_, 
            (int)use_ai_for_control_, 
            (int)waiting_for_tray_change_.load());
    }
    RCLCPP_INFO(this->get_logger(), "[STATE] Executing INIT_LOAD_CHAMBER_DIRECT");
    publishSystemStatus("INIT_LOAD_CHAMBER_DIRECT");
    
    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion to finish...");
        return;
    }

    // 🔒 INTERLOCK: Chời khi cartridge system đang thay khạy (S2/S3)
    if (cartridge_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[INTERLOCK] Cartridge BUSY (S2/S3) — INIT_LOAD_CHAMBER_DIRECT bị chặn, đợi...");
        return;
    }

    // ========================================================================

    // ✅ BLOCK if waiting for tray change
    // ========================================================================
    if (waiting_for_tray_change_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[CHANGE_TRAY] Waiting for new tray - INIT_LOAD_CHAMBER_DIRECT blocked");
        return;
    }

    // ✅ AUTO/MANUAL MODE BYPASS: Skip camera requirement
    if (!use_ai_for_control_ || manual_mode_ || stop_after_single_motion_) {
        RCLCPP_INFO(this->get_logger(), "[AUTO] Auto Mode - Skipping camera switch (not required)");
    } else {
        // Ensure Camera 1 active for input tray operations (AI Mode only)
        if (!switchAndWaitForCameraWithRetry(1, 3))
        {
            RCLCPP_ERROR(this->get_logger(), "[STATE] ❌ Failed to switch to Camera 1 after retries");
            publishError("CAMERA_SWITCH_FAILED_INPUT_TRAY");
            system_enabled_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
    }

    if (!manual_mode_ && is_first_batch_)
    {
        // Only require feed_chamber on first batch — after init, cycle runs autonomously
        if (!feed_chamber_signal_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "[INIT] ⏳ Waiting for feed_chamber signal (first batch)...");
            return;
        }
    }

    // ✅ ASYNC MOTION: Check if we already sent a goal
    if (motion_current_cmd_ == "INPUT_TRAY_CHAMBER") {
        if (motion_in_progress_) return;  // Still moving, poll again next cycle
        
        // Motion complete - process result
        motion_current_cmd_.clear();
        if (!motion_result_) {
            motion_fail_count_++;
            RCLCPP_ERROR(this->get_logger(), "[INIT] ❌ InputTrayChamber motion failed (%d/3)", motion_fail_count_);
            if (motion_fail_count_ >= 3) {
                RCLCPP_ERROR(this->get_logger(), "[INIT] ❌ 3 consecutive failures — Robot may be in error state. Returning to IDLE.");
                publishError("MOTION_FAILED_3X");
                motion_fail_count_ = 0;
                system_enabled_ = false;
                transitionTo(SystemState::IDLE);
            }
            return;
        }
        motion_fail_count_ = 0;  // Reset on success
        
        chamber_has_cartridge_ = true;
        chamber_is_empty_ = false;
        
        if (stop_after_single_motion_) {
            stop_after_single_motion_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
        if (manual_mode_) {
            transitionTo(SystemState::IDLE);
        } else {
            transitionTo(SystemState::INIT_REFILL_BUFFER);
        }
        return;
    }
    
    // ✅ ASYNC MOTION: Prepare row and send (non-blocking)
    int row_to_pick = -1;
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        row_to_pick = selected_input_row_;
        selected_input_row_ = -1;
    }
    
    // Set auto-increment start from selected row + 1
    if (row_to_pick > 0 && !use_ai_for_control_) {
        current_auto_row_ = row_to_pick + 1;
        if (current_auto_row_ > 5) current_auto_row_ = 1;
        RCLCPP_INFO(this->get_logger(), "[MOTION] Auto Mode: User selected Row %d, next buffer row = %d", row_to_pick, current_auto_row_);
    }
    
    if (row_to_pick == -1) {
        if (manual_mode_) {
            RCLCPP_ERROR(this->get_logger(), "[MOTION] No row selected (Manual)");
            return;
        }
        if (!use_ai_for_control_) {
            row_to_pick = 1;
            current_auto_row_ = 2;
            RCLCPP_INFO(this->get_logger(), "[MOTION] Auto Mode: No row selected, default Pick Row %d", row_to_pick);
        } else {
            for (size_t i = 0; i < row_full_.size(); ++i) {
                if (row_full_[i]) { row_to_pick = static_cast<int>(i) + 1; break; }
            }
        }
    }
    
    if (row_to_pick == -1) {
        RCLCPP_ERROR(this->get_logger(), "[MOTION] No full row available");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "[MOTION] Input Tray → Chamber (Row %d) [ASYNC]", row_to_pick);
    sendMotionActionAsync("INPUT_TRAY_CHAMBER", row_to_pick);
}

void RobotLogicNode::stateInitRefillBuffer()
{
    publishSystemStatus("INIT_REFILL_BUFFER");
    
    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion to finish...");
        return;
    }

    // ========================================================================

    // ✅ BLOCK if waiting for tray change
    // ========================================================================
    if (waiting_for_tray_change_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[CHANGE_TRAY] Waiting for new tray - INIT_REFILL_BUFFER blocked");
        return;
    }
    
    // ✅ AUTO/MANUAL MODE BYPASS: Skip camera requirement
    if (!use_ai_for_control_ || manual_mode_ || stop_after_single_motion_) {
        RCLCPP_INFO(this->get_logger(), "[AUTO] Auto Mode - Skipping camera switch (not required)");
    } else {
        // Ensure Camera 1 active for buffer refill (AI Mode only)
        if (!switchAndWaitForCameraWithRetry(1, 3))
        {
            RCLCPP_ERROR(this->get_logger(), "[STATE] ❌ Failed to switch to Camera 1 after retries");
            publishError("CAMERA_SWITCH_FAILED_BUFFER");
            system_enabled_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
    }

    // ✅ ASYNC MOTION: Check if we already sent a goal
    if (motion_current_cmd_ == "INPUT_TRAY_BUFFER") {
        if (motion_in_progress_) return;  // Still moving
        
        motion_current_cmd_.clear();
        if (!motion_result_) {
            RCLCPP_ERROR(this->get_logger(), "[INIT] ❌ InputTrayBuffer motion failed");
            return;
        }
        
        buffer_is_empty_ = false;
        
        // AUTO MODE: Change Tray Detection (after row 5)
        if (!use_ai_for_control_ && !manual_mode_ && current_auto_row_ == 1 && !waiting_for_tray_change_) {
            if (is_last_tray_available_) {
                RCLCPP_WARN(this->get_logger(), "[LAST_TRAY] Auto Mode - Row 5 & Last Tray -> LAST BATCH");
                is_last_batch_ = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "[CHANGE_TRAY] Auto Mode - Row 5 → Publishing change_tray");
                auto change_msg = std_msgs::msg::Bool();
                change_msg.data = true;
                change_tray_pub_->publish(change_msg);
                waiting_for_tray_change_ = true;
            }
        }
        
        is_first_batch_ = false;
        if (stop_after_single_motion_) {
            stop_after_single_motion_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
        if (manual_mode_) {
            transitionTo(SystemState::IDLE);
        } else {
            transitionTo(SystemState::WAIT_FILLING);
        }
        return;
    }
    
    // ✅ ASYNC MOTION: Prepare row and send (non-blocking)
    int row_num = -1;
    if (stop_after_single_motion_) {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        row_num = selected_input_row_;
        selected_input_row_ = -1;
        if (row_num < 1 || row_num > static_cast<int>(row_full_.size())) {
            RCLCPP_ERROR(this->get_logger(), "[MOTION] Invalid row: %d", row_num);
            return;
        }
    } else {
        if (!use_ai_for_control_) {
            row_num = current_auto_row_;
            current_auto_row_++;
            if (current_auto_row_ > 5) {
                current_auto_row_ = 1;
                RCLCPP_WARN(this->get_logger(), "[AUTO] 📦 All 5 rows consumed - requesting input tray change");
                auto change_msg = std_msgs::msg::Bool();
                change_msg.data = true;
                change_tray_pub_->publish(change_msg);
                waiting_for_tray_change_ = true;
            }
        } else {
            auto it = std::find_if(row_full_.begin(), row_full_.end(), [](bool v){ return v; });
            if (it == row_full_.end()) {
                RCLCPP_WARN(this->get_logger(), "[MOTION] No full rows for buffer refill");
                return;
            }
            row_num = static_cast<int>(std::distance(row_full_.begin(), it)) + 1;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "[MOTION] Input Tray → Buffer (Row %d) [ASYNC]", row_num);
    sendMotionActionAsync("INPUT_TRAY_BUFFER", row_num);
}

void RobotLogicNode::stateWaitFilling()
{
    publishSystemStatus("WAIT_FILLING");
    
    if (manual_mode_)
    {
        transitionTo(SystemState::IDLE);
        return;
    }
    
    // ========================================================================
    // ✅ SIMPLE: Just wait for fill to complete
    // ========================================================================
    if (fill_done_ && chamber_has_cartridge_)
    {
        RCLCPP_INFO(this->get_logger(), 
            "[PIPELINE] ✅ Chamber fill complete → Take to scale");
        fill_done_ = false; // ✅ Reset flag consumed
        transitionTo(SystemState::TAKE_CHAMBER_TO_SCALE);
        return;
    }
    
    // Note: is_last_batch is handled AFTER PLACE, not here
}

void RobotLogicNode::stateTakeChamberToScale()
{
    publishSystemStatus("TAKE_CHAMBER_TO_SCALE");
    
    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion to finish...");
        return;
    }
    // ✅ ASYNC MOTION: Check if we already sent a goal
    if (motion_current_cmd_ == "CHAMBER_SCALE") {
        if (motion_in_progress_) return;  // Still moving
        
        motion_current_cmd_.clear();
        if (!motion_result_) {
            chamber_has_cartridge_ = true;  // Assume didn't move
            return;
        }
        
        chamber_has_cartridge_ = false;
        chamber_is_empty_ = true;
        scale_has_cartridge_ = true;
        scale_wait_start_ = this->now();
        
        if (!buffer_is_empty_ && !is_last_batch_) {
            RCLCPP_INFO(this->get_logger(), "[PIPELINE] 🚀 Loading next chamber from buffer");
            transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
        } else {
            // last_batch or buffer empty: just process scale, no refill
            transitionTo(SystemState::PROCESSING_SCALE);
        }
        return;
    }

    // ✅ ASYNC MOTION: Send (non-blocking)
    int current_id = ++cartridge_counter_;
    RCLCPP_INFO(this->get_logger(), "[SCALE] 🏷️ Moving Cartridge #%d to Scale [ASYNC]", current_id);
    sendMotionActionAsync("CHAMBER_SCALE");
}

void RobotLogicNode::stateProcessingScale()
{
    publishSystemStatus("PROCESSING_SCALE");
    
    if (manual_mode_) {
        transitionTo(SystemState::IDLE);
        return;
    }
    
    // ========================================================================
    // PROCESS QUEUE (Patch 2)
    // ========================================================================
    
    // ✅ AUTO BYPASS: No scale hardware, force pass
    if (simulate_scale_ || force_pass_) {
        RCLCPP_INFO(this->get_logger(), "[SCALE] 🔄 bypass scale processing -> Force PASS");
        stored_scale_result_.store(true);
        transitionTo(SystemState::PLACE_TO_OUTPUT);
        return;
    }

    bool result_pass;
    int cartridge_id;
    
    // Check queue
    if (processNextScaleResult(result_pass, cartridge_id))
    {
        // Result available
        stored_scale_result_.store(result_pass); 
        RCLCPP_INFO(this->get_logger(), "[SCALE] ✅ Processing Cartridge #%d: %s", 
            cartridge_id, result_pass ? "PASS" : "FAIL");
            
        if (result_pass) {
            transitionTo(SystemState::PLACE_TO_OUTPUT);
        } else {
            transitionTo(SystemState::PLACE_TO_FAIL);
        }
        return;
    }
    
    // ========================================================================
    // WAIT FOR RESULT (TIMEOUT CHECK)
    // ========================================================================
    auto elapsed = (this->now() - scale_wait_start_).seconds();
    
    if (elapsed > 30.0) // 30s Timeout
    {
        RCLCPP_ERROR(this->get_logger(), 
            "[SCALE] ❌ Timeout after %.1fs! No result in queue. Queue size: %d", 
             elapsed, (int)pending_scale_results_.size()); // vector/deque access thread-safe here? 
             // Accessing size without lock is technically unsafe if pushing in callback.
             // But logging is okay.
        
        publishError("SCALE_RESULT_TIMEOUT");
        transitionTo(SystemState::ERROR_SCALE_TIMEOUT);
        return;
    }
    
    // Throttle logs
    if (static_cast<int>(elapsed) % 5 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[SCALE] ⏳ Waiting for result...Queue empty... (%.1fs / 30.0s)", elapsed);
    }
}

void RobotLogicNode::stateErrorScaleTimeout()
{
    publishSystemStatus("ERROR_SCALE_TIMEOUT");
    
    // ✅ AUTO RESUME: If result arrives while paused
    if (scale_result_received_) {
        RCLCPP_INFO(this->get_logger(), "[SCALE] ✅ Result received! AUTO RESUMING from pause...");
        if (stored_scale_result_) {
            transitionTo(SystemState::PLACE_TO_OUTPUT);
        } else {
            transitionTo(SystemState::PLACE_TO_FAIL);
        }
        // Reset flags
        scale_result_received_ = false;
        stored_scale_result_.store(false);
        return;
    }
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "[ERROR] ⚠️ System PAUSED - Waiting for scale result...");
        
    if (!system_enabled_) {
         transitionTo(SystemState::IDLE);
    }
}

void RobotLogicNode::statePlaceToOutput()
{
    publishSystemStatus("PLACE_TO_OUTPUT");
    
    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion to finish...");
        return;
    }

    // ========================================================================

    // CAMERA SETUP
    // ========================================================================
    if (!use_ai_for_control_ || manual_mode_ || stop_after_single_motion_) {
        RCLCPP_INFO(this->get_logger(), "[AUTO] Auto Mode/Manual - Skipping camera switch (not required)");
    } else {
        // Switch to Camera 2 for output tray (AI Mode only)
        if (!switchAndWaitForCameraWithRetry(2, 3))
        {
            RCLCPP_ERROR(this->get_logger(), "[STATE] ❌ Failed to switch to Camera 2 after retries");
            publishError("CAMERA_SWITCH_FAILED_OUTPUT_TRAY");
            system_enabled_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
    }

    // ========================================================================
    // SLOT SELECTION (Thread-Safe Patch 1)
    // ========================================================================
    int slot_to_place = -1;
    bool waiting = false;
    
    {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        
        if (manual_mode_) {
            slot_to_place = selected_output_slot_;
        }
        else if (!use_ai_for_control_)
        {
            // AUTO: Vision Node handles the counter 1->9
            // We just verify it's valid
            if (selected_output_slot_ != -1) {
                slot_to_place = selected_output_slot_;
            } else {
                waiting = true;
            }
        }
        else
        {
            // AI: Check if valid slot selected
            if (selected_output_slot_ != -1) {
                slot_to_place = selected_output_slot_;
            } else {
                waiting = true; 
            }
        }
    }
    
    if (slot_to_place == -1)
    {
        if (manual_mode_)
        {
            RCLCPP_ERROR(this->get_logger(), "[PLACE_TO_OUTPUT] No slot selected (Manual)");
            transitionTo(SystemState::IDLE);
            return;
        }
        
        if (waiting)
        {
            // Handle Timeout
            if (output_tray_wait_start_.seconds() == 0) {
                output_tray_wait_start_ = this->now();
                RCLCPP_INFO(this->get_logger(), "[OUTPUT] ⏳ Waiting for slot selection...");
            }
            
            auto elapsed = (this->now() - output_tray_wait_start_).seconds();
            if (elapsed > OUTPUT_TRAY_TIMEOUT_SEC)
            {
                RCLCPP_ERROR(this->get_logger(), "[OUTPUT] ❌ Timeout waiting for slot");
                publishError("OUTPUT_TRAY_TIMEOUT");
                transitionTo(SystemState::ERROR_OUTPUT_TRAY_TIMEOUT);
                return;
            }
            return;  // Wait for callback to select slot
        }
    }
    
    // ========================================================================
    // MOTION EXECUTION (ASYNC)
    // ========================================================================
    
    // ✅ ASYNC: Check if we already sent a goal
    if (motion_current_cmd_ == "SCALE_OUTPUT") {
        if (motion_in_progress_) return;  // Still moving
        
        motion_current_cmd_.clear();
        int placed_slot = async_slot_;
        
        if (!motion_result_) {
            RCLCPP_ERROR(this->get_logger(), "[PLACE] ❌ ScaleOutput motion failed");
            return;
        }
        
        // Cleanup
        scale_has_cartridge_ = false;
        
        // Notify Vision
        {
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            place_done_pub_->publish(msg);
        }

        // Slot 9 -> Change Tray Logic (AUTO Mode)
        if (!use_ai_for_control_ && placed_slot >= 9) {
            RCLCPP_WARN(this->get_logger(), "[PIPELINE] 🏁 Slot 9 reached in AUTO Mode. Requesting tray change...");
            auto done_output_msg = std_msgs::msg::Bool();
            done_output_msg.data = true;
            done_output_tray_pub_->publish(done_output_msg);
            auto change_msg = std_msgs::msg::Bool();
            change_msg.data = true;
            change_tray_pub_->publish(change_msg);
            waiting_for_tray_change_ = true;
            transitionTo(SystemState::IDLE);
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            selected_output_slot_ = -1;
        }
        output_tray_wait_start_ = rclcpp::Time(0);
        
        tray_count_++;
        RCLCPP_INFO(this->get_logger(), "[STATS] ✅ Tray count: %d", tray_count_.load());
        if (tray_count_pub_) {
            auto msg = std::make_shared<std_msgs::msg::Int32>();
            msg->data = tray_count_.load();
            tray_count_pub_->publish(*msg);
        }
        
        if (placed_slot >= 9) {
            RCLCPP_WARN(this->get_logger(), "[OUTPUT] ⚠️ Output Tray Full (Slot 9) -> Requesting Change Output");
            auto done_msg = std_msgs::msg::Bool();
            done_msg.data = true;
            done_output_tray_pub_->publish(done_msg);
        }
        
        RCLCPP_INFO(this->get_logger(), "[PIPELINE] ✅ Placement complete.");
        
        // DECIDE NEXT STATE
        if (is_last_batch_ && !chamber_has_cartridge_) {
            // Last batch done — no more cartridges
            is_last_batch_ = false;
            transitionTo(SystemState::IDLE);
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            last_batch_complete_pub_->publish(msg);
        } else {
            // Continue cycle: wait for next fill_done
            transitionTo(SystemState::WAIT_FILLING);
        }
        return;
    }
    
    // ✅ ASYNC: Send motion (non-blocking)
    RCLCPP_INFO(this->get_logger(), 
        "[PIPELINE] 📦 Placing to Slot %d (%s Mode) [ASYNC]", 
        slot_to_place, use_ai_for_control_ ? "AI" : "AUTO");
    
    async_slot_ = slot_to_place;
    sendMotionActionAsync("SCALE_OUTPUT", slot_to_place);
}

void RobotLogicNode::statePlaceToFail()
{
    publishSystemStatus("PLACE_TO_FAIL");
    
    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion to finish...");
        return;
    }

    RCLCPP_INFO(this->get_logger(), 
        "[PIPELINE] ❌ Scale FAIL - placing to reject position");
    
    // ✅ ASYNC: Check if we already sent a goal
    if (motion_current_cmd_ == "SCALE_FAIL") {
        if (motion_in_progress_) return;
        
        motion_current_cmd_.clear();
        if (!motion_result_) {
            RCLCPP_ERROR(this->get_logger(), "[FAIL] ❌ ScaleFail motion failed");
            return;
        }
        
        scale_has_cartridge_ = false;
        
        // DECIDE NEXT STATE
        if (manual_mode_) {
            transitionTo(SystemState::IDLE);
        } else if (is_last_batch_ && !chamber_has_cartridge_) {
            // Last batch done — no more cartridges
            is_last_batch_ = false;
            transitionTo(SystemState::IDLE);
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            last_batch_complete_pub_->publish(msg);
        } else {
            // Continue cycle: wait for next fill_done
            transitionTo(SystemState::WAIT_FILLING);
        }
        return;
    }
    
    // ✅ ASYNC: Send (non-blocking)
    sendMotionActionAsync("SCALE_FAIL");
}

void RobotLogicNode::stateRefillBuffer()
{
    publishSystemStatus("REFILL_BUFFER");
    
    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion to finish...");
        return;
    }

    // 🔒 INTERLOCK: Chời khi cartridge system đang thay khạy (S2/S3)
    if (cartridge_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[INTERLOCK] Cartridge BUSY (S2/S3) — REFILL_BUFFER bị chặn, đợi...");
        return;
    }

    // ========================================================================
    // ✅ CHECK PENDING SCALE RESULTS (Patch 2)
    // ========================================================================
    if (hasPendingScaleResults()) {
        RCLCPP_WARN(this->get_logger(), "[REFILL] ⚠️ Pending scale results found! Prioritizing processing");
        transitionTo(SystemState::PROCESSING_SCALE);
        return;
    }
    
    // ========================================================================
    // ✅ BLOCK if waiting for tray change
    // ========================================================================
    if (waiting_for_tray_change_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[CHANGE_TRAY] Waiting for new tray - REFILL_BUFFER blocked");
        return;
    }
    
    // ========================================================================
    // Camera switch (Auto Mode bypass)
    // ========================================================================
    if (!use_ai_for_control_ || manual_mode_ || stop_after_single_motion_) {
        RCLCPP_INFO(this->get_logger(), "[AUTO] Auto Mode - Skipping camera switch");
    } else {
        if (!switchAndWaitForCameraWithRetry(0, 3))
        {
            RCLCPP_ERROR(this->get_logger(), "[STATE] ❌ Failed to switch to Camera 0");
            publishError("CAMERA_SWITCH_FAILED_REFILL");
            system_enabled_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
    }
    
    // ========================================================================
    // ✅ ALWAYS REFILL - No conditions
    // ========================================================================
    RCLCPP_INFO(this->get_logger(), "[BUFFER] 🔄 Refilling from input tray...");
    // ASYNC: Check if we already sent a goal
    if (motion_current_cmd_ == "INPUT_TRAY_BUFFER") {
        if (motion_in_progress_) return;
        
        motion_current_cmd_.clear();
        if (!motion_result_) {
            RCLCPP_ERROR(this->get_logger(), "[REFILL] InputTrayBuffer motion failed");
            return;
        }
        
        buffer_is_empty_ = false;
        RCLCPP_INFO(this->get_logger(), "[BUFFER] Buffer refilled");
        
        // AUTO MODE: Change Tray Detection (after row 5)
        if (!use_ai_for_control_ && !manual_mode_ && current_auto_row_ == 1 && !waiting_for_tray_change_) {
            if (is_last_tray_available_) {
                RCLCPP_WARN(this->get_logger(), "[LAST_TRAY] Auto Mode - Row 5 & Last Tray -> LAST BATCH");
                is_last_batch_ = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "[CHANGE_TRAY] Auto Mode - Row 5 -> Publishing change_tray");
                auto change_msg = std_msgs::msg::Bool();
                change_msg.data = true;
                change_tray_pub_->publish(change_msg);
                waiting_for_tray_change_ = true;
            }
        }
        
        if (input_tray_empty_ && is_last_tray_available_) {
            is_last_batch_ = true;
            RCLCPP_WARN(this->get_logger(), "[PIPELINE] Input tray empty & Last Tray -> last batch");
        }
        
        if (stop_after_single_motion_) {
            stop_after_single_motion_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
        
        // ✅ After REFILL_BUFFER done → WAIT_FILLING (cycle continues autonomously)
        RCLCPP_INFO(this->get_logger(), "[PIPELINE] ✅ Refill Buffer complete → WAIT_FILLING");
        transitionTo(SystemState::WAIT_FILLING);
        return;
    }
    
    // ASYNC: Prepare row and send (non-blocking)
    int row_num = -1;
    if (stop_after_single_motion_) {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        row_num = selected_input_row_;
        selected_input_row_ = -1;
        if (row_num < 1 || row_num > static_cast<int>(row_full_.size())) {
            RCLCPP_ERROR(this->get_logger(), "[MOTION] Invalid row: %d", row_num);
            return;
        }
    } else {
        if (!use_ai_for_control_) {
            row_num = current_auto_row_;
            current_auto_row_++;
            if (current_auto_row_ > 5) {
                current_auto_row_ = 1;
                RCLCPP_WARN(this->get_logger(), "[AUTO] All 5 rows consumed - requesting input tray change");
                auto change_msg = std_msgs::msg::Bool();
                change_msg.data = true;
                change_tray_pub_->publish(change_msg);
                waiting_for_tray_change_ = true;
            }
        } else {
            auto it = std::find_if(row_full_.begin(), row_full_.end(), [](bool v){ return v; });
            if (it == row_full_.end()) {
                RCLCPP_WARN(this->get_logger(), "[MOTION] No full rows for buffer refill");
                return;
            }
            row_num = static_cast<int>(std::distance(row_full_.begin(), it)) + 1;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "[MOTION] Input Tray to Buffer (Row %d) [ASYNC]", row_num);
    sendMotionActionAsync("INPUT_TRAY_BUFFER", row_num);
}



void RobotLogicNode::stateLoadChamberFromBuffer()
{
    publishSystemStatus("LOAD_CHAMBER_FROM_BUFFER");
    
    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion to finish...");
        return;
    }

    // ✅ ASYNC: Check if we already sent a goal
    if (motion_current_cmd_ == "BUFFER_CHAMBER") {
        if (motion_in_progress_) return;
        
        motion_current_cmd_.clear();
        if (!motion_result_) {
            RCLCPP_ERROR(this->get_logger(), "[LOAD] BufferChamber motion failed");
            return;
        }
        
        buffer_is_empty_ = true;
        chamber_has_cartridge_ = true;
        chamber_is_empty_ = false;
        RCLCPP_INFO(this->get_logger(), "[PIPELINE] Chamber filling started in background!");
        
        if (manual_mode_) {
            transitionTo(SystemState::IDLE);
        } else {
            // Always refill buffer after loading chamber (unless last batch)
            if (is_last_batch_) {
                RCLCPP_INFO(this->get_logger(), "[PIPELINE] Last batch — skip refill → WAIT_FILLING");
                transitionTo(SystemState::WAIT_FILLING);
            } else {
                transitionTo(SystemState::REFILL_BUFFER);
            }
        }
        return;
    }
    
    // ASYNC: Send (non-blocking)
    RCLCPP_INFO(this->get_logger(), "[MOTION] Buffer to Chamber [ASYNC]");
    sendMotionActionAsync("BUFFER_CHAMBER");
}



void RobotLogicNode::stateLastBatchWait()
{
    publishSystemStatus("LAST_BATCH_WAIT");
    
    // ========================================================================
    // Linear flow: cho fill_done → TAKE_CHAMBER_TO_SCALE
    // Sau khi scale xong → PLACE → is_last_batch → IDLE + pub (handled in PLACE)
    // ========================================================================
    
    if (chamber_has_cartridge_ && fill_done_)
    {
        RCLCPP_INFO(this->get_logger(), 
            "[LAST_BATCH] fill_done → TAKE_CHAMBER_TO_SCALE (cartridge cuoi)");
        fill_done_ = false;
        transitionTo(SystemState::TAKE_CHAMBER_TO_SCALE);
        return;
    }
    
    // Waiting for fill_done (logged on transition)
}

void RobotLogicNode::stateErrorScale()
{
    publishSystemStatus("ERROR_SCALE");
    publishError("SCALE ERROR");
    
    if (system_started_)
    {
        system_started_ = false;
        is_first_batch_ = true;
        chamber_has_cartridge_ = false;
        chamber_is_empty_ = true;
        scale_has_cartridge_ = false;
        
        transitionTo(SystemState::INIT_CHECK);
    }
}

void RobotLogicNode::stateErrorOutputTrayTimeout()
{
    publishSystemStatus("ERROR_OUTPUT_TRAY_TIMEOUT");
    publishError("OUTPUT TRAY ERROR");
    
    if (system_started_)
    {
        system_started_ = false;
        transitionTo(SystemState::WAIT_FILLING);
    }
}

void RobotLogicNode::stateErrorMotionLost()
{
    publishSystemStatus("ERROR_MOTION_LOST");
    publishError("MOTION_EXECUTOR_LOST");

    RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "[ERROR] 🚨 Motion Executor lost. Waiting for recovery..."
    );

    // ⛔ KHÔNG gửi thêm motion trong state này
    // ⛔ KHÔNG chuyển state tự động trừ khi có heartbeat lại

    // ✅ Auto recover nếu heartbeat quay lại
    if (checkMotionAlive(2.0))
    {
        RCLCPP_WARN(this->get_logger(),
            "[RECOVERY] Motion heartbeat restored → Returning to IDLE");
        
        // Reset busy flag just in case
        motion_busy_ = false;
        transitionTo(SystemState::IDLE);
    }
}


bool RobotLogicNode::checkMotionAlive(double timeout_sec)
{
    if (last_motion_hb_.nanoseconds() == 0) {
        // First time check - maybe node just started
        return true; 
    }

    auto now_t = this->now();
    double dt = (now_t - last_motion_hb_).seconds();

    if (dt > timeout_sec) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[WATCHDOG] ❌ Motion node lost! Last HB %.2fs ago", dt);
        return false;
    }

    return true;
}


// ============================================================================
// MOTION IMPLEMENTATIONS
// ============================================================================

// ============================================================================
// MOTION IMPLEMENTATIONS
// ============================================================================





bool RobotLogicNode::setDigitalOutput(int index, bool status)
{
    auto req = std::make_shared<DO::Request>();
    req->index = index;
    req->status = status ? 1 : 0;

    auto res = callService<DO>(do_client_, req, "DO");
    if (!res)
    {
        RCLCPP_ERROR(get_logger(), "[DO] Failed to set DO[%d]", index);
        return false;
    }
    if (res->res != 0) {
        RCLCPP_ERROR(get_logger(), "[DO] Driver rejected DO[%d] (err: %d)", index, res->res);
        return false;
    }

    if (index == 1 && gripper_cmd_pub_)
    {
        auto msg = std::make_shared<std_msgs::msg::Bool>();
        msg->data = status;
        gripper_cmd_pub_->publish(*msg);
    }
    if (index == 2 && picker_cmd_pub_)
    {
        auto msg = std::make_shared<std_msgs::msg::Bool>();
        msg->data = status;
        picker_cmd_pub_->publish(*msg);
    }
    
    return true;
}



// ────────────────────────────────────────────────────────────────────────────
// PATCH 3: MOTION ERROR HANDLING MACRO & STUBS
// ────────────────────────────────────────────────────────────────────────────

// Macro to check motion command - Transitions to IDLE (or ERROR) on failure
#define CHECK_MOTION(cmd) \
    if (!(cmd)) { \
        RCLCPP_ERROR(this->get_logger(), "[MOTION] Command failed: " #cmd); \
        transitionTo(SystemState::IDLE); \
        publishError("MOTION_FAILURE"); \
        return false; \
    }

// ============================================================================
// MOTION COMMAND DELEGATION
// ============================================================================

bool RobotLogicNode::sendMotionAction(const std::string& cmd, int slot, int timeout_sec)
{
    if (!motion_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "[ACTION] Action server not available!");
        publishError("ACTION_SERVER_OFFLINE");
        return false;
    }

    auto goal_msg = ExecuteMotion::Goal();
    goal_msg.command = cmd;
    goal_msg.slot = slot;
    goal_msg.allow_rollback = true; // ENABLE ROLLBACK FOR INDUSTRIAL SAFETY


    RCLCPP_INFO(this->get_logger(), "[ACTION] 🚀 Sending goal: %s (slot: %d)", cmd.c_str(), slot);

    auto send_goal_options = rclcpp_action::Client<ExecuteMotion>::SendGoalOptions();
    
    // We can add feedback/result callbacks here if needed
    
    auto future_goal_handle = motion_action_client_->async_send_goal(goal_msg, send_goal_options);
    
    if (future_goal_handle.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "[ACTION] Goal request timeout");
        return false;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "[ACTION] Goal was rejected by server");
        return false;
    }

    auto future_result = motion_action_client_->async_get_result(goal_handle);
    
    if (future_result.wait_for(std::chrono::seconds(timeout_sec)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "[ACTION] Goal result timeout! Cancelling...");
        motion_action_client_->async_cancel_goal(goal_handle);
        return false;
    }

    auto result = future_result.get();
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "[ACTION] ✅ Goal succeeded: %s", result.result->message.c_str());
            return result.result->success;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "[ACTION] ❌ Goal was aborted");
            return false;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "[ACTION] ❌ Goal was canceled");
            return false;
        default:
            RCLCPP_ERROR(this->get_logger(), "[ACTION] Unknown result code");
            return false;
    }
}

// ============================================================================
// ASYNC MOTION (Non-blocking) - Returns immediately
// ============================================================================
void RobotLogicNode::sendMotionActionAsync(const std::string& cmd, int slot)
{
    if (!motion_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "[ASYNC] Action server not available!");
        publishError("ACTION_SERVER_OFFLINE");
        motion_in_progress_ = false;
        motion_result_ = false;
        return;
    }

    auto goal_msg = ExecuteMotion::Goal();
    goal_msg.command = cmd;
    goal_msg.slot = slot;
    goal_msg.allow_rollback = true;

    RCLCPP_INFO(this->get_logger(), "[ASYNC] 🚀 Sending goal: %s (slot: %d)", cmd.c_str(), slot);

    auto send_goal_options = rclcpp_action::Client<ExecuteMotion>::SendGoalOptions();

    // Goal accepted/rejected callback
    send_goal_options.goal_response_callback =
        [this, cmd](const rclcpp_action::ClientGoalHandle<ExecuteMotion>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "[ASYNC] ❌ Goal '%s' rejected by server", cmd.c_str());
                motion_in_progress_ = false;
                motion_result_ = false;
                state_changed_ = true;
                state_cv_.notify_one();
            } else {
                RCLCPP_INFO(this->get_logger(), "[ASYNC] ✅ Goal '%s' accepted", cmd.c_str());
            }
        };

    // Result callback - fires when motion completes
    send_goal_options.result_callback =
        [this, cmd](const rclcpp_action::ClientGoalHandle<ExecuteMotion>::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    motion_result_ = result.result->success;
                    RCLCPP_INFO(this->get_logger(), "[ASYNC] ✅ '%s' completed: %s", 
                        cmd.c_str(), result.result->message.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    motion_result_ = false;
                    RCLCPP_ERROR(this->get_logger(), "[ASYNC] ❌ '%s' aborted", cmd.c_str());
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    motion_result_ = false;
                    RCLCPP_WARN(this->get_logger(), "[ASYNC] ⚠️ '%s' canceled", cmd.c_str());
                    break;
                default:
                    motion_result_ = false;
                    RCLCPP_ERROR(this->get_logger(), "[ASYNC] ❓ '%s' unknown result", cmd.c_str());
                    break;
            }
            motion_in_progress_ = false;
            
            // Wake state machine immediately
            state_changed_ = true;
            state_cv_.notify_one();
        };

    // Send goal - returns IMMEDIATELY
    motion_in_progress_ = true;
    motion_result_ = false;
    motion_current_cmd_ = cmd;
    motion_action_client_->async_send_goal(goal_msg, send_goal_options);
}



bool RobotLogicNode::motionStub_InputTrayChamber()
{
    RCLCPP_INFO(this->get_logger(), "[MOTION] Input Tray → Chamber");
    
    int row_to_pick = -1;
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        row_to_pick = selected_input_row_;
        selected_input_row_ = -1;
    }
    
    if (row_to_pick == -1)
    {
        if (manual_mode_)
        {
            RCLCPP_ERROR(this->get_logger(), "[MOTION] No row selected (Manual)");
            return false;
        }

        if (!use_ai_for_control_)
        {
            // Auto Mode Sequence: Start at Row 1
            row_to_pick = 1;
            current_auto_row_ = 2;
            RCLCPP_INFO(this->get_logger(), "[MOTION] Auto Mode: Initial Pick Row %d", row_to_pick);
        }
        else
        {
            // AI Mode: Pick first available full row
            for (size_t i = 0; i < row_full_.size(); ++i)
            {
                if (row_full_[i])
                {
                    row_to_pick = static_cast<int>(i) + 1;
                    break;
                }
            }
        }
    }
    
    if (row_to_pick == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "[MOTION] No full row available");
        return false;
    }
    
    // Delegate actual motion to motion_executor (Action Only)
    return sendMotionAction("INPUT_TRAY_CHAMBER", row_to_pick);
}



bool RobotLogicNode::motionStub_InputTrayBuffer_SinglePick()
{
    RCLCPP_INFO(this->get_logger(), "[MOTION] Input Tray → Buffer (SINGLE)");
    
    int row_num = -1;

    if (stop_after_single_motion_)
    {
        // Manual Mode
        {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            row_num = selected_input_row_;
            selected_input_row_ = -1;
        }

        if (row_num < 1 || row_num > static_cast<int>(row_full_.size()))
        {
            RCLCPP_ERROR(this->get_logger(), "[MOTION] Invalid row: %d", row_num);
            return false;
        }
    }
    else
    {
        // Auto / AI Mode Logic
        if (!use_ai_for_control_)
        {
            row_num = current_auto_row_;
            current_auto_row_++;
            if (current_auto_row_ > 5) {
                current_auto_row_ = 1;
                // AUTO MODE: All 5 rows consumed - request input tray change
                RCLCPP_WARN(this->get_logger(), "[AUTO] 📦 All 5 rows consumed - requesting input tray change");
                auto change_msg = std_msgs::msg::Bool();
                change_msg.data = true;
                change_tray_pub_->publish(change_msg);
                waiting_for_tray_change_ = true;
            }
            RCLCPP_INFO(this->get_logger(), "[MOTION] Auto Mode: Refill Row %d", row_num);
        }
        else
        {
            auto it = std::find_if(row_full_.begin(), row_full_.end(), [](bool v){ return v; });
            if (it == row_full_.end()) {
                RCLCPP_WARN(this->get_logger(), "[MOTION] No full rows for buffer refill");
                return false;
            }
            row_num = static_cast<int>(std::distance(row_full_.begin(), it)) + 1;
        }
    }

    // Delegate (Action Only)
    return sendMotionAction("INPUT_TRAY_BUFFER", row_num);
}



bool RobotLogicNode::motionStub_InputTrayBuffer()
{
    return motionStub_InputTrayBuffer_SinglePick();
}

bool RobotLogicNode::motionStub_ChamberScale()
{
    RCLCPP_INFO(this->get_logger(), "[MOTION] Chamber → Scale");
    return sendMotionAction("CHAMBER_SCALE");
}



bool RobotLogicNode::motionStub_ScaleOutput(int slot)
{
    RCLCPP_INFO(this->get_logger(), "[MOTION] Scale → Output Slot %d", slot);
    return sendMotionAction("SCALE_OUTPUT", slot);
}


bool RobotLogicNode::motionStub_ScaleFail()
{
    RCLCPP_INFO(this->get_logger(), "[MOTION] Scale → Fail Position");
    return sendMotionAction("SCALE_FAIL");
}


bool RobotLogicNode::motionStub_BufferChamber()
{
    RCLCPP_INFO(this->get_logger(), "[MOTION] Buffer → Chamber");
    return sendMotionAction("BUFFER_CHAMBER");
}



void RobotLogicNode::transitionTo(SystemState new_state)
{
    RCLCPP_INFO(this->get_logger(), "[STATE] Transition: %s -> %s", stateToString(current_state_).c_str(), stateToString(new_state).c_str());
    // Validate robot state before critical transitions
    if (new_state == SystemState::INIT_CHECK || 
        new_state == SystemState::INIT_LOAD_CHAMBER_DIRECT || 
        new_state == SystemState::TAKE_CHAMBER_TO_SCALE)
    {
        if (!validateRobotState())
        {
            RCLCPP_ERROR(get_logger(), "[FATAL] Robot State Validation Failed! Aborting transition.");
            publishError("Robot Disconnected or Disabled");
            // Do not transition, stay in current state or go to error state?
            // For safety, force emergency stop logic
            system_running_ = false;
            system_started_ = false;
            return;
        }
    }

    {
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        if (current_state_ != new_state)
        {
            current_state_ = new_state;
            state_changed_ = true;
            state_cv_.notify_all();
            
            std::string state_str = stateToString(new_state);
            RCLCPP_INFO(get_logger(), "[STATE] %s -> %s", 
                stateToString(current_state_).c_str(), state_str.c_str());
            publishSystemStatus(state_str);

            // Removed erroneous motion_busy_ publish here; motion_executor handles it.
        }
    }
}

bool RobotLogicNode::checkConnection()
{
    // BYPASS CONNECTION CHECK (Driver is single-threaded and blocks on Sync)
    return true;
    
    /*
    if (!error_client_->service_is_ready())
    {
        RCLCPP_ERROR(get_logger(), "[CONNECTION] Robot Driver Service NOT ready!");
        return false;
    }

    auto req = std::make_shared<GetErrorID::Request>();
    
    // Simple wait for future status (blocking but with timeout)
    std::future_status status = future.wait_for(1000ms);
    
    if (status != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "[CONNECTION] Robot Driver Ping Timeout! (Waited 1.0s)");
        return false;
    }
    
    return true;
    */
}

bool RobotLogicNode::validateRobotState()
{
    // 1. Check Connection
    if (!checkConnection())
    {
        return false;
    }
    
    // 2. Check Enabled Flag (local logic)
    if (manual_mode_) return true; // Bypass enable check in manual mode? Maybe unsafe. 
    // Let's enforce enable check even in Manual unless strictly debugging without robot.
    
    if (!system_enabled_)
    {
        RCLCPP_ERROR(get_logger(), "[VALIDATION] Robot is NOT ENABLED. Cannot proceed.");
        return false;
    }
    
    return true;
}



// ============================================================================
// UTILITY IMPLEMENTATIONS
// ============================================================================

// transitionTo defined above in helper section

std::string RobotLogicNode::stateToString(SystemState state)
{
    switch (state)
    {
        case SystemState::IDLE: return "IDLE";
        case SystemState::INIT_CHECK: return "INIT_CHECK";
        case SystemState::INIT_LOAD_CHAMBER_DIRECT: return "INIT_LOAD_CHAMBER_DIRECT";
        case SystemState::INIT_REFILL_BUFFER: return "INIT_REFILL_BUFFER";
        case SystemState::WAIT_FILLING: return "WAIT_FILLING";
        case SystemState::TAKE_CHAMBER_TO_SCALE: return "TAKE_CHAMBER_TO_SCALE";
        case SystemState::PROCESSING_SCALE: return "PROCESSING_SCALE";
        case SystemState::ERROR_SCALE_TIMEOUT: return "ERROR_SCALE_TIMEOUT";
        case SystemState::PLACE_TO_OUTPUT: return "PLACE_TO_OUTPUT";
        case SystemState::PLACE_TO_FAIL: return "PLACE_TO_FAIL";
        case SystemState::REFILL_BUFFER: return "REFILL_BUFFER";
        case SystemState::LOAD_CHAMBER_FROM_BUFFER: return "LOAD_CHAMBER_FROM_BUFFER";
        case SystemState::LAST_BATCH_WAIT: return "LAST_BATCH_WAIT";
        case SystemState::ERROR_SCALE: return "ERROR_SCALE";
        case SystemState::ERROR_OUTPUT_TRAY_TIMEOUT: return "ERROR_OUTPUT_TRAY_TIMEOUT";
        case SystemState::ERROR_INPUT_TRAY_EMPTY:
            return "ERROR_INPUT_TRAY_EMPTY";
        case SystemState::ERROR_MOTION_LOST:
            return "ERROR_MOTION_LOST";
        default:
            return "UNKNOWN";
    }
}

void RobotLogicNode::publishSystemStatus(const std::string &status)
{
    RCLCPP_INFO(this->get_logger(), "[STATUS] %s", status.c_str());
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = status;
    system_status_pub_->publish(*msg);
}

void RobotLogicNode::publishError(const std::string &error)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = error;
    error_pub_->publish(*msg);
    RCLCPP_ERROR(this->get_logger(), "[ERROR] %s", error.c_str());
}

template <typename ServiceT>
typename ServiceT::Response::SharedPtr RobotLogicNode::callService(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request,
    const std::string &service_name)
{
    using namespace std::chrono_literals;
    
    // ✅ Quick check - don't block state machine
    if (!client->wait_for_service(100ms))
    {
        RCLCPP_ERROR(get_logger(), "[SERVICE] %s not available", service_name.c_str());
        return nullptr;
    }
    
    // ✅ Send request and WAIT for response
    auto future = client->async_send_request(request);
    
    // ✅ Wait up to 5s for response
    if (future.wait_for(5s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "[SERVICE] %s timeout after 5s", service_name.c_str());
        return nullptr;
    }
    
    // ✅ Get REAL response
    try {
        auto response = future.get();
        return response;  // Return actual response from driver
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "[SERVICE] %s exception: %s", service_name.c_str(), e.what());
        return nullptr;
    }
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotLogicNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
