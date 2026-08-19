#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
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
#include "dobot_msgs_v3/srv/pause.hpp"
#include "dobot_msgs_v3/srv/continues.hpp"
#include "dobot_msgs_v3/srv/disable_robot.hpp"
#include "dobot_msgs_v3/srv/emergency_stop.hpp"
#include "dobot_msgs_v3/srv/stop_script.hpp"

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
#include <chrono>
#include <thread>

using EnableRobot   = dobot_msgs_v3::srv::EnableRobot;
using DO            = dobot_msgs_v3::srv::DO;
using RobotMode     = dobot_msgs_v3::srv::RobotMode;
using ClearError    = dobot_msgs_v3::srv::ClearError;
using GetErrorID    = dobot_msgs_v3::srv::GetErrorID;
using ResetRobot    = dobot_msgs_v3::srv::ResetRobot;
using Pause         = dobot_msgs_v3::srv::Pause;
using Continues     = dobot_msgs_v3::srv::Continues;
using DisableRobot  = dobot_msgs_v3::srv::DisableRobot;
using EmergencyStop = dobot_msgs_v3::srv::EmergencyStop;
using StopScript    = dobot_msgs_v3::srv::StopScript;

// ============================================================================
// ENUMS
// ============================================================================

enum class SystemState
{
    IDLE,
    INIT_CHECK,
    INIT_LOAD_CHAMBER_DIRECT,   // Row X → CHAMBER (first pick)
    INIT_REFILL_BUFFER,         // Row X+1 → BUFFER (second pick, init only)
    AI_CHECK_INPUT_TRAY,        // At index 0: wait fresh READY/EMPTY decision
    WAIT_FILLING,               // Wait fill_done; check tray flags
    TAKE_CHAMBER_TO_SCALE,      // CHAMBER → SCALE (8 cartridges)
    LOAD_CHAMBER_FROM_BUFFER,   // BUFFER → CHAMBER
    WAIT_RESUME_CHOICE,         // feed_chamber timeout → wait operator popup (INIT_LOAD_CHAMBER_DIRECT or LOAD_CHAMBER_FROM_BUFFER)
    WAIT_SCALE_CHOICE,          // loadcell silent 150s in PROCESSING_SCALE → wait operator popup (WAIT_FILLING / PLACE_TO_OUTPUT / PLACE_TO_FAIL)
    REFILL_BUFFER,              // INPUT_TRAY → BUFFER (if tray available)
    PROCESSING_SCALE,           // Wait scale result
    ERROR_SCALE_TIMEOUT,
    PLACE_TO_OUTPUT,            // SCALE → OUTPUT slot
    PLACE_TO_FAIL,              // SCALE → FAIL position
    LAST_BATCH_WAIT,
    ERROR_SCALE,
    ERROR_INPUT_TRAY_EMPTY,
    ERROR_MOTION_LOST
};

enum class ControlMode : uint8_t
{
    MANUAL = 0,
    AUTO   = 1,
    AI     = 2
};

// ============================================================================
// MAIN NODE
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
    static constexpr int    INPUT_ROW_THRESHOLD      = 8;
    static constexpr int    BUFFER_CAPACITY          = 8;
    static constexpr int    SLOT_UNSET               = -1;
    static constexpr int    ROW_UNSET                = -1;
    static constexpr double SCALE_TIMEOUT_SEC        = 30.0;
    static constexpr double FILLING_DURATION_SEC     = 90.0;
    static constexpr double MOTION_WATCHDOG_SEC      = 120.0;
    static constexpr int    TOTAL_ROWS               = 5;   // rows per tray
    static constexpr int    TOTAL_OUTPUT_SLOTS       = 10;  // slots per output tray

    // ========================================================================
    // SUBSCRIPTIONS
    // ========================================================================
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  vision_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr vision_row_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   vision_empty_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  vision_slot_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr vision_slot_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr vision_output_snapshot_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   vision_output_present_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   ignore_scale_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   feed_chamber_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   fill_done_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   scale_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   start_button_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   new_tray_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   input_trays_empty_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   cartridge_drain_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  selected_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  command_row_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  selected_slot_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  command_slot_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goto_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   new_trayoutput_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr motion_hb_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr vision_hb_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr cartridge_hb_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   motion_busy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   soft_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   stop_button_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   pause_button_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   resume_button_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  set_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   cartridge_homing_done_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensors_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   cartridge_pos1_busy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   cartridge_pos2_busy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   cartridge_busy_sub_;     // aggregate status for startup/resume coordination
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   output_tray_full_sub_;   // /vision/output_tray/full — AI mode
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   cam0_view_clear_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   cam1_view_clear_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   output_zone_clear_sub_;

    rclcpp::Time last_motion_hb_;
    rclcpp::Time last_vision_hb_;
    std::mutex heartbeat_mutex_;
    std::atomic<bool> motion_hb_seen_{false};
    std::atomic<bool> vision_hb_seen_{false};
    std::atomic<int64_t> motion_hb_last_steady_ns_{0};
    std::atomic<int64_t> vision_hb_last_steady_ns_{0};
    std::atomic<int64_t> cartridge_hb_last_steady_ns_{0};
    std::atomic<int64_t> cartridge_sensors_last_steady_ns_{0};
    std::atomic<int64_t> cartridge_drain_last_steady_ns_{0};
    std::atomic<bool> output_tray_present_{false};
    std::atomic<int64_t> last_output_slot_rx_ns_{0};
    std::atomic<int64_t> last_output_status_rx_ns_{0};
    std::atomic<int64_t> last_output_present_rx_ns_{0};
    std::atomic<bool> motion_busy_{false};
    std::atomic<bool> cartridge_busy_{false};
    std::atomic<bool> cartridge_pos1_busy_{true};
    std::atomic<bool> cartridge_pos2_busy_{true};  // STATE 3/4 OutX/OutY đang chạy — block PLACE_TO_OUTPUT
    std::atomic<bool> cam0_view_clear_{false};
    std::atomic<int64_t> cam0_view_last_steady_ns_{0};
    std::atomic<int64_t> input_decision_accept_after_steady_ns_{0};
    std::atomic<int64_t> input_decision_last_rx_steady_ns_{0};
    std::atomic<bool> cam1_view_clear_{false};
    std::atomic<int64_t> cam1_view_last_steady_ns_{0};
    std::atomic<bool> mechanical_output_zone_clear_{false};
    std::atomic<int64_t> mechanical_output_zone_last_steady_ns_{0};

    // ========================================================================
    // PUBLISHERS
    // ========================================================================
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  selected_slot_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  selected_row_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   stop_button_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   picker_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_uptime_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  tray_count_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   done_input_tray_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   input_scan_allowed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   output_scan_allowed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   new_tray_loaded_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   new_trayoutput_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   done_output_tray_pub_;

    // ========================================================================
    // ACTION CLIENT
    // ========================================================================
    using ExecuteMotion         = robot_control_interfaces::action::ExecuteMotion;
    using GoalHandleExecuteMotion = rclcpp_action::ClientGoalHandle<ExecuteMotion>;
    rclcpp_action::Client<ExecuteMotion>::SharedPtr motion_action_client_;

    // ========================================================================
    // SERVICES (exposed)
    // ========================================================================
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_system_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_system_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_state_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_system_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_manual_mode_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_ai_mode_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr soft_stop_service_;

    // ========================================================================
    // CLIENTS (dobot driver)
    // ========================================================================
    rclcpp::Client<EnableRobot>::SharedPtr   enable_client_;
    rclcpp::Client<ClearError>::SharedPtr    clear_error_client_;
    rclcpp::Client<DO>::SharedPtr            do_client_;
    rclcpp::Client<RobotMode>::SharedPtr     robot_mode_client_;
    rclcpp::Client<GetErrorID>::SharedPtr    error_client_;
    rclcpp::Client<ResetRobot>::SharedPtr    reset_robot_client_;
    rclcpp::Client<Pause>::SharedPtr         pause_client_;
    rclcpp::Client<Continues>::SharedPtr     continue_client_;
    rclcpp::Client<DisableRobot>::SharedPtr  disable_robot_client_;
    rclcpp::Client<EmergencyStop>::SharedPtr emergency_stop_client_;
    rclcpp::Client<StopScript>::SharedPtr    stop_script_client_;

    // ========================================================================
    // THREAD SYNC
    // ========================================================================
    std::condition_variable  state_cv_;
    std::mutex               state_cv_mutex_;
    std::atomic<bool>        state_changed_{false};
    rclcpp::CallbackGroup::SharedPtr callback_group_reentrant_;

    // ========================================================================
    // STATE MACHINE
    // ========================================================================
    SystemState       current_state_;
    SystemState       input_check_next_state_{SystemState::WAIT_FILLING};
    std::thread       state_machine_thread_;
    std::atomic<bool> state_machine_running_{true};
    std::recursive_mutex state_mutex_;
    std::mutex           pause_mutex_;  // serialize service + topic duplicates

    bool is_first_batch_{true};

    // ========================================================================
    // SYSTEM FLAGS
    // ========================================================================
    std::atomic<bool> system_started_{false};
    std::atomic<bool> system_running_{false};

    // --- Tray flags ---
    // new_tray_loaded_       : set true by /cartridge_providesystem/new_tray_loaded
    //                          reset to false when row5 picked (pub done_tray_input)
    // new_trayoutput_loaded_ : set true by /cartridge_providesystem/new_trayoutput_loaded
    //                          reset to false when done_tray_output published
    // waiting_for_new_input_ : true  = đã pub done_tray_input, đang chờ new_tray_loaded
    //                          false = tray input sẵn sàng
    std::atomic<bool> new_tray_loaded_{false};
    std::atomic<bool> input_trays_empty_{true}; // /cartridge/input_trays_empty mirror
    std::atomic<bool> input_belt_empty_from_sensors_{true}; // Direct S1/S2/S3 parse from /providesystem/sensors_state
    std::atomic<bool> cartridge_input_sensors_seen_{false};
    std::atomic<bool> cartridge_drain_confirmed_{false}; // Drain accepted only when S1/S2/S3 are all OFF
    // waiting_for_new_output_: true  = đã pub done_tray_output, đang chờ new_trayoutput_loaded
    //                          false = tray output sẵn sàng
    std::atomic<bool> new_trayoutput_loaded_{false};
    std::atomic<bool> waiting_for_new_input_{true};   // start = chờ tray đầu tiên
    std::atomic<bool> waiting_for_new_output_{false};
    rclcpp::Time wait_tray_start_time_;

    std::atomic<bool> feed_chamber_signal_{false};
    // feed_chamber wait timeout in LOAD_CHAMBER_FROM_BUFFER → if timeout, skip
    // BUFFER→CHAMBER and drain SCALE, then enter WAIT_RESUME_CHOICE after PLACE.
    // May Fill tu xu ly loi cua no va tu ngung phat /revpi/feed_chamber khi co
    // van de; he thong nay chi chay THEO topic cua Fill. Khi vang tin hieu robot
    // dung yen nhu dang PAUSE — khong bao loi, khong dong cham gi toi SCALE.
    //
    // Sau 10 phut moi hoi operator, vi luc do nhieu kha nang su co da duoc khac
    // phuc nhung robot van dang cho. Popup de ho chon: cho tiep, hay bo qua ma
    // sang xu ly can (khi ho da tu xu ly chamber).
    //
    // Truoc day 150s VA XA SCALE truoc khi hoi — hanh dong pha huy do mot su
    // cham tre binh thuong cua may Fill gay ra. Dat 0 de cho vo han khong hoi.
    static constexpr double LOAD_BUFFER_FEED_TIMEOUT_S = 600.0;
    static constexpr double SCALE_TOPIC_TIMEOUT_S      = 150.0;  // PROCESSING_SCALE: no loadcell msg → WAIT_SCALE_CHOICE
    rclcpp::Time feed_chamber_wait_start_;          // set on first wait, reset on pass/exit
    bool feed_chamber_wait_active_{false};          // true while measuring elapsed
    std::atomic<bool> skipped_buffer_load_{false};  // set on timeout, consumed in WAIT_RESUME_CHOICE
    // WAIT_SCALE_CHOICE: can da hong nen khong the chay tiep. Operator chi con
    // chon dat cartridge dang tren can vao OUTPUT hay FAIL, xong la roi han
    // process. Co nay danh dau "dat xong thi dung ca he thong", de chuyen dong
    // dat van chay tron ven roi moi dung — khac han bam STOP luc dang dat.
    std::atomic<bool> exit_after_scale_place_{false};
    std::atomic<bool> s7_at_robot_{false};   // Cartridge S7 — khay đang ở vị trí Robot (parse từ /providesystem/sensors_state)
    std::atomic<bool> fill_done_{false};
    // Buffer needs another valid input row.  Chamber work always has priority:
    // fill_done -> CHAMBER_SCALE, feed_chamber -> BUFFER_CHAMBER; refill runs
    // only after those commands have been handled or when neither is pending.
    std::atomic<bool> refill_pending_{false};
    std::atomic<bool> scale_result_received_{false};
    std::atomic<bool> scale_result_pass_{false};
    std::atomic<bool> system_enabled_{true};
    std::atomic<bool> emergency_stop_{false};
    std::atomic<bool> manual_mode_{true};   // Default = MANUAL (toàn hệ thống) — sync với Python cartridge node
    std::atomic<bool> system_paused_{false};    // PAUSE: giữ state machine + Dobot motion tại chỗ
    std::atomic<bool> dobot_motion_paused_{false}; // True only after native Pause() succeeds/mode=10
    std::atomic<bool> use_ai_for_control_{false};
    std::atomic<bool> stored_scale_result_{false};
    std::atomic<bool> stop_after_single_motion_{false};
    bool              operator_explicitly_set_row_{false}; // Tracks explicit human overrides

    // ========================================================================
    // ROW TRACKING
    // ========================================================================
    // current_auto_row_ : row sẽ pick KẾ TIẾP vào buffer/chamber
    // Khi pick row 5 → pub done_tray_input → reset new_tray_loaded_ = false
    //                                       → waiting_for_new_input_ = true
    // Khi nhận new_tray_loaded → current_auto_row_ = 1, waiting_for_new_input_ = false
    int current_auto_row_{ROW_UNSET};
    int motion_fail_count_{0};

    std::vector<bool> row_full_;
    // Written by the ROS vision callback and read by the state-machine thread.
    // Atomic also makes the confirmed EMPTY decision an explicit cross-thread
    // event instead of inferring it from a temporarily all-false row cache.
    std::atomic<bool> input_tray_empty_{false};
    // Sau new_tray_loaded phai thay empty=false cua CHINH khay moi truoc khi
    // chap nhan canh true. Ngan true con sot tu khay cu kich done sau 0.1s.
    std::atomic<bool> vision_empty_armed_{false};
    std::atomic<bool> input_scan_allowed_{true};
    std::atomic<bool> input_scan_recovery_required_{false};
    int  selected_input_row_{ROW_UNSET};
    std::mutex row_selection_mutex_;

    // ========================================================================
    // OUTPUT SLOT TRACKING
    // ========================================================================
    int  current_auto_slot_{1};
    int  current_fail_slot_{1};
    int  selected_output_slot_{SLOT_UNSET};
    std::vector<bool> output_slot_occupied_;
    std::mutex output_slot_selection_mutex_;
    std::atomic<bool> output_scan_allowed_{true};
    std::atomic<bool> output_scan_recovery_required_{false};
    // A cam1 decision is accepted only as one coherent, frame-stamped sample.
    // These fields are protected by output_slot_selection_mutex_.
    bool output_snapshot_valid_{false};
    bool output_snapshot_present_{false};
    int output_snapshot_slot_{SLOT_UNSET};
    std::array<bool, TOTAL_OUTPUT_SLOTS> output_snapshot_occupied_{};
    int64_t output_snapshot_frame_ns_{0};
    int64_t output_snapshot_rx_ns_{0};
    int64_t output_snapshot_accept_after_ns_{0};
    // Normal AI flow returns HOME after AI_INPUT_TRAY_BUFFER and scans both
    // cameras there while scale processing continues. A dedicated HOME action
    // is only a fallback for drain/restart/no-snapshot paths.
    std::atomic<bool> output_prescan_home_done_{false};
    std::atomic<bool> output_prescan_zone_verified_{false};
    // At the PLACE_TO_OUTPUT edge, invalidate only Robot Logic's aggregate
    // cache (Vision remains warm). The next cam1 frame is then the actual
    // event-time snapshot; no additional HOME/cooldown is introduced.
    std::atomic<bool> output_place_snapshot_requested_{false};
    std::atomic<bool> output_tray_change_home_pending_{false};
    std::atomic<bool> output_tray_change_home_motion_done_{false};
    std::atomic<int64_t> output_tray_change_home_wait_since_ns_{0};
    // SCALE_OUTPUT already returns to index 0. Before leaving PLACE_TO_OUTPUT
    // for WAIT_FILLING, use that dwell as the second cam0+cam1 checkpoint:
    // refresh input rows and confirm whether the updated output tray is full.
    // No additional HOME action.
    std::atomic<bool> post_place_output_check_pending_{false};
    std::atomic<bool> post_place_output_scan_started_{false};

    // ========================================================================
    // ROBOT STATE
    // ========================================================================
    bool buffer_is_empty_{true};
    bool chamber_is_empty_{true};
    bool chamber_has_cartridge_{false};
    bool scale_has_cartridge_{false};
    bool cartridge_is_homed_{false};

    // ========================================================================
    // ASYNC MOTION STATE
    // ========================================================================
    std::atomic<bool> motion_in_progress_{false};
    std::atomic<bool> motion_result_{false};
    std::atomic<bool> last_motion_interlock_rejected_{false};
    std::atomic<bool> startup_home_pending_{false};
    std::atomic<bool> startup_home_motion_done_{false};
    std::atomic<int64_t> startup_home_wait_since_ns_{0};
    std::atomic<bool> startup_home_required_{false};
    std::string       motion_current_cmd_;
    std::mutex        motion_cmd_mutex_;
    std::atomic<int>  async_slot_{0};
    rclcpp::Time      motion_started_at_{0, 0, RCL_ROS_TIME};

    // Camera state

    // ========================================================================
    // SCALE QUEUE
    // ========================================================================
    struct PendingScaleResult {
        bool pass;
        rclcpp::Time timestamp;
        int cartridge_id;
    };
    std::deque<PendingScaleResult> pending_scale_results_;
    std::mutex                     scale_result_mutex_;
    std::atomic<int>               cartridge_counter_{0};
    rclcpp::Time                   scale_wait_start_;

    bool processNextScaleResult(bool& result_pass, int& cartridge_id);
    bool hasPendingScaleResults();

    // ========================================================================
    // TIMING / UPTIME
    // ========================================================================
    rclcpp::Time system_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::TimerBase::SharedPtr uptime_timer_;
    rclcpp::TimerBase::SharedPtr startup_release_timer_;
    std::atomic<int> tray_count_{0};

    // ========================================================================
    // SIMULATION
    // ========================================================================
    bool simulate_scale_{false};
    bool force_pass_{false};
    bool ignore_scale_{false};

    // ========================================================================
    // INIT
    // ========================================================================
    void initServiceClients();
    void initSubscriptions();
    void initPublishers();
    void initServices();

    // ========================================================================
    // CALLBACKS
    // ========================================================================
    void feedChamberCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void fillDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void scaleResultCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void startButtonCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void newTrayCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void newTrayOutputCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void selectedRowCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void selectedSlotCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void gotoStateCallback(const std_msgs::msg::String::SharedPtr msg);
    void commandRowCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void commandSlotCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void setModeCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void cartridgeHomingDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void publishCameraStatus(const std::string& status);

    // ========================================================================
    // SERVICE CALLBACKS
    // ========================================================================
    void enableSystemCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void startSystemCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void resetStateCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void pauseSystemCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    // ========================================================================
    // STATE MACHINE
    // ========================================================================
    void stateMachineLoop();
    void handleCurrentState();
    void notifyStateChange();

    void stateIdle();
    void stateInitCheck();
    void stateInitLoadChamberDirect();
    void stateInitRefillBuffer();
    void stateAiCheckInputTray();
    void stateWaitFilling();
    void stateTakeChamberToScale();
    void stateLoadChamberFromBuffer();
    void stateWaitResumeChoice();
    void stateWaitScaleChoice();
    void stateRefillBuffer();
    void stateProcessingScale();
    void stateErrorScaleTimeout();
    void statePlaceToOutput();
    void statePlaceToFail();
    void stateLastBatchWait();
    void stateErrorScale();
    void stateErrorMotionLost();

    // ========================================================================
    // MOTION
    // ========================================================================
    bool setDigitalOutput(int index, bool status);
    bool checkMotionAlive(double timeout_sec = 2.0);
    bool sendMotionAction(const std::string& command, int slot = 0, int timeout_sec = 30);
    void sendMotionActionAsync(const std::string& command, int slot = 0);
    bool sendMotionCommand(const std::string& command, int timeout_sec = 60);

    // ========================================================================
    // HELPERS
    // ========================================================================
    void logState(const std::string& message);
    void transitionTo(SystemState new_state);
    void setInputScanAllowed(bool allowed);
    void setOutputScanAllowed(bool allowed);
    void invalidateOutputSnapshot();
    bool claimFreshOutputSnapshot(int& slot);
    bool claimFreshFullOutputSnapshot();
    void requestOutputSnapshotNow();
    bool cam0DecisionWindowReady() const;
    bool cam0CheckpointFrameReceived() const;
    bool cam1ViewClearFresh() const;
    bool mechanicalOutputZoneClearFresh() const;
    bool visionHeartbeatFresh();
    bool cartridgeHeartbeatFresh() const;
    bool cartridgeSafetyFreshAndClear(bool require_pos2 = true) const;
    bool cartridgeDrainEvidenceFresh() const;
    bool servicePendingOutputTrayChangeHome();
    bool servicePendingStartupHome();
    void requestStartupHome(const char* source);
    bool consumeTemporaryMotionInterlockReject(const char* context, bool reopen_input_scan);
    void routeAfterOutputPlacement();
    static int64_t steadyNowNs();
    void softStopToManual(const std::string& source);
    bool exitAfterScalePlaceRequested(const std::string& source);
    void forceScalePass(const std::string& source);
    bool checkConnection();
    bool validateRobotState();
    std::string stateToString(SystemState state);
    void publishSystemStatus(const std::string& status);
    void publishError(const std::string& error);

    // Thread-safe motion cmd
    void setMotionCmd(const std::string& cmd) {
        std::lock_guard<std::mutex> lk(motion_cmd_mutex_);
        motion_current_cmd_ = cmd;
    }
    std::string getMotionCmd() {
        std::lock_guard<std::mutex> lk(motion_cmd_mutex_);
        return motion_current_cmd_;
    }
    void clearMotionCmd() {
        std::lock_guard<std::mutex> lk(motion_cmd_mutex_);
        motion_current_cmd_.clear();
    }

    // Helper: pick next row in AUTO mode
    // Returns row number (1-5). If row==5 → publishes done_tray_input + sets waiting flags.
    // Returns -1 if waiting for tray.
    int getNextAutoRow();

    // Helper: advance row AFTER successful pick
    void advanceAutoRow();

    // Helper: publish done_tray_output + set waiting flag
    void publishDoneTrayOutput();
    bool requestOutputTrayChangeForDrain(const char* reason);

    // Helper: check if BOTH trays are ready (used in IDLE to restart)
    bool bothTraysReady() const {
        return !waiting_for_new_input_.load() && !waiting_for_new_output_.load();
    }

    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr callService(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request,
        const std::string& service_name);
};

// ============================================================================
// CONSTRUCTOR
// ============================================================================

RobotLogicNode::RobotLogicNode()
    : Node("robot_logic_nova5"),
      current_state_(SystemState::IDLE),
      is_first_batch_(true),
      input_tray_empty_(false),
      selected_input_row_(ROW_UNSET),
      buffer_is_empty_(true),
      selected_output_slot_(SLOT_UNSET),
      chamber_is_empty_(true),
      chamber_has_cartridge_(false),
      scale_has_cartridge_(false),
      simulate_scale_(false),
      force_pass_(false),
      ignore_scale_(false)
{
    RCLCPP_INFO(this->get_logger(), "=== Robot Logic Node Starting ===");

    row_full_.assign(TOTAL_ROWS, false);
    // Fail-safe cho toi khi vision gui status dau tien.
    output_slot_occupied_.assign(TOTAL_OUTPUT_SLOTS, true);
    output_snapshot_occupied_.fill(true);

    callback_group_reentrant_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    initServiceClients();
    initSubscriptions();
    initPublishers();
    initServices();

    // Mốc dự phòng phải hợp lệ ngay từ lúc node khởi động. Một số lệnh chuyển
    // trạng thái có thể bật system_running_ mà không đi qua Start callback;
    // nếu giữ giá trị ROS time = 0, uptime sẽ bị tính từ Unix epoch (~495k giờ).
    system_start_time_ = this->now();

    state_machine_thread_ = std::thread(&RobotLogicNode::stateMachineLoop, this);

    // Startup safe state: NHẢ cả gripper + picker (đảm bảo không kẹp khay nếu
    // node restart khi đang giữ). Delay 2s để Python cartridge node kịp lên + subscriber connect.
    startup_release_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
            auto msg = std_msgs::msg::Bool();
            msg.data = false;  // NHẢ
            if (gripper_cmd_pub_) gripper_cmd_pub_->publish(msg);
            if (picker_cmd_pub_)  picker_cmd_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "[STARTUP] Released gripper + picker (NHẢ)");
            startup_release_timer_->cancel();
        });


    // Uptime timer
    uptime_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            if (system_enabled_ && system_running_ && !manual_mode_) {
                const auto now = this->now();
                // Tự phục hồi nếu clock ROS vừa reset hoặc một code path cũ bật
                // running trước khi đặt start time.
                if (system_start_time_.nanoseconds() <= 0 || system_start_time_ > now)
                    system_start_time_ = now;
                const int64_t elapsed = std::max<int64_t>(
                    0, static_cast<int64_t>((now - system_start_time_).seconds()));
                int64_t h = elapsed / 3600;
                int64_t m = (elapsed % 3600) / 60;
                int64_t s = elapsed % 60;
                char buf[32];
                snprintf(buf, sizeof(buf), "%02lld:%02lld:%02lld",
                         static_cast<long long>(h),
                         static_cast<long long>(m),
                         static_cast<long long>(s));
                auto msg = std_msgs::msg::String();
                msg.data = buf;
                system_uptime_pub_->publish(msg);
            } else {
                auto msg = std_msgs::msg::String();
                msg.data = "00:00:00";
                system_uptime_pub_->publish(msg);
            }
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
    if (state_machine_thread_.joinable()) {
        state_machine_thread_.join();
    }
}

// ============================================================================
// INIT
// ============================================================================

void RobotLogicNode::initServiceClients()
{
    motion_action_client_ = rclcpp_action::create_client<ExecuteMotion>(
        this, "/robot/execute_motion");
    // ROS 2 Humble create_client() takes an rmw_qos_profile_t.
    const auto qos = rmw_qos_profile_services_default;

    enable_client_        = create_client<EnableRobot>  ("/nova5/dobot_bringup/EnableRobot",   qos, callback_group_reentrant_);
    clear_error_client_   = create_client<ClearError>   ("/nova5/dobot_bringup/ClearError",    qos, callback_group_reentrant_);
    do_client_            = create_client<DO>            ("/nova5/dobot_bringup/DO",            qos, callback_group_reentrant_);
    robot_mode_client_    = create_client<RobotMode>    ("/nova5/dobot_bringup/RobotMode",     qos, callback_group_reentrant_);
    error_client_         = create_client<GetErrorID>   ("/nova5/dobot_bringup/GetErrorID",    qos, callback_group_reentrant_);
    reset_robot_client_   = create_client<ResetRobot>   ("/nova5/dobot_bringup/ResetRobot",    qos, callback_group_reentrant_);
    pause_client_         = create_client<Pause>        ("/nova5/dobot_bringup/Pause",         qos, callback_group_reentrant_);
    continue_client_      = create_client<Continues>    ("/nova5/dobot_bringup/Continue",      qos, callback_group_reentrant_);
    disable_robot_client_ = create_client<DisableRobot> ("/nova5/dobot_bringup/DisableRobot",  qos, callback_group_reentrant_);
    emergency_stop_client_= create_client<EmergencyStop>("/nova5/dobot_bringup/EmergencyStop", qos, callback_group_reentrant_);
    stop_script_client_   = create_client<StopScript>   ("/nova5/dobot_bringup/StopScript",    qos, callback_group_reentrant_);
}

void RobotLogicNode::initSubscriptions()
{
    // Vision
    vision_row_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/vision/input_tray/selected_row", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            if (use_ai_for_control_.load() && !cam0DecisionWindowReady()) {
                selected_input_row_ = ROW_UNSET;
                return;
            }
            if (msg->data >= 1 && msg->data <= TOTAL_ROWS) {
                selected_input_row_ = msg->data;
                row_full_[msg->data - 1] = true;
            } else {
                // Vision gate (vd InX chua o -60mm) publish -1 de huy lua
                // chon cu. Khong duoc giu row stale va gap khi khay dang di.
                selected_input_row_ = ROW_UNSET;
            }
        });

    // Nguon su that cho 5 den READY. Truoc day robot chi set true cho row
    // duoc selected va khong bao gio nhan false, nen sau khi R1 da pick (GUI
    // tat dung) cache robot van true va AI gap lai R1.
    vision_row_status_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/vision/input_tray/row_status", 10,
        [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            if (use_ai_for_control_.load() && !cam0DecisionWindowReady()) {
                std::fill(row_full_.begin(), row_full_.end(), false);
                return;
            }
            for (size_t i = 0; i < row_full_.size(); ++i) {
                row_full_[i] =
                    (i < msg->data.size() && msg->data[i] == 1);
            }
            input_decision_last_rx_steady_ns_ = steadyNowNs();
        });

    vision_empty_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/vision/input_tray/empty", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (use_ai_for_control_.load() && !cam0DecisionWindowReady()) return;
            if (use_ai_for_control_.load())
                input_decision_last_rx_steady_ns_ = steadyNowNs();
            // Detection bi che trong luc robot dang gap khong phai bang chung
            // khay het. Vision cung freeze luc motion; guard nay la lop an toan
            // cuoi de khong bao done_tray_input do message den tre.
            if (msg->data && motion_busy_.load()) return;
            if (!msg->data) {
                input_tray_empty_ = false;
                vision_empty_armed_ = true;
                return;
            }

            const bool was_empty = input_tray_empty_.exchange(msg->data);

            // AI mode: rising edge of "all rows empty" → publish done_tray_input
            // This mirrors advanceAutoRow() last-row logic, gated by vision instead of row counter.
            // The cartridge Python node then checks S7 ON before starting S2A (take back empty tray).
            if (use_ai_for_control_ && vision_empty_armed_.load()
                && msg->data && !was_empty
                && !waiting_for_new_input_.load() && system_running_.load())
            {
                setInputScanAllowed(false);
                RCLCPP_WARN(get_logger(),
                    "[AI] Vision: all rows empty → publishing done_tray_input");
                auto done_msg = std_msgs::msg::Bool();
                done_msg.data = true;
                done_input_tray_pub_->publish(done_msg);
                new_tray_loaded_ = false;
                new_tray_loaded_pub_->publish(std_msgs::msg::Bool());
                waiting_for_new_input_ = true;
                vision_empty_armed_ = false;
                cartridge_drain_confirmed_ = false;
                wait_tray_start_time_  = this->now();
                notifyStateChange();
            }
        });

    vision_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/vision/output_tray/selected_slot", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            if (!output_scan_allowed_.load()) {
                selected_output_slot_ = SLOT_UNSET;
                return;
            }
            selected_output_slot_ = msg->data;
            last_output_slot_rx_ns_ = this->now().nanoseconds();
        });

    // Cung nguon voi 10 den O tren GUI: 0=empty/den sang/co the dat,
    // 1=occupied/den tat/khong duoc dat lai.
    vision_slot_status_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/vision/output_tray/slot_status", 10,
        [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            if (!output_scan_allowed_.load()) {
                std::fill(output_slot_occupied_.begin(), output_slot_occupied_.end(), true);
                selected_output_slot_ = SLOT_UNSET;
                return;
            }
            for (size_t i = 0; i < output_slot_occupied_.size(); ++i) {
                output_slot_occupied_[i] =
                    !(i < msg->data.size() && msg->data[i] == 0);
            }
            if (selected_output_slot_ >= 1 &&
                selected_output_slot_ <= TOTAL_OUTPUT_SLOTS &&
                output_slot_occupied_[selected_output_slot_ - 1]) {
                selected_output_slot_ = SLOT_UNSET;
            }
            last_output_status_rx_ns_ = this->now().nanoseconds();
        });

    vision_output_present_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/vision/output_tray/present", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            output_tray_present_ = msg->data;
            last_output_present_rx_ns_ = this->now().nanoseconds();
            if (!msg->data) {
                std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
                selected_output_slot_ = SLOT_UNSET;
                std::fill(output_slot_occupied_.begin(), output_slot_occupied_.end(), true);
            }
        });

    vision_output_snapshot_sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(
        "/vision/output_tray/snapshot", 10,
        [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
            constexpr size_t SNAPSHOT_FIELDS = 5 + TOTAL_OUTPUT_SLOTS;
            const int64_t now_ns = this->now().nanoseconds();
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);

            auto reject = [this]() {
                output_snapshot_valid_ = false;
                output_snapshot_present_ = false;
                output_snapshot_slot_ = SLOT_UNSET;
                output_snapshot_occupied_.fill(true);
                output_snapshot_frame_ns_ = 0;
                output_snapshot_rx_ns_ = 0;
            };

            if (!output_scan_allowed_.load() || !cam1ViewClearFresh() ||
                msg->data.size() != SNAPSHOT_FIELDS || msg->data[0] != 1) {
                reject();
                return;
            }

            const int64_t sec = msg->data[1];
            const int64_t nanosec = msg->data[2];
            if (sec < 0 || nanosec < 0 || nanosec >= 1000000000LL ||
                sec > (std::numeric_limits<int64_t>::max() - nanosec) / 1000000000LL) {
                reject();
                return;
            }
            const int64_t frame_ns = sec * 1000000000LL + nanosec;
            const int selected = static_cast<int>(msg->data[4]);
            const bool present = msg->data[3] == 1;
            bool schema_ok = msg->data[3] == 0 || msg->data[3] == 1;
            std::array<bool, TOTAL_OUTPUT_SLOTS> occupied{};
            for (size_t i = 0; i < TOTAL_OUTPUT_SLOTS; ++i) {
                const int64_t value = msg->data[5 + i];
                schema_ok = schema_ok && (value == 0 || value == 1);
                occupied[i] = value != 0;
            }
            const bool selected_ok = selected == SLOT_UNSET ||
                (selected >= 1 && selected <= TOTAL_OUTPUT_SLOTS &&
                 !occupied[static_cast<size_t>(selected - 1)]);
            const bool frame_fresh = frame_ns > 0 && now_ns >= frame_ns &&
                now_ns - frame_ns <= 2000000000LL;
            // Both receipt time and camera capture time must be after the gate
            // reopened.  This rejects YOLO/DDS samples queued while the arm was
            // still moving or while cam1 was occluded.
            const bool clean_epoch = frame_fresh && output_snapshot_accept_after_ns_ > 0 &&
                now_ns >= output_snapshot_accept_after_ns_ &&
                frame_ns >= output_snapshot_accept_after_ns_ &&
                frame_ns <= now_ns;
            if (!schema_ok || !selected_ok || !clean_epoch) {
                if (frame_ns <= 0) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                        "[SAFETY][CAM1] Reject snapshot: camera frame stamp is zero/invalid");
                }
                reject();
                return;
            }

            output_snapshot_valid_ = true;
            output_snapshot_present_ = present;
            output_snapshot_slot_ = selected;
            output_snapshot_occupied_ = occupied;
            output_snapshot_frame_ns_ = frame_ns;
            output_snapshot_rx_ns_ = now_ns;

            // Mirror the coherent sample into the legacy fields used by the
            // GUI; motion admission below reads only the snapshot fields.
            output_tray_present_ = present;
            selected_output_slot_ = selected;
            for (size_t i = 0; i < TOTAL_OUTPUT_SLOTS; ++i)
                output_slot_occupied_[i] = occupied[i];
            notifyStateChange();
        });

    feed_chamber_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/revpi/feed_chamber", 10,
        std::bind(&RobotLogicNode::feedChamberCallback, this, std::placeholders::_1));

    fill_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/revpi/fill_done", 10,
        std::bind(&RobotLogicNode::fillDoneCallback, this, std::placeholders::_1));

    scale_result_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/loadcell/signal_process", 10,
        std::bind(&RobotLogicNode::scaleResultCallback, this, std::placeholders::_1));

    start_button_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/system/start_button", 10,
        std::bind(&RobotLogicNode::startButtonCallback, this, std::placeholders::_1));

    // Topic path keeps web/other HMIs in sync with the service path. The GUI
    // currently sends both; pause_mutex_ makes the duplicate idempotent.
    pause_button_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/system/pause_button", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (!msg->data) return;
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            auto res = std::make_shared<std_srvs::srv::SetBool::Response>();
            req->data = true;
            pauseSystemCallback(req, res);
        });

    resume_button_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/system/resume_button", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (!msg->data) return;
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            auto res = std::make_shared<std_srvs::srv::SetBool::Response>();
            req->data = false;
            pauseSystemCallback(req, res);
        });

    // new_tray_loaded: input tray ready after cartridge system change
    new_tray_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge_providesystem/new_tray_loaded", 10,
        std::bind(&RobotLogicNode::newTrayCallback, this, std::placeholders::_1));

    // cartridge sensors state — binary "S1..S28".
    // S1/S2/S3 confirm feeder belt empty for pipeline drain; S7 verifies tray at robot.
    sensors_state_sub_ = create_subscription<std_msgs::msg::String>(
        "/providesystem/sensors_state", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            cartridge_sensors_last_steady_ns_ = steadyNowNs();
            if (msg->data.size() >= 3) {
                bool s1 = (msg->data[0] == '1');
                bool s2 = (msg->data[1] == '1');
                bool s3 = (msg->data[2] == '1');
                bool empty = !(s1 || s2 || s3);
                bool prev_empty = input_belt_empty_from_sensors_.load();
                input_belt_empty_from_sensors_ = empty;
                cartridge_input_sensors_seen_ = true;
                // Drain confirmation is level-triggered, not just gated on arrival of
                // /cartridge/drain. A tray back on S1/S2/S3 invalidates any earlier
                // confirmation — without this the latched flag survives until
                // WAIT_FILLING/REFILL_BUFFER/PLACE_* and ends the batch on stale state.
                bool drain_revoked = false;
                if (!empty && cartridge_drain_confirmed_.exchange(false)) {
                    drain_revoked = true;
                    RCLCPP_WARN(get_logger(),
                        "[DRAIN] Revoked drain confirmation: S1/S2/S3 sees tray again");
                }
                if (prev_empty != empty) {
                    RCLCPP_WARN(get_logger(), "[DRAIN] S1/S2/S3 belt empty from sensors: %s",
                        empty ? "TRUE" : "FALSE");
                }
                if (prev_empty != empty || drain_revoked) {
                    notifyStateChange();
                }
            }
            if (msg->data.size() >= 7) {
                bool prev = s7_at_robot_.load();
                bool curr = (msg->data[6] == '1');
                if (prev != curr) {
                    s7_at_robot_ = curr;
                    notifyStateChange();
                }
            }
        });

    ignore_scale_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/ignore_scale", rclcpp::QoS(10).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            ignore_scale_ = msg->data;
            RCLCPP_INFO(get_logger(), "[SCALE] Ignore Scale mode toggled: %s", ignore_scale_ ? "ON" : "OFF");
            if (ignore_scale_) {
                forceScalePass("/robot/ignore_scale");
            }
        });

    input_trays_empty_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge/input_trays_empty", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            input_trays_empty_ = msg->data;
        });

    cartridge_drain_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge/drain", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            cartridge_drain_last_steady_ns_ = steadyNowNs();
            bool prev = cartridge_drain_confirmed_.load();
            bool accept = false;
            if (msg->data) {
                bool sensor_seen = cartridge_input_sensors_seen_.load();
                bool s123_empty = input_belt_empty_from_sensors_.load();
                accept = sensor_seen && s123_empty;
                if (!sensor_seen) {
                    RCLCPP_WARN(get_logger(),
                        "[DRAIN] Ignored cartridge drain=true: no S1/S2/S3 sensor snapshot yet");
                } else if (!s123_empty) {
                    RCLCPP_WARN(get_logger(),
                        "[DRAIN] Rejected cartridge drain=true: S1/S2/S3 still sees tray");
                }
            }
            cartridge_drain_confirmed_ = accept;
            if (prev != accept || msg->data != accept) {
                RCLCPP_WARN(get_logger(), "[DRAIN] Cartridge drain confirmation: %s (topic=%s, sensors_empty=%s)",
                    accept ? "TRUE" : "FALSE",
                    msg->data ? "TRUE" : "FALSE",
                    input_belt_empty_from_sensors_.load() ? "TRUE" : "FALSE");
                notifyStateChange();
            }
        });

    // new_trayoutput_loaded: output tray ready after cartridge system change
    new_trayoutput_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge_providesystem/new_trayoutput_loaded",
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&RobotLogicNode::newTrayOutputCallback, this, std::placeholders::_1));

    selected_row_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/camera/ai/selected_row", 10,
        std::bind(&RobotLogicNode::selectedRowCallback, this, std::placeholders::_1));

    selected_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/camera/ai/selected_slot", 10,
        std::bind(&RobotLogicNode::selectedSlotCallback, this, std::placeholders::_1));

    goto_state_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot/goto_state", 10,
        std::bind(&RobotLogicNode::gotoStateCallback, this, std::placeholders::_1));

    cartridge_homing_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge/homing_done", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&RobotLogicNode::cartridgeHomingDoneCallback, this, std::placeholders::_1));

    command_row_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/robot/command_row", 10,
        std::bind(&RobotLogicNode::commandRowCallback, this, std::placeholders::_1));

    command_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/robot/command_slot", 10,
        std::bind(&RobotLogicNode::commandSlotCallback, this, std::placeholders::_1));

    // Heartbeat & busy
    motion_hb_sub_ = create_subscription<std_msgs::msg::Header>(
        "/robot/motion_heartbeat", 10,
        [this](const std_msgs::msg::Header::SharedPtr msg) {
            {
                std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                last_motion_hb_ = msg->stamp;
            }
            motion_hb_seen_ = true;
            motion_hb_last_steady_ns_ = steadyNowNs();
        });

    vision_hb_sub_ = create_subscription<std_msgs::msg::Header>(
        "/vision/heartbeat", 10,
        [this](const std_msgs::msg::Header::SharedPtr msg) {
            const bool was_seen = vision_hb_seen_.exchange(true);
            {
                std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                last_vision_hb_ = msg->stamp;
            }
            vision_hb_last_steady_ns_ = steadyNowNs();
            if (!was_seen && use_ai_for_control_) {
                RCLCPP_INFO(get_logger(),
                    "[SAFETY][CAM1] Vision heartbeat connected — waiting for a fresh slot snapshot");
            }
        });

    cartridge_hb_sub_ = create_subscription<std_msgs::msg::Header>(
        "/cartridge/heartbeat", 10,
        [this](const std_msgs::msg::Header::SharedPtr) {
            cartridge_hb_last_steady_ns_ = steadyNowNs();
        });

    cam0_view_clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/cam0_view_clear", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            const int64_t now_ns = steadyNowNs();
            cam0_view_last_steady_ns_ = now_ns;
            const bool was_clear = cam0_view_clear_.exchange(msg->data);
            if (!msg->data) {
                input_decision_accept_after_steady_ns_ = 0;
                input_decision_last_rx_steady_ns_ = 0;
                vision_empty_armed_ = false;
                std::lock_guard<std::mutex> lock(row_selection_mutex_);
                selected_input_row_ = ROW_UNSET;
                std::fill(row_full_.begin(), row_full_.end(), false);
            } else if (!was_clear) {
                // Motion Executor has already required 0.5 s stable joints at
                // HOME/index 0. Add the same clean
                // image-pipeline delay used by Vision before accepting READY/
                // EMPTY messages in Robot Logic.
                input_decision_accept_after_steady_ns_ = now_ns + 1500000000LL;
                input_decision_last_rx_steady_ns_ = 0;
                std::lock_guard<std::mutex> lock(row_selection_mutex_);
                selected_input_row_ = ROW_UNSET;
                std::fill(row_full_.begin(), row_full_.end(), false);
            }
            if (was_clear != msg->data) {
                RCLCPP_INFO(get_logger(),
                    "[SAFETY][CAM0_CHECKPOINT] Robot view %s",
                    msg->data
                        ? "CLEAR — wait 1.5 s for a new HOME frame"
                        : "BLOCKED — cached input rows invalidated");
            }
            notifyStateChange();
        });

    cam1_view_clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/cam1_view_clear", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            cam1_view_last_steady_ns_ = steadyNowNs();
            const bool was_clear = cam1_view_clear_.exchange(msg->data);
            if (!msg->data) {
                invalidateOutputSnapshot();
                const bool was_verified = output_prescan_zone_verified_.exchange(false);
                // Repeated FALSE samples during the 0.5 s settle interval after
                // an already completed HOME action are expected.  Invalidate a
                // checkpoint only on a real CLEAR->BLOCKED edge (or after state
                // logic had already verified it).
                const bool had_home_checkpoint =
                    (was_clear || was_verified) && output_prescan_home_done_.exchange(false);
                // FALSE is expected while OUTPUT_SCAN_HOME is travelling and
                // after a claimed SCALE_OUTPUT starts.  If permission vanished
                // after verification but before claim, treat it as a fault.
                if (was_verified) {
                    RCLCPP_ERROR(get_logger(),
                        "[SAFETY][CAM1] Camera view became blocked after verification — "
                        "snapshot invalidated; a new clear checkpoint is required");
                } else if (had_home_checkpoint) {
                    RCLCPP_WARN(get_logger(),
                        "[SAFETY][CAM1] Camera snapshot checkpoint consumed/invalidated");
                }
            }
            if (was_clear != msg->data) {
                RCLCPP_INFO(get_logger(), "[SAFETY][CAM1] Robot camera view: %s",
                    msg->data ? "CLEAR" : "BLOCKED");
            }
            notifyStateChange();
        });

    output_zone_clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/output_zone_clear", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            mechanical_output_zone_last_steady_ns_ = steadyNowNs();
            mechanical_output_zone_clear_ = msg->data;
            notifyStateChange();
        });

    motion_busy_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/motion_busy", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { motion_busy_ = msg->data; });

    soft_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/system/soft_stop", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) softStopToManual("/system/soft_stop");
        });

    // Phat lai chinh topic cua nut STOP: sau khi dat cartridge tu can xong,
    // node tu bam STOP thay operator de cartridge/VFD/GUI cung reset mot the.
    stop_button_pub_ = create_publisher<std_msgs::msg::Bool>("/system/stop_button", 10);

    stop_button_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/system/stop_button", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) softStopToManual("/system/stop_button");
        });

    // PHẢI gán vào member SharedPtr — create_subscription return SharedPtr; nếu vứt return,
    // sub bị destruct ngay, callback không bao giờ fire. Bug history: interlock cartridge_busy_
    // không hoạt động → motion thread chạy đè cartridge → kẹt khay.
    cartridge_busy_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge/busy", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            cartridge_busy_ = msg->data;
            if (msg->data)
                RCLCPP_WARN(get_logger(), "[INTERLOCK] 🔒 Cartridge BUSY");
            else
                RCLCPP_INFO(get_logger(), "[INTERLOCK] 🔓 Cartridge FREE");
        });

    cartridge_pos1_busy_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge/pos1_busy", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            cartridge_pos1_busy_ = msg->data;
            RCLCPP_INFO(get_logger(), "[MUTEX][POS1] Cartridge State1/2 %s",
                msg->data ? "BUSY" : "FREE");
        });

    // Pos2 busy: STATE 3 (cấp khay thành phẩm) hoặc STATE 4 (thay khay output)
    // đang chạy → cụm OutX/OutY có thể đang di chuyển → block PLACE_TO_OUTPUT.
    cartridge_pos2_busy_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cartridge/pos2_busy", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            const bool was_busy = cartridge_pos2_busy_.exchange(msg->data);
            if (msg->data) {
                RCLCPP_WARN(get_logger(), "[INTERLOCK] 🔒 Cartridge Pos2 BUSY");
                if (use_ai_for_control_) {
                    output_prescan_zone_verified_ = false;
                    setOutputScanAllowed(false);
                }
            } else {
                RCLCPP_INFO(get_logger(), "[INTERLOCK] 🔓 Cartridge Pos2 FREE");
                if (was_busy && use_ai_for_control_ &&
                    !waiting_for_new_output_.load() &&
                    !output_scan_recovery_required_.load()) {
                    const bool robot_still_home = cam1ViewClearFresh();
                    output_prescan_home_done_ = robot_still_home;
                    output_prescan_zone_verified_ = false;
                    setOutputScanAllowed(robot_still_home);
                    if (!robot_still_home) {
                        RCLCPP_WARN(get_logger(),
                            "[SAFETY][CAM1] Pos2 FREE but robot HOME is not verified — "
                            "output scan remains blocked");
                    }
                }
            }
        });

    // AI mode: Camera báo output tray full → set waiting flag.
    // Gán vào member: nếu không, callback chết → AI mode không reset state khi tray full.
    output_tray_full_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/vision/output_tray/full", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data && use_ai_for_control_) {
                if (!visionHeartbeatFresh() || !cam1ViewClearFresh() ||
                    !output_scan_allowed_.load() ||
                    !claimFreshFullOutputSnapshot()) {
                    RCLCPP_ERROR(get_logger(),
                        "[SAFETY][CAM1] Ignore OUTPUT FULL — heartbeat/zone/coherent "
                        "full snapshot missing or stale");
                    return;
                }
                // The claim above already closed/invalidate the admission cache;
                // this publishes the matching latched gate without a TOCTOU gap.
                setOutputScanAllowed(false);
                RCLCPP_WARN(get_logger(),
                    "[VISION_SYNC][CAM1_FULL] 📦 Fresh camera reports OUTPUT FULL — "
                    "Robot Logic locked placement; evacuating robot to HOME before State 4");
                output_tray_change_home_motion_done_ = false;
                output_tray_change_home_wait_since_ns_ = 0;
                output_tray_change_home_pending_ = true;
                notifyStateChange();
            }
        });
}

void RobotLogicNode::initPublishers()
{
    system_status_pub_  = create_publisher<std_msgs::msg::String>("/robot/system_status", 10);
    error_pub_          = create_publisher<std_msgs::msg::String>("/robot/error", 10);
    selected_slot_pub_  = create_publisher<std_msgs::msg::Int32> ("/robot/selected_output_slot", 10);
    selected_row_pub_   = create_publisher<std_msgs::msg::Int32> ("/robot/selected_input_row", 10);
    gripper_cmd_pub_    = create_publisher<std_msgs::msg::Bool>  ("/robot/gripper_cmd", 10);
    picker_cmd_pub_     = create_publisher<std_msgs::msg::Bool>  ("/robot/picker_cmd", 10);
    camera_status_pub_  = create_publisher<std_msgs::msg::String>("/camera/status", 10);
    system_uptime_pub_  = create_publisher<std_msgs::msg::String>("/robot/system_uptime", 10);
    tray_count_pub_     = create_publisher<std_msgs::msg::Int32> ("/robot/tray_count", 10);
    done_input_tray_pub_  = create_publisher<std_msgs::msg::Bool>("/robot/done_tray_input",  10);
    input_scan_allowed_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/vision/input_scan_allowed", rclcpp::QoS(1).reliable().transient_local());
    output_scan_allowed_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/vision/output_scan_allowed", rclcpp::QoS(1).reliable().transient_local());
    new_tray_loaded_pub_  = create_publisher<std_msgs::msg::Bool>("/cartridge_providesystem/new_tray_loaded", rclcpp::QoS(10).reliable());
    done_output_tray_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/done_tray_output", 10);
    new_trayoutput_pub_   = create_publisher<std_msgs::msg::Bool>("/cartridge_providesystem/new_trayoutput_loaded", rclcpp::QoS(10).reliable());

    // Idle/default: camera may inspect the input tray. AI pick motions
    // explicitly close this gate and reopen it when their action completes.
    setInputScanAllowed(true);
    setOutputScanAllowed(true);
}

void RobotLogicNode::initServices()
{
    enable_system_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/enable_system",
        std::bind(&RobotLogicNode::enableSystemCallback, this,
            std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_reentrant_);

    emergency_stop_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/emergency_stop",
        std::bind(&RobotLogicNode::emergencyStopCallback, this,
            std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_reentrant_);

    start_system_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/start_system",
        std::bind(&RobotLogicNode::startSystemCallback, this,
            std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_reentrant_);

    reset_state_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/reset_state",
        std::bind(&RobotLogicNode::resetStateCallback, this,
            std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_reentrant_);

    pause_system_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/pause_system",
        std::bind(&RobotLogicNode::pauseSystemCallback, this,
            std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_reentrant_);

    set_manual_mode_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/set_manual_mode",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
            if (req->data) {
                manual_mode_ = true;
                use_ai_for_control_ = false;
                std::lock_guard<std::mutex> lock(row_selection_mutex_);
                for (size_t i = 0; i < row_full_.size(); ++i) row_full_[i] = true;
                input_tray_empty_ = false;
                selected_input_row_ = ROW_UNSET;
                buffer_is_empty_ = true;
                selected_output_slot_ = 1;
                RCLCPP_INFO(get_logger(), "[MODE] MANUAL (via service)");
            } else {
                const bool leaving_manual = manual_mode_;
                manual_mode_ = false;
                // Roi MANUAL = chu ky moi (dong bo voi setModeCallback case AUTO):
                // xoa selected=1 cua MANUAL + counter ve 1, cho START.
                if (leaving_manual) {
                    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
                    selected_output_slot_ = SLOT_UNSET;
                    current_auto_slot_ = 1;
                }
                RCLCPP_INFO(get_logger(), "[MODE] MANUAL OFF (via service)%s",
                    leaving_manual ? " — slot counter reset to 1" : "");
            }
            res->success = true;
            res->message = req->data ? "Manual mode ON" : "Manual mode OFF";
        },
        rmw_qos_profile_services_default, callback_group_reentrant_);

    soft_stop_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/soft_stop_to_manual",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr /*req*/,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
            softStopToManual("/robot/soft_stop_to_manual");
            res->success = true;
            res->message = "Soft stop OK — motion cancelled, state=IDLE, mode=MANUAL, data preserved";
        },
        rmw_qos_profile_services_default, callback_group_reentrant_);

    set_ai_mode_service_ = create_service<std_srvs::srv::SetBool>(
        "/robot/set_ai_mode",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
            if (req->data) {
                use_ai_for_control_ = true;
                manual_mode_ = false;
                // Cung ly do voi setModeCallback case AI: vao AI la xoa selected
                // mo coi de robot cho vision thay vi dung du lieu cu.
                {
                    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
                    selected_output_slot_ = SLOT_UNSET;
                }
                RCLCPP_INFO(get_logger(), "[MODE] AI (via service)");
            } else {
                use_ai_for_control_ = false;
                RCLCPP_INFO(get_logger(), "[MODE] AI OFF (via service)");
            }
            res->success = true;
            res->message = req->data ? "AI mode ON" : "AI mode OFF";
        },
        rmw_qos_profile_services_default, callback_group_reentrant_);

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_reentrant_;
    set_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/robot/set_mode", 10,
        std::bind(&RobotLogicNode::setModeCallback, this, std::placeholders::_1),
        sub_options);
}

bool RobotLogicNode::exitAfterScalePlaceRequested(const std::string& source)
{
    if (!exit_after_scale_place_.exchange(false))
        return false;
    RCLCPP_WARN(get_logger(),
        "[SCALE_CHOICE] %s xong — phat /system/stop_button, roi process va ve MANUAL",
        source.c_str());
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    stop_button_pub_->publish(msg);   // chinh node nay cung nhan -> softStopToManual
    return true;
}

void RobotLogicNode::softStopToManual(const std::string& source)
{
    // Soft STOP: cancel motion goals NGAY + chuyển MANUAL + reset state machine về IDLE.
    // GIỮ NGUYÊN: row tracking, slot, chamber/buffer flags (data state).
    // Reset current_state_=IDLE để GUI nhận "robotBusy=false" → unlock manual controls.
    RCLCPP_WARN(get_logger(),
        "[SOFT_STOP] %s -> cancel motion + state=IDLE + switch MANUAL (keep data)",
        source.c_str());
    if (motion_action_client_) {
        motion_action_client_->async_cancel_all_goals();
    }
    exit_after_scale_place_ = false;
    system_start_time_  = this->now();   // lan chay sau bat dau tu 0
    current_state_      = SystemState::IDLE;
    system_running_     = false;
    system_started_     = false;
    manual_mode_        = true;
    use_ai_for_control_ = false;
    system_paused_      = false;
    dobot_motion_paused_ = false;
    motion_busy_        = false;
    motion_in_progress_ = false;
    clearMotionCmd();
    output_prescan_home_done_ = false;
    output_prescan_zone_verified_ = false;
    output_tray_change_home_pending_ = false;
    output_tray_change_home_motion_done_ = false;
    post_place_output_check_pending_ = false;
    post_place_output_scan_started_ = false;
    startup_home_pending_ = false;
    startup_home_motion_done_ = false;
    startup_home_required_ = false;
    setOutputScanAllowed(false);

    notifyStateChange();
}

void RobotLogicNode::forceScalePass(const std::string& source)
{
    PendingScaleResult r;
    r.pass         = true;
    r.timestamp    = this->now();
    r.cartridge_id = cartridge_counter_.load();

    // KHONG day ket qua vao hang doi. Ca 4 noi goi forceScalePass (toggle
    // ignore_scale, PROCESSING_SCALE, ERROR_SCALE_TIMEOUT, WAIT_SCALE_CHOICE)
    // deu di thang sang PLACE_TO_OUTPUT, ma consumer duy nhat cua hang doi la
    // processNextScaleResult() nam trong nhanh THUONG cua PROCESSING_SCALE —
    // khong bao gio chay o cac duong nay. Ket qua day vao se nam lai vinh vien,
    // roi WAIT_FILLING thay hasPendingScaleResults() == true va chan chuyen
    // chamber sang can: "fill_done latched but scale not ready (occupied=0
    // result_pending=1)" — ket cung, chi START lai hoac dat khay moi moi go duoc.
    // Quyet dinh PASS di theo stored_scale_result_, khong can hang doi.
    {
        std::lock_guard<std::mutex> lock(scale_result_mutex_);
        pending_scale_results_.clear();
    }

    stored_scale_result_.store(true);
    scale_result_received_ = true;

    RCLCPP_WARN(get_logger(),
        "[SCALE] IGNORE/BYPASS from %s -> force PASS for cartridge #%d",
        source.c_str(), r.cartridge_id);

    if (current_state_ == SystemState::PROCESSING_SCALE ||
        current_state_ == SystemState::ERROR_SCALE_TIMEOUT ||
        current_state_ == SystemState::WAIT_SCALE_CHOICE)
    {
        notifyStateChange();
    }
}

// ============================================================================
// HELPER: getNextAutoRow
// Returns the row number to pick next (1-5).
// When row 5 is returned: publishes done_tray_input and sets waiting flags.
// Returns -1 if currently waiting for a new tray.
// ============================================================================

int RobotLogicNode::getNextAutoRow()
{
    if (waiting_for_new_input_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[ROW] Waiting for new input tray (done_tray_input already published)");
        return -1;
    }

    int row = current_auto_row_;

    if (row == TOTAL_ROWS) {
        // This is the LAST row — publish done_tray_input AFTER picking it
        // (caller picks this row, then we set flags)
        RCLCPP_WARN(get_logger(),
            "[ROW] Picking last row (%d) — will pub done_tray_input after pick", row);
    }

    return row;
}

// ============================================================================
// HELPER: advanceAutoRow
// Called after successfully picking a row
// ============================================================================
void RobotLogicNode::advanceAutoRow()
{
    if (use_ai_for_control_) return;

    if (current_auto_row_ >= TOTAL_ROWS) {
        RCLCPP_WARN(get_logger(), "[TRAY_IN] Last row pick DONE → pub done_tray_input");
        auto doneMsg = std_msgs::msg::Bool();
        doneMsg.data = true;
        done_input_tray_pub_->publish(doneMsg);
        new_tray_loaded_      = false;
        new_tray_loaded_pub_->publish(std_msgs::msg::Bool());
        waiting_for_new_input_ = true;
        cartridge_drain_confirmed_ = false;
        wait_tray_start_time_  = this->now();
        
        current_auto_row_ = 1;
    } else {
        current_auto_row_++;
    }

    if (selected_row_pub_) {
        auto r_msg = std_msgs::msg::Int32();
        r_msg.data = current_auto_row_;
        selected_row_pub_->publish(r_msg);
    }
}


// ============================================================================
// HELPER: publishDoneTrayOutput
// ============================================================================

void RobotLogicNode::publishDoneTrayOutput()
{
    bool expected = false;
    if (!waiting_for_new_output_.compare_exchange_strong(expected, true)) {
        RCLCPP_WARN(get_logger(),
            "[TRAY_OUT] Change request already pending — duplicate suppressed");
        return;
    }
    output_prescan_home_done_ = false;
    output_prescan_zone_verified_ = false;
    output_tray_change_home_pending_ = false;
    output_tray_change_home_motion_done_ = false;
    output_tray_change_home_wait_since_ns_ = 0;
    setOutputScanAllowed(false);
    new_trayoutput_loaded_ = false;
    {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        selected_output_slot_ = SLOT_UNSET;
    }
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    done_output_tray_pub_->publish(msg);
    new_trayoutput_pub_->publish(std_msgs::msg::Bool()); // Publish false to GUI
    RCLCPP_WARN(get_logger(), "[TRAY_OUT] 📤 done_tray_output published — waiting for new_trayoutput_loaded");
}

bool RobotLogicNode::requestOutputTrayChangeForDrain(const char* reason)
{
    if (waiting_for_new_output_.load()) {
        RCLCPP_WARN(get_logger(),
            "[DRAIN] Output tray change already pending — %s", reason);
        return true;
    }

    if (!cartridgeDrainEvidenceFresh() ||
        !cartridge_input_sensors_seen_.load() ||
        !input_belt_empty_from_sensors_.load() ||
        !cartridge_drain_confirmed_.load()) {
        cartridge_drain_confirmed_ = false;
        RCLCPP_ERROR(get_logger(),
            "[DRAIN][PIPELINE_DRAIN] Request blocked (%s) — Cartridge heartbeat/"
            "S1-S3/drain evidence missing or stale", reason);
        return false;
    }

    RCLCPP_WARN(get_logger(),
        "[DRAIN][PIPELINE_DRAIN] Pipeline drained — request output tray change "
        "for STATE4/STATE3 (%s); vision heartbeat is not required", reason);
    setOutputScanAllowed(false);
    publishDoneTrayOutput();
    return waiting_for_new_output_.load();
}

// ============================================================================
// CALLBACKS
// ============================================================================

void RobotLogicNode::newTrayCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        // Authoritative cartridge state: no tray at the robot position.  Clear
        // stale READY rows even if vision is frozen by PAUSE/recovery gating.
        new_tray_loaded_ = false;
        waiting_for_new_input_ = true;
        {
            std::lock_guard<std::mutex> lock(row_selection_mutex_);
            selected_input_row_ = ROW_UNSET;
            std::fill(row_full_.begin(), row_full_.end(), false);
            input_tray_empty_ = false;
        }
        RCLCPP_INFO(get_logger(),
            "[TRAY_IN] new_tray_loaded=false -> cleared stale input rows");
        notifyStateChange();
        return;
    }

    // A new input tray is loaded by the cartridge system — LUÔN track flag
    // (kể cả trong manual mode) để operator chuyển sang AUTO/AI rồi PICK_INPUT
    // vẫn pick được. Manual mode chỉ chặn AUTO-START pipeline, không chặn data tracking.
    new_tray_loaded_ = true;
    refill_pending_ = true;
    bool was_waiting = waiting_for_new_input_.load();
    waiting_for_new_input_ = false;
    setInputScanAllowed(true);
    // Khong chap nhan empty=true con sot tu khay truoc. Vision phai xac nhan
    // empty=false cua khay moi de arm lai.
    vision_empty_armed_ = false;
    cartridge_drain_confirmed_ = false;

    // Clear scale results sót lại từ batch trước. Loadcell đã có latch, nhưng nếu
    // robot crash giữa chừng, queue có thể còn 1 entry chưa consume — entry đó thuộc
    // batch cũ, không được apply cho batch mới (cartridge_id sẽ lệch). An toàn nhất:
    // wipe sạch khi đặt khay input mới.
    stored_scale_result_.store(false);
    scale_result_received_ = false;
    {
        std::lock_guard<std::mutex> lock(scale_result_mutex_);
        if (!pending_scale_results_.empty()) {
            RCLCPP_WARN(get_logger(),
                "[TRAY_IN] 🧹 Cleared %zu stale scale results from previous batch",
                pending_scale_results_.size());
            pending_scale_results_.clear();
        }
    }

    // Manual mode: tracking đã set, nhưng không auto-trigger pipeline.
    // Operator phải nhấn PICK_INPUT (đã được handle bởi feed_chamber_signal_).
    if (manual_mode_.load()) {
        RCLCPP_INFO(get_logger(), "[TRAY_IN] new_tray_loaded=true (manual mode — chờ PICK_INPUT thủ công)");
        return;
    }

    bool should_reset_to_1 = true;
    
    // Always default a new tray to Row 1, UNLESS the human operator explicitly overrode it
    if (operator_explicitly_set_row_) {
        should_reset_to_1 = false;
        operator_explicitly_set_row_ = false; // Consume the explicit override for this tray only
        RCLCPP_INFO(get_logger(), "[TRAY_IN] ✅ New input tray — honoring operator explicit choice: Row %d", current_auto_row_);
    } else {
        RCLCPP_INFO(get_logger(), "[TRAY_IN] ✅ New input tray — auto-resetting to Row 1");
    }

    if (should_reset_to_1) {
        current_auto_row_ = 1;
        // Broadcast the reset to the GUI so it snaps visual highlight back to R1
        if (selected_row_pub_) {
            auto r_msg = std_msgs::msg::Int32();
            r_msg.data = current_auto_row_;
            selected_row_pub_->publish(r_msg);
        }
    }

    // AUTO may assume a full tray. AI must start unknown/empty and wait for a
    // fresh camera decision; otherwise all five rows flash READY with no tray.
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        std::fill(row_full_.begin(), row_full_.end(), !use_ai_for_control_.load());
        input_tray_empty_ = false;
        if (should_reset_to_1 && !use_ai_for_control_) {
            selected_input_row_ = 1;
        } else if (use_ai_for_control_) {
            selected_input_row_ = ROW_UNSET;
        }
    }

    notifyStateChange();
}

void RobotLogicNode::newTrayOutputCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    output_tray_change_home_pending_ = false;
    output_tray_change_home_motion_done_ = false;
    output_tray_change_home_wait_since_ns_ = 0;
    if (!msg->data) {
        // Authoritative physical state: output tray is absent/not ready.  This
        // may originate from manual State3/4, so do not depend solely on Robot
        // Logic having initiated the tray change.
        new_trayoutput_loaded_ = false;
        waiting_for_new_output_ = true;
        output_prescan_home_done_ = false;
        output_prescan_zone_verified_ = false;
        output_tray_change_home_pending_ = false;
        output_tray_change_home_motion_done_ = false;
        post_place_output_check_pending_ = false;
        post_place_output_scan_started_ = false;
        setOutputScanAllowed(false);
        RCLCPP_WARN(get_logger(),
            "[TRAY_OUT] Output tray NOT READY — all placement paths locked");
        notifyStateChange();
        return;
    }

    output_prescan_zone_verified_ = false;
    post_place_output_check_pending_ = false;
    post_place_output_scan_started_ = false;
    // New physical tray = new camera epoch.  Never retain an old-tray sample.
    setOutputScanAllowed(false);
    new_trayoutput_loaded_ = true;
    waiting_for_new_output_ = false;
    if (manual_mode_.load()) {
        output_prescan_home_done_ = false;
        RCLCPP_INFO(get_logger(),
            "[TRAY_OUT] New output tray recorded in MANUAL — no automatic motion");
    } else if (use_ai_for_control_ && output_scan_recovery_required_.load()) {
        output_prescan_home_done_ = false;
        RCLCPP_ERROR(get_logger(),
            "[TRAY_OUT] New tray recorded, but cam1 remains blocked until robot HOME recovery");
    } else if (use_ai_for_control_ && cartridge_pos2_busy_.load()) {
        RCLCPP_WARN(get_logger(),
            "[SAFETY][CAM1] New tray reported while Pos2 still BUSY — "
            "scan remains blocked until Pos2 FREE");
    } else {
        const bool robot_still_home = !use_ai_for_control_ || cam1ViewClearFresh();
        output_prescan_home_done_ = robot_still_home && use_ai_for_control_.load();
        setOutputScanAllowed(robot_still_home);
    }
    // Reset output slot counter for new tray
    current_auto_slot_ = 1;
    {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        selected_output_slot_ = SLOT_UNSET;
    }
    // AUTO: GUI sang den O1 — khay moi bat dau tu slot 1
    if (!use_ai_for_control_ && selected_slot_pub_) {
        auto s_msg = std_msgs::msg::Int32();
        s_msg.data = 1;
        selected_slot_pub_->publish(s_msg);
    }

    RCLCPP_INFO(get_logger(),
        "[TRAY_OUT] ✅ New output tray loaded — slot reset to 1, waiting_for_new_output=false");
    notifyStateChange();
}







void RobotLogicNode::publishCameraStatus(const std::string& status)
{
    auto msg = std_msgs::msg::String();
    msg.data = status;
    camera_status_pub_->publish(msg);
}

void RobotLogicNode::feedChamberCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    feed_chamber_signal_ = msg->data;
    if (msg->data) notifyStateChange();
}

void RobotLogicNode::fillDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        fill_done_ = true;
        RCLCPP_INFO(get_logger(), "[FILL_DONE] ✅ Fill complete");
        notifyStateChange();
    }
}

// ============================================================================
// SCALE QUEUE
// ============================================================================

bool RobotLogicNode::processNextScaleResult(bool& result_pass, int& cartridge_id)
{
    std::lock_guard<std::mutex> lock(scale_result_mutex_);
    if (pending_scale_results_.empty()) return false;
    auto r = pending_scale_results_.front();
    pending_scale_results_.pop_front();
    result_pass = r.pass;
    cartridge_id = r.cartridge_id;
    RCLCPP_INFO(get_logger(), "[SCALE] 📦 Processing result #%d: %s (queue: %zu)",
        cartridge_id, result_pass ? "PASS" : "FAIL", pending_scale_results_.size());
    return true;
}

bool RobotLogicNode::hasPendingScaleResults()
{
    std::lock_guard<std::mutex> lock(scale_result_mutex_);
    return !pending_scale_results_.empty();
}

void RobotLogicNode::scaleResultCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool pass = ignore_scale_ ? true : msg->data;

    PendingScaleResult r;
    r.pass         = pass;
    r.timestamp    = this->now();
    r.cartridge_id = cartridge_counter_.load();

    {
        std::lock_guard<std::mutex> lock(scale_result_mutex_);
        pending_scale_results_.push_back(r);
        if (pending_scale_results_.size() > 10) {
            RCLCPP_WARN(get_logger(), "[SCALE] Queue overflow — dropping oldest");
            pending_scale_results_.pop_front();
        }
    }

    stored_scale_result_.store(pass);
    scale_result_received_ = true;

    RCLCPP_INFO(get_logger(), "[SCALE] ⚡ Result #%d: %s",
        r.cartridge_id, pass ? "PASS" : "FAIL");

    if (current_state_ == SystemState::PROCESSING_SCALE ||
        current_state_ == SystemState::ERROR_SCALE_TIMEOUT)
        notifyStateChange();
}

void RobotLogicNode::startButtonCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) return;

    if (system_running_.load() && !startup_home_required_.load() &&
        !startup_home_pending_.load()) {
        RCLCPP_WARN(get_logger(),
            "[START] Ignored duplicate START — automatic cycle is already running");
        return;
    }

    // Same rule as the service path, and it matters more here: this topic is
    // what /api/start on the web HMI publishes, which never sees the GUI's
    // START lock. Continue() below would hand the held trajectory back to the
    // Dobot while the state machine stayed frozen on system_paused_.
    if (system_paused_.load()) {
        RCLCPP_WARN(get_logger(),
            "[START] Rejected (/system/start_button): system is PAUSED — press RESUME or STOP");
        return;
    }

    RCLCPP_INFO(get_logger(), "[INIT] ⚡ Quick start...");

    if (emergency_stop_) emergency_stop_ = false;
    system_enabled_ = true;
    system_running_ = true;
    startup_home_required_ = true;
    // REMOVED: waiting_for_new_input_ = false; (operator pressed start -> we must still wait for tray to be strictly loaded)
    system_start_time_ = this->now();

    auto clear_req = std::make_shared<ClearError::Request>();
    callService<ClearError>(clear_error_client_, clear_req, "ClearError");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto enable_req = std::make_shared<EnableRobot::Request>();
    enable_req->load = 0.0;
    callService<EnableRobot>(enable_client_, enable_req, "EnableRobot");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // START never resumes an old paused trajectory. Only explicit RESUME may
    // call Continue(), after revalidating the live Cartridge interlocks.
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    if (!manual_mode_) {
        if (cartridge_is_homed_) {
            RCLCPP_INFO(get_logger(), "[INIT] Cartridge is already homed. Executing Robot HOME and starting process.");
            requestStartupHome("start_button");
        } else {
            RCLCPP_INFO(get_logger(), "[INIT] Auto/AI mode — waiting for Cartridge Homing to finish before moving HOME.");
            // system_started_ will be set to true in cartridgeHomingDoneCallback
        }
    } else {
        RCLCPP_INFO(get_logger(), "[INIT] ⚡ Manual mode — skipping HOME command, holding position");
        // We do NOT set system_started_ = true, because manual mode shouldn't jump to INIT_CHECK sequence
    }

    RCLCPP_INFO(get_logger(), "[INIT] ✅ System Received Start");
    notifyStateChange();
}

void RobotLogicNode::selectedRowCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(row_selection_mutex_);
    if (use_ai_for_control_.load() && !cam0DecisionWindowReady()) {
        selected_input_row_ = ROW_UNSET;
        return;
    }
    int row = msg->data;
    selected_input_row_ =
        (row >= 1 && row <= TOTAL_ROWS) ? row : ROW_UNSET;
    notifyStateChange();
}

void RobotLogicNode::selectedSlotCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!output_scan_allowed_.load()) {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        selected_output_slot_ = SLOT_UNSET;
        return;
    }
    int slot = msg->data;
    {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        selected_output_slot_ =
            (slot >= 1 && slot <= TOTAL_OUTPUT_SLOTS) ? slot : SLOT_UNSET;
        if (selected_output_slot_ != SLOT_UNSET)
            last_output_slot_rx_ns_ = this->now().nanoseconds();
    }
    notifyStateChange();
}

void RobotLogicNode::gotoStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string sn = msg->data;

    if (system_running_ && !manual_mode_ && current_state_ != SystemState::IDLE) {
        // PLACE_TO_OUTPUT / PLACE_TO_FAIL are only valid after PROCESSING_SCALE
        bool is_place_cmd = (sn == "PLACE_TO_OUTPUT" || sn == "PLACE_TO_FAIL");
        bool in_scale_state = (current_state_ == SystemState::PROCESSING_SCALE ||
                               current_state_ == SystemState::ERROR_SCALE_TIMEOUT ||
                               current_state_ == SystemState::PLACE_TO_OUTPUT ||
                               current_state_ == SystemState::PLACE_TO_FAIL);

        // Resume from WAIT_RESUME_CHOICE: chỉ cho phép operator chọn 2 target hợp lệ.
        // PROCESSING_SCALE: operator da tu xu ly chamber → bo qua cho fill, sang can luon.
        bool is_resume_cmd = (sn == "INIT_LOAD_CHAMBER_DIRECT" || sn == "LOAD_CHAMBER_FROM_BUFFER"
                              || sn == "PROCESSING_SCALE");
        bool in_resume_wait = (current_state_ == SystemState::WAIT_RESUME_CHOICE);

        // Scale choice from WAIT_SCALE_CHOICE: 3 target hợp lệ.
        bool is_scale_choice_cmd = (sn == "WAIT_FILLING" || sn == "PLACE_TO_OUTPUT" || sn == "PLACE_TO_FAIL");
        bool in_scale_choice     = (current_state_ == SystemState::WAIT_SCALE_CHOICE);

        if (is_place_cmd && !in_scale_state && !in_scale_choice) {
            RCLCPP_WARN(get_logger(),
                "[GOTO] '%s' ignored — only valid from PROCESSING_SCALE, current: %s",
                sn.c_str(), stateToString(current_state_).c_str());
            return;
        }

        if (!is_place_cmd && !(is_resume_cmd && in_resume_wait) && !(is_scale_choice_cmd && in_scale_choice)) {
            RCLCPP_WARN(get_logger(), "[GOTO] Ignored '%s' — auto pipeline active", sn.c_str());
            return;
        }
    }

    SystemState target = SystemState::IDLE;
    if      (sn == "IDLE")                      target = SystemState::IDLE;
    else if (sn == "INIT_CHECK")                target = SystemState::INIT_CHECK;
    else if (sn == "INIT_LOAD_CHAMBER_DIRECT")  target = SystemState::INIT_LOAD_CHAMBER_DIRECT;
    else if (sn == "INIT_REFILL_BUFFER")        target = SystemState::INIT_REFILL_BUFFER;
    else if (sn == "WAIT_FILLING")              target = SystemState::WAIT_FILLING;
    else if (sn == "TAKE_CHAMBER_TO_SCALE")     target = SystemState::TAKE_CHAMBER_TO_SCALE;
    else if (sn == "PROCESSING_SCALE")          target = SystemState::PROCESSING_SCALE;
    else if (sn == "PLACE_TO_OUTPUT")           target = SystemState::PLACE_TO_OUTPUT;
    else if (sn == "PLACE_TO_FAIL")             target = SystemState::PLACE_TO_FAIL;
    else if (sn == "REFILL_BUFFER")             target = SystemState::REFILL_BUFFER;
    else if (sn == "LOAD_CHAMBER_FROM_BUFFER")  target = SystemState::LOAD_CHAMBER_FROM_BUFFER;
    else if (sn == "WAIT_RESUME_CHOICE")        target = SystemState::WAIT_RESUME_CHOICE;
    else if (sn == "WAIT_SCALE_CHOICE")         target = SystemState::WAIT_SCALE_CHOICE;
    else { RCLCPP_ERROR(get_logger(), "[GOTO] Unknown: %s", sn.c_str()); return; }

    // Resume after feed_chamber timeout: operator đã xử lý thực tế →
    // reset flags theo lựa chọn để pipeline chạy đúng nhánh.
    if (current_state_ == SystemState::WAIT_RESUME_CHOICE &&
        (target == SystemState::INIT_LOAD_CHAMBER_DIRECT ||
         target == SystemState::LOAD_CHAMBER_FROM_BUFFER))
    {
        skipped_buffer_load_     = false;
        feed_chamber_wait_active_ = false;
        feed_chamber_signal_     = false;  // require new feed_chamber tại gate

        if (target == SystemState::INIT_LOAD_CHAMBER_DIRECT) {
            // Operator báo đã xử lý buffer thực tế → coi như bắt đầu process mới
            is_first_batch_        = true;
            buffer_is_empty_       = true;
            chamber_has_cartridge_ = false;
            chamber_is_empty_      = true;
            RCLCPP_WARN(get_logger(),
                "[RESUME] Operator chose INIT_LOAD_CHAMBER_DIRECT → reset to fresh init flow");
        } else {
            RCLCPP_WARN(get_logger(),
                "[RESUME] Operator chose LOAD_CHAMBER_FROM_BUFFER → retry with current buffer");
        }
    }

    // Can da bao su co: dat not cartridge dang tren can roi dung han, khong
    // quay lai vong lap. Neu con chay tiep duoc thi operator da bam IGNORE
    // SCALE va popup nay khong xuat hien.
    if (current_state_ == SystemState::WAIT_SCALE_CHOICE &&
        (target == SystemState::PLACE_TO_OUTPUT || target == SystemState::PLACE_TO_FAIL)) {
        exit_after_scale_place_ = true;
        RCLCPP_WARN(get_logger(),
            "[SCALE_CHOICE] '%s' — dat xong se dung he thong va ve MANUAL", sn.c_str());
    }

    // Scale choice: reset scale timer so PROCESSING_SCALE doesn't re-trigger immediately
    if (current_state_ == SystemState::WAIT_SCALE_CHOICE) {
        scale_wait_start_      = this->now();
        scale_result_received_ = false;
        stored_scale_result_.store(false);
        RCLCPP_WARN(get_logger(), "[SCALE_CHOICE] Operator chose '%s' → resuming", sn.c_str());
    }

    if (target == SystemState::PROCESSING_SCALE) {
        scale_wait_start_ = this->now();
        scale_result_received_ = false;
    }

    if (current_state_ == SystemState::IDLE && target != SystemState::IDLE) {
        system_running_ = true;
        system_enabled_ = true;
        // Duong nay bat lai system_running_ ma khong qua callback START, nen
        // truoc day moc thoi gian cu con nguyen va uptime cong don qua ca nhung
        // lan da STOP. Coi day la mot lan chay moi.
        system_start_time_ = this->now();
    }

    // Unblock stalled auto sequence without dropping to manual mode
    if (target == SystemState::PLACE_TO_OUTPUT ||
        target == SystemState::PLACE_TO_FAIL)
    {
        system_enabled_ = true;
        // Removed: manual_mode_ = true and stop_after_single_motion_ = true
        // so the system continues in the current AUTO sequence naturally!
        RCLCPP_INFO(get_logger(),
            "[GOTO] Emergency Override %s: Auto sequence resumed.",
            sn.c_str());
        
        // Auto-select slot if none selected (for PLACE_TO_OUTPUT)
        if (target == SystemState::PLACE_TO_OUTPUT)
        {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            if (selected_output_slot_ == -1) // Unset or unassigned
            {
                selected_output_slot_ = current_auto_slot_;
                RCLCPP_INFO(get_logger(),
                    "[GOTO] Auto-selected slot %d for PLACE_TO_OUTPUT", selected_output_slot_);
            }
        }
    }

    transitionTo(target);
}

void RobotLogicNode::commandRowCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int row = msg->data;
    if (row < 1 || row > TOTAL_ROWS) return;

    // Prevent changing row while actively running a batch
    if (system_running_ &&
        current_state_ != SystemState::IDLE &&
        current_state_ != SystemState::INIT_CHECK &&
        current_state_ != SystemState::INIT_LOAD_CHAMBER_DIRECT) {
        RCLCPP_WARN(get_logger(),
            "[CMD_ROW] Ignored row %d — system already executing batch", row);
            
        // Force GUI back to current row
        if (selected_row_pub_) {
            auto r_msg = std_msgs::msg::Int32();
            r_msg.data = current_auto_row_;
            selected_row_pub_->publish(r_msg);
        }
        return;
    }

    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        selected_input_row_ = row;
        row_full_[row - 1] = true;
    }
    if (!use_ai_for_control_) {
        current_auto_row_ = row;
    }
    operator_explicitly_set_row_ = true;
    notifyStateChange();
}

void RobotLogicNode::commandSlotCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int slot = msg->data;
    if (slot < 1 || slot > TOTAL_OUTPUT_SLOTS) return;

    // Mirror commandRowCallback: GUI khong duoc doi slot giua batch. Nut O
    // tren CameraPage van bam duoc o AUTO (canSelect), va truoc day callback
    // nay nhan im lang -> selected_output_slot_ de len counter, AUTO nhay coc
    // (vd dang slot 3 bam O10 -> dat slot 10 -> bao tray full -> thay khay non).
    if (system_running_ &&
        current_state_ != SystemState::IDLE &&
        current_state_ != SystemState::INIT_CHECK &&
        current_state_ != SystemState::INIT_LOAD_CHAMBER_DIRECT) {
        RCLCPP_WARN(get_logger(),
            "[CMD_SLOT] Ignored slot %d — system already executing batch", slot);

        // Force GUI highlight back to the slot AUTO is actually counting
        if (selected_slot_pub_ && !use_ai_for_control_) {
            auto s_msg = std_msgs::msg::Int32();
            s_msg.data = current_auto_slot_;
            selected_slot_pub_->publish(s_msg);
        }
        return;
    }

    {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        selected_output_slot_ = slot;
        // AUTO: dong bo counter de sau khi dat slot nay, dem tiep slot+1...
        // (operator chon diem bat dau, vd resume khay dang do tu slot 4).
        if (!use_ai_for_control_) {
            current_auto_slot_ = slot;
        }
    }
    // Echo lai cho GUI de den O sang dung slot vua chon (GUI chi highlight
    // theo /robot/selected_output_slot, khong tu sang khi bam nut).
    if (selected_slot_pub_) {
        auto s_msg = std_msgs::msg::Int32();
        s_msg.data = slot;
        selected_slot_pub_->publish(s_msg);
    }
    RCLCPP_INFO(get_logger(), "[CMD_SLOT] Operator selected slot %d%s", slot,
        use_ai_for_control_ ? "" : " (AUTO counter synced)");
    notifyStateChange();
}

// ============================================================================
// SERVICE CALLBACKS
// ============================================================================

void RobotLogicNode::enableSystemCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        if (emergency_stop_) emergency_stop_ = false;
        system_enabled_ = true;

        auto clear_req = std::make_shared<ClearError::Request>();
        callService<ClearError>(clear_error_client_, clear_req, "ClearError");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        auto enable_req = std::make_shared<EnableRobot::Request>();
        enable_req->load = 0.0;
        callService<EnableRobot>(enable_client_, enable_req, "EnableRobot");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        response->success = true;
        response->message = "Robot Power ON (paused trajectory remains held until RESUME)";
    } else {
        system_enabled_ = false;
        system_started_ = false;
        system_running_ = false;
        startup_home_pending_ = false;
        startup_home_motion_done_ = false;
        startup_home_required_ = false;
        response->success = true;
        response->message = "System DISABLED";
    }
}

void RobotLogicNode::startSystemCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        if (!system_enabled_) {
            response->success = false;
            response->message = "Robot DISABLED (E-Stop). Press ENABLE first.";
            return;
        }
        if (system_running_.load() && !startup_home_required_.load() &&
            !startup_home_pending_.load()) {
            RCLCPP_WARN(get_logger(),
                "[START] Ignored duplicate service START — automatic cycle is already running");
            response->success = true;
            response->message = "System already running";
            return;
        }

        // START is not a way out of PAUSE. It used to fall straight through to
        // the three digital outputs below, which release the gripper and the
        // picker — pressing it while the arm was holding a cartridge or a tray
        // mid-cycle dropped the part. It also never cleared system_paused_, so
        // the state machine stayed frozen afterwards anyway. RESUME continues
        // the held step; STOP ends the cycle.
        if (system_paused_.load()) {
            RCLCPP_WARN(get_logger(),
                "[START] Rejected: system is PAUSED — press RESUME to continue or STOP to end the cycle");
            response->success = false;
            response->message = "System is PAUSED. Press RESUME to continue, or STOP to end the cycle.";
            return;
        }

        system_running_ = true;
        startup_home_required_ = true;
        stop_after_single_motion_ = false;
        system_start_time_ = this->now();

        //  Cần 2 kênh digital 1,2 = false Sau đó mới bắt đầu chạy init
        RCLCPP_INFO(get_logger(), "[INIT] Ensuring Gripper & Picker are OPEN (DO 1 & 2 = False) before init...");
        setDigitalOutput(1, false);  // Gripper NHẢ — safe state trước khi init
        setDigitalOutput(2, false);  // Picker  NHẢ — safe state trước khi init
        setDigitalOutput(6, true);   // Cyl loadcell EXTEND — mặc định mở xy lanh cho robot đặt khay vào (sau khi đổi dây khí)

        // REMOVED: waiting_for_new_input_ = false; (must wait for actual tray)

        if (!manual_mode_) {
            if (cartridge_is_homed_) {
                RCLCPP_INFO(get_logger(),
                    "[INIT] Auto/AI service START — Cartridge already homed; queue verified Robot HOME");
                requestStartupHome("start_service");
            } else {
                RCLCPP_INFO(get_logger(),
                    "[INIT] Auto/AI mode — waiting for Cartridge Homing to finish before moving HOME.");
            }
        } else {
            RCLCPP_INFO(get_logger(), "[INIT] ⚡ Manual mode — skipping HOME command, holding position");
        }

        notifyStateChange();

        response->success = true;
        response->message = "System Started";
    } else {
        system_started_ = false;
        system_running_ = false;
        startup_home_pending_ = false;
        startup_home_motion_done_ = false;
        startup_home_required_ = false;
        response->success = true;
        response->message = "System Stopped";
    }
}

void RobotLogicNode::emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        if (emergency_stop_client_ && emergency_stop_client_->service_is_ready()) {
            auto e_stop_req = std::make_shared<EmergencyStop::Request>();
            emergency_stop_client_->async_send_request(e_stop_req,
                [this](rclcpp::Client<EmergencyStop>::SharedFuture f) {
                    try {
                        RCLCPP_WARN(get_logger(), "[E-STOP] Dobot EmergencyStop result: %d", f.get()->res);
                    } catch (...) {
                        RCLCPP_WARN(get_logger(), "[E-STOP] Dobot EmergencyStop failed");
                    }
                });
        } else {
            RCLCPP_WARN(get_logger(), "[E-STOP] Dobot EmergencyStop service not ready");
        }

        if (stop_script_client_ && stop_script_client_->service_is_ready()) {
            auto stop_script_req = std::make_shared<StopScript::Request>();
            stop_script_client_->async_send_request(stop_script_req,
                [this](rclcpp::Client<StopScript>::SharedFuture f) {
                    try {
                        RCLCPP_WARN(get_logger(), "[E-STOP] Dobot StopScript result: %d", f.get()->res);
                    } catch (...) {
                        RCLCPP_WARN(get_logger(), "[E-STOP] Dobot StopScript failed");
                    }
                });
        } else {
            RCLCPP_WARN(get_logger(), "[E-STOP] Dobot StopScript service not ready");
        }

        if (pause_client_ && pause_client_->service_is_ready()) {
            auto pause_req = std::make_shared<Pause::Request>();
            pause_client_->async_send_request(pause_req,
                [this](rclcpp::Client<Pause>::SharedFuture f) {
                    try {
                        RCLCPP_WARN(get_logger(), "[E-STOP] Dobot Pause result: %d", f.get()->res);
                    } catch (...) {
                        RCLCPP_WARN(get_logger(), "[E-STOP] Dobot Pause failed");
                    }
                });
        } else {
            RCLCPP_WARN(get_logger(), "[E-STOP] Dobot Pause service not ready");
        }

        emergency_stop_ = true;
        system_enabled_ = false;
        system_paused_  = false;
        dobot_motion_paused_ = false;
        system_running_ = false;
        system_started_ = false;
        current_state_  = SystemState::IDLE;
        manual_mode_    = true;
        use_ai_for_control_ = false;
        motion_busy_        = false;
        motion_in_progress_ = false;
        is_first_batch_ = true;
        motion_fail_count_ = 0;
        tray_count_ = 0;
        cartridge_counter_ = 0;
        selected_output_slot_ = 1;
        skipped_buffer_load_     = false;
        feed_chamber_wait_active_ = false;
        clearMotionCmd();
        output_prescan_home_done_ = false;
        output_prescan_zone_verified_ = false;
        output_tray_change_home_pending_ = false;
        output_tray_change_home_motion_done_ = false;
        post_place_output_check_pending_ = false;
        post_place_output_scan_started_ = false;
        startup_home_pending_ = false;
        startup_home_motion_done_ = false;
        startup_home_required_ = false;
        setOutputScanAllowed(false);

        if (motion_action_client_) {
            motion_action_client_->async_cancel_all_goals();
            RCLCPP_WARN(get_logger(), "[E-STOP] Cancelled all in-flight motion goals");
        }

        if (disable_robot_client_ && disable_robot_client_->service_is_ready()) {
            auto disable_req = std::make_shared<DisableRobot::Request>();
            disable_robot_client_->async_send_request(disable_req,
                [this](rclcpp::Client<DisableRobot>::SharedFuture f) {
                    try {
                        RCLCPP_WARN(get_logger(), "[E-STOP] Dobot DisableRobot result: %d", f.get()->res);
                    } catch (...) {
                        RCLCPP_WARN(get_logger(), "[E-STOP] Dobot DisableRobot failed");
                    }
                });
        } else {
            RCLCPP_WARN(get_logger(), "[E-STOP] Dobot DisableRobot service not ready");
        }

        publishError("EMERGENCY STOP");
        notifyStateChange();

        response->success = true;
        response->message = "EMERGENCY STOP — disabled, clear error + enable required";
    } else {
        emergency_stop_ = false;
        response->success = true;
        response->message = "Emergency stop cleared";
    }
}

void RobotLogicNode::resetStateCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_INFO(get_logger(), "[RESET] 🔄 Full state reset...");

    // Cancel mọi motion action goal đang chạy NGAY để motion_executor thoát loop
    // và không tiếp tục thực hiện các bước còn lại trong sequence.
    if (motion_action_client_) {
        motion_action_client_->async_cancel_all_goals();
        RCLCPP_INFO(get_logger(), "[RESET] Cancelled all in-flight motion goals");
    }

    auto clear_req = std::make_shared<ClearError::Request>();
    callService<ClearError>(clear_error_client_, clear_req, "ClearError");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    current_state_  = SystemState::IDLE;
    is_first_batch_ = true;

    // STOP is a new row-selection boundary.  The GUI already clears its local
    // highlight, so clear the authoritative backend state as well; otherwise
    // current_auto_row_ / operator_explicitly_set_row_ can silently reuse the
    // row chosen before STOP on the next START or new-tray event.
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        selected_input_row_ = ROW_UNSET;
        current_auto_row_ = ROW_UNSET;
        operator_explicitly_set_row_ = false;
    }
    if (selected_row_pub_) {
        auto row_msg = std_msgs::msg::Int32();
        row_msg.data = ROW_UNSET;
        selected_row_pub_->publish(row_msg);
    }

    // Output slot tracking is also preserved so it doesn't overwrite completed slots.

    // Robot state
    buffer_is_empty_       = true;
    chamber_is_empty_      = true;
    chamber_has_cartridge_ = false;
    scale_has_cartridge_   = false;

    // Scale
    stored_scale_result_.store(false);
    scale_result_received_ = false;
    {
        std::lock_guard<std::mutex> lock(scale_result_mutex_);
        pending_scale_results_.clear();
    }

    // System flags
    system_started_          = false;
    system_running_          = false;
    feed_chamber_signal_     = false;
    feed_chamber_wait_active_ = false;
    skipped_buffer_load_     = false;
    fill_done_               = false;
    refill_pending_          = false;
    cartridge_drain_confirmed_ = false;
    emergency_stop_          = false;
    system_paused_           = false;
    dobot_motion_paused_     = false;
    manual_mode_             = true;   // Reset về MANUAL (safe default toàn hệ thống)
    stop_after_single_motion_ = false;
    motion_fail_count_       = 0;
    tray_count_              = 0;
    cartridge_counter_       = 0;
    selected_output_slot_    = 1;
    output_prescan_home_done_ = false;
    output_prescan_zone_verified_ = false;
    output_scan_recovery_required_ = false;
    output_tray_change_home_pending_ = false;
    output_tray_change_home_motion_done_ = false;
    post_place_output_check_pending_ = false;
    post_place_output_scan_started_ = false;
    startup_home_pending_ = false;
    startup_home_motion_done_ = false;
    startup_home_required_ = false;
    setOutputScanAllowed(false);

    RCLCPP_INFO(get_logger(), "[RESET] ✅ Reset complete");

    response->success = true;
    response->message = "All states reset";
    notifyStateChange();
}

void RobotLogicNode::pauseSystemCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    std::lock_guard<std::mutex> pause_lock(pause_mutex_);
    if (request->data) {
        // PAUSE tức thời: khóa state machine trước. Chỉ gửi native
        // Pause() khi Dobot thực sự đang RUNNING (mode 7). Ở WAIT_FILLING,
        // IDLE hoặc khe giữa hai motion primitive, Pause() trả -1 là bình
        // thường; MotionExecutor vẫn giữ action/step qua pause topic.
        if (system_paused_) {
            response->success = true;
            response->message = "Already paused";
            return;
        }
        system_paused_ = true;
        dobot_motion_paused_ = false;
        publishSystemStatus("PAUSED");

        int controller_mode = -1;
        if (robot_mode_client_ && robot_mode_client_->service_is_ready()) {
            auto mode_req = std::make_shared<RobotMode::Request>();
            auto mode_res = callService<RobotMode>(robot_mode_client_, mode_req, "RobotMode(Pause)");
            if (mode_res && mode_res->res == 0) {
                try { controller_mode = std::stoi(mode_res->mode); }
                catch (...) { controller_mode = -1; }
            }
        }
        const bool tracked_motion_active =
            motion_in_progress_.load() || motion_busy_.load();
        const bool native_pause_needed =
            controller_mode == 7 || (controller_mode < 0 && tracked_motion_active);

        if (controller_mode == 10) {
            dobot_motion_paused_ = true;
            RCLCPP_WARN(get_logger(),
                "[PAUSE] Dobot was already paused (mode=10); state/step held");
            response->success = true;
            response->message = "System PAUSED; Dobot already paused";
        } else if (native_pause_needed && pause_client_ && pause_client_->service_is_ready()) {
            auto pause_req = std::make_shared<Pause::Request>();
            auto pause_res = callService<Pause>(pause_client_, pause_req, "Pause");
            if (pause_res && pause_res->res == 0) {
                dobot_motion_paused_ = true;
                RCLCPP_WARN(get_logger(),
                    "[PAUSE] Dobot motion paused at current position");
                response->success = true;
                response->message = "System PAUSED; Dobot motion paused";
            } else {
                // The motion may finish in the small RobotMode->Pause race.
                // Re-read mode: idle means software pause is sufficient.
                int mode_after_pause = -1;
                auto mode_req = std::make_shared<RobotMode::Request>();
                auto mode_res = callService<RobotMode>(
                    robot_mode_client_, mode_req, "RobotMode(after Pause)");
                if (mode_res && mode_res->res == 0) {
                    try { mode_after_pause = std::stoi(mode_res->mode); }
                    catch (...) { mode_after_pause = -1; }
                }
                if (mode_after_pause == 10) {
                    dobot_motion_paused_ = true;
                    response->success = true;
                    response->message = "System PAUSED; Dobot reports paused";
                } else if (mode_after_pause == 5) {
                    RCLCPP_WARN(get_logger(),
                        "[PAUSE] Motion completed before native Pause; holding next step");
                    response->success = true;
                    response->message = "System PAUSED between motion steps";
                } else {
                    const int result = pause_res ? pause_res->res : -1;
                    RCLCPP_ERROR(get_logger(),
                        "[PAUSE] Dobot Pause failed result=%d mode=%d; state remains held",
                        result, mode_after_pause);
                    response->success = false;
                    response->message = "State held, but Dobot motion pause failed";
                }
            }
        } else if (native_pause_needed) {
            RCLCPP_ERROR(get_logger(),
                "[PAUSE] Active motion detected but Dobot Pause service is unavailable; state remains held");
            response->success = false;
            response->message = "State held, but Dobot Pause service is offline";
        } else {
            RCLCPP_WARN(get_logger(),
                "[PAUSE] Software hold only (Dobot mode=%d, motion_in_progress=%s)",
                controller_mode, motion_in_progress_.load() ? "true" : "false");
            response->success = true;
            response->message = "System PAUSED; no active Dobot trajectory";
        }
    } else {
        // Chỉ Continue() khi native Pause() đã thực sự được engage.
        // Software-only pause (idle/between-step) mở state machine ngay.
        if (!system_paused_) {
            response->success = true;
            response->message = "Already running";
            return;
        }
        const bool native_pause_engaged = dobot_motion_paused_.load();
        if (native_pause_engaged) {
            if (!cartridgeSafetyFreshAndClear(true)) {
                RCLCPP_ERROR(get_logger(),
                    "[RESUME] Blocked — Cartridge heartbeat missing/stale or mechanism BUSY");
                response->success = false;
                response->message = "Still paused: Cartridge safety interlock is busy/stale";
                return;
            }
            if (!continue_client_ || !continue_client_->service_is_ready()) {
                RCLCPP_ERROR(get_logger(), "[RESUME] Dobot Continue service not ready");
                response->success = false;
                response->message = "Still paused: Dobot Continue service is offline";
                return;
            }

            auto continue_req = std::make_shared<Continues::Request>();
            auto continue_res = callService<Continues>(continue_client_, continue_req, "Continue");
            if (!continue_res || continue_res->res != 0) {
                // A completed/cancelled trajectory can make Continue return -1.
                // Only mode 10 means the controller is genuinely still paused.
                int controller_mode = -1;
                auto mode_req = std::make_shared<RobotMode::Request>();
                auto mode_res = callService<RobotMode>(
                    robot_mode_client_, mode_req, "RobotMode(after Continue)");
                if (mode_res && mode_res->res == 0) {
                    try { controller_mode = std::stoi(mode_res->mode); }
                    catch (...) { controller_mode = -1; }
                }
                if (controller_mode == 10 || controller_mode < 0) {
                    const int result = continue_res ? continue_res->res : -1;
                    RCLCPP_ERROR(get_logger(),
                        "[RESUME] Dobot still paused; Continue result=%d mode=%d",
                        result, controller_mode);
                    response->success = false;
                    response->message = "Still paused: Dobot Continue failed";
                    return;
                }
                RCLCPP_WARN(get_logger(),
                    "[RESUME] Continue returned nonzero but Dobot mode=%d is not paused; resuming state machine",
                    controller_mode);
            } else {
                RCLCPP_INFO(get_logger(),
                    "[RESUME] Dobot motion continuing from paused position");
            }
        } else {
            RCLCPP_INFO(get_logger(),
                "[RESUME] Software-only pause; no Dobot Continue required");
        }

        motion_fail_count_ = 0;
        if (!motion_in_progress_ && !motion_result_ && !getMotionCmd().empty())
            clearMotionCmd();
        system_paused_ = false;
        dobot_motion_paused_ = false;
        auto now = this->now();
        motion_started_at_   = now;
        wait_tray_start_time_ = now;
        scale_wait_start_    = now;
        RCLCPP_INFO(get_logger(), "[RESUME] State machine resumed — timers refreshed");
        notifyStateChange();
        response->success = true;
        response->message = native_pause_engaged
            ? "System RESUMED; Dobot motion continued"
            : "System RESUMED; software hold released";
    }
}

void RobotLogicNode::setModeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    switch (msg->data) {
        case 1: {
            // DESIGN: khong co chuyen doi mode giua batch — phai STOP truoc.
            // Quay lai AUTO = chu ky moi: counter ve slot 1, operator nhan START.
            // CHI reset khi THUC SU chuyen tu mode khac sang (GUI co luc publish
            // set_mode AUTO lap lai — message AUTO->AUTO trung lap ma reset thi
            // counter dang chay batch bi keo ve 1 -> dat de slot co hang).
            const bool entering_auto = manual_mode_ || use_ai_for_control_;
            manual_mode_ = false;
            use_ai_for_control_ = false;
            if (entering_auto) {
                {
                    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
                    selected_output_slot_ = SLOT_UNSET;
                    current_auto_slot_ = 1;
                }
                // GUI sang den O1 ngay khi vao AUTO (chu ky moi)
                if (selected_slot_pub_) {
                    auto s_msg = std_msgs::msg::Int32();
                    s_msg.data = 1;
                    selected_slot_pub_->publish(s_msg);
                }
                output_prescan_home_done_ = false;
                output_prescan_zone_verified_ = false;
                output_tray_change_home_pending_ = false;
                output_tray_change_home_motion_done_ = false;
                post_place_output_check_pending_ = false;
                post_place_output_scan_started_ = false;
                setOutputScanAllowed(false);
            }
            RCLCPP_INFO(get_logger(),
                "[MODE] AUTO (giữ nguyên new_tray_loaded — tray vẫn còn vật lý)%s",
                entering_auto ? " — slot counter reset to 1" : "");
            break;
        }
        case 2: {
            const bool entering_ai = manual_mode_ || !use_ai_for_control_;
            manual_mode_ = false;
            use_ai_for_control_ = true;
            // AI phai thuan vision-driven: xoa selected mo coi (MANUAL set =1)
            // de neu vision chua/khong publish thi robot CHO camera ("Waiting
            // for slot selection...") thay vi dat theo du lieu cu.
            if (entering_ai) {
                {
                    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
                    selected_output_slot_ = SLOT_UNSET;
                }
                output_prescan_home_done_ = false;
                output_prescan_zone_verified_ = false;
                output_tray_change_home_pending_ = false;
                output_tray_change_home_motion_done_ = false;
                post_place_output_check_pending_ = false;
                post_place_output_scan_started_ = false;
                setOutputScanAllowed(false);
            }
            if (entering_ai)
                RCLCPP_INFO(get_logger(), "[MODE] AI camera");
            break;
        }
        case 3:
            manual_mode_ = true;
            use_ai_for_control_ = false;
            RCLCPP_INFO(get_logger(), "[MODE] MANUAL");
            {
                std::lock_guard<std::mutex> lock(row_selection_mutex_);
                for (size_t i=0; i<row_full_.size(); ++i) row_full_[i] = true;
                input_tray_empty_ = false;
                selected_input_row_ = ROW_UNSET;
            }
            buffer_is_empty_ = true;
            {
                std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
                selected_output_slot_ = 1;
            }
            output_prescan_home_done_ = false;
            output_prescan_zone_verified_ = false;
            output_tray_change_home_pending_ = false;
            output_tray_change_home_motion_done_ = false;
            post_place_output_check_pending_ = false;
            post_place_output_scan_started_ = false;
            startup_home_pending_ = false;
            startup_home_motion_done_ = false;
            startup_home_required_ = false;
            setOutputScanAllowed(false);
            break;
        default:
            RCLCPP_ERROR(get_logger(), "[MODE] Invalid: %d", msg->data);
            return;
    }

    // MODE and START arrive on different ROS topics, so START may be handled
    // first while the node is still in MANUAL.  If AUTO/AI arrives afterwards,
    // continue the pending start here instead of remaining idle forever.
    if ((msg->data == 1 || msg->data == 2) &&
        system_running_ && startup_home_required_.load() && !system_started_) {
        if (cartridge_is_homed_) {
            RCLCPP_INFO(get_logger(),
                "[MODE] AUTO/AI arrived after START — executing pending Robot HOME");
            requestStartupHome("mode_after_start");
        } else {
            RCLCPP_INFO(get_logger(),
                "[MODE] AUTO/AI arrived after START — waiting for Cartridge Homing");
        }
    }

    notifyStateChange();
}

void RobotLogicNode::cartridgeHomingDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        cartridge_is_homed_ = false;
        return;
    }

    cartridge_is_homed_ = true;

    // Chỉ chain HOME khi:
    //   - system_running_ = true (user đã nhấn START)
    //   - !manual_mode_   (auto/ai mode)
    //   - !system_started_ (chưa bao giờ chạy chain init — đây mới là lần đầu)
    // → Operator nhấn HOMING thủ công trong manual/jog hoặc giữa cycle KHÔNG kéo
    //    theo robot home.
    if (system_running_ && startup_home_required_.load() &&
        !manual_mode_ && !system_started_) {
        RCLCPP_INFO(get_logger(), "[INIT] Cartridge Homing Done. Executing Robot HOME and starting process.");
        requestStartupHome("cartridge_homing_done");
        notifyStateChange();
    } else {
        RCLCPP_INFO(get_logger(),
            "[HOMING] Cartridge homed (running=%d manual=%d started=%d) — robot KHÔNG auto chain HOME",
            system_running_.load(), manual_mode_.load(), system_started_.load());
    }
}

// ============================================================================
// STATE MACHINE LOOP
// ============================================================================

void RobotLogicNode::notifyStateChange()
{
    state_changed_ = true;
    state_cv_.notify_all();
}

int64_t RobotLogicNode::steadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void RobotLogicNode::invalidateOutputSnapshot()
{
    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
    output_snapshot_valid_ = false;
    output_snapshot_present_ = false;
    output_snapshot_slot_ = SLOT_UNSET;
    output_snapshot_occupied_.fill(true);
    output_snapshot_frame_ns_ = 0;
    output_snapshot_rx_ns_ = 0;
    selected_output_slot_ = SLOT_UNSET;
    std::fill(output_slot_occupied_.begin(), output_slot_occupied_.end(), true);
}

bool RobotLogicNode::claimFreshOutputSnapshot(int& slot)
{
    const int64_t now_ns = this->now().nanoseconds();
    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
    const int candidate = output_snapshot_slot_;
    const bool fresh = output_snapshot_rx_ns_ > 0 && now_ns >= output_snapshot_rx_ns_ &&
        now_ns - output_snapshot_rx_ns_ <= 2000000000LL &&
        output_snapshot_frame_ns_ > 0 && now_ns >= output_snapshot_frame_ns_ &&
        now_ns - output_snapshot_frame_ns_ <= 2000000000LL;
    const bool usable = output_snapshot_valid_ && output_snapshot_present_ && fresh &&
        candidate >= 1 && candidate <= TOTAL_OUTPUT_SLOTS &&
        !output_snapshot_occupied_[static_cast<size_t>(candidate - 1)];
    if (!usable)
        return false;

    slot = candidate;
    // Claim before releasing the mutex so neither legacy callbacks nor the
    // aggregate callback can replace the selected destination in the small
    // interval before the scan-gate DDS message is published.
    output_scan_allowed_ = false;
    output_snapshot_valid_ = false;
    output_snapshot_present_ = false;
    output_snapshot_slot_ = SLOT_UNSET;
    output_snapshot_occupied_.fill(true);
    output_snapshot_frame_ns_ = 0;
    output_snapshot_rx_ns_ = 0;
    selected_output_slot_ = SLOT_UNSET;
    std::fill(output_slot_occupied_.begin(), output_slot_occupied_.end(), true);
    return true;
}

void RobotLogicNode::requestOutputSnapshotNow()
{
    const int64_t now_ns = this->now().nanoseconds();
    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
    // Keep Vision's gate open and its debouncers warm. Only reject aggregate
    // samples captured before the PLACE_TO_OUTPUT event, so the next camera
    // frame becomes the decision without an extra HOME or 1.5 s cooldown.
    output_snapshot_valid_ = false;
    output_snapshot_present_ = false;
    output_snapshot_slot_ = SLOT_UNSET;
    output_snapshot_occupied_.fill(true);
    output_snapshot_frame_ns_ = 0;
    output_snapshot_rx_ns_ = 0;
    selected_output_slot_ = SLOT_UNSET;
    std::fill(output_slot_occupied_.begin(), output_slot_occupied_.end(), true);
    output_snapshot_accept_after_ns_ = now_ns;
    output_place_snapshot_requested_ = true;
}

bool RobotLogicNode::claimFreshFullOutputSnapshot()
{
    const int64_t now_ns = this->now().nanoseconds();
    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
    const bool fresh = output_snapshot_rx_ns_ > 0 && now_ns >= output_snapshot_rx_ns_ &&
        now_ns - output_snapshot_rx_ns_ <= 2000000000LL &&
        output_snapshot_frame_ns_ > 0 && now_ns >= output_snapshot_frame_ns_ &&
        now_ns - output_snapshot_frame_ns_ <= 2000000000LL;
    const bool full = output_snapshot_valid_ && output_snapshot_present_ && fresh &&
        output_snapshot_slot_ == SLOT_UNSET &&
        std::all_of(output_snapshot_occupied_.begin(), output_snapshot_occupied_.end(),
                    [](bool occupied) { return occupied; });
    if (!full)
        return false;

    // Claim the full decision under the same lock used by aggregate updates.
    output_scan_allowed_ = false;
    output_snapshot_valid_ = false;
    output_snapshot_present_ = false;
    output_snapshot_slot_ = SLOT_UNSET;
    output_snapshot_occupied_.fill(true);
    output_snapshot_frame_ns_ = 0;
    output_snapshot_rx_ns_ = 0;
    selected_output_slot_ = SLOT_UNSET;
    std::fill(output_slot_occupied_.begin(), output_slot_occupied_.end(), true);
    return true;
}

bool RobotLogicNode::cam0DecisionWindowReady() const
{
    if (!input_scan_allowed_.load() || !cam0_view_clear_.load())
        return false;
    const int64_t now_ns = steadyNowNs();
    const int64_t seen_ns = cam0_view_last_steady_ns_.load();
    const int64_t accept_ns = input_decision_accept_after_steady_ns_.load();
    return seen_ns > 0 && accept_ns > 0 && now_ns >= seen_ns &&
        now_ns - seen_ns <= 2000000000LL && now_ns >= accept_ns;
}

bool RobotLogicNode::cam0CheckpointFrameReceived() const
{
    if (!cam0DecisionWindowReady())
        return false;
    const int64_t accept_ns = input_decision_accept_after_steady_ns_.load();
    const int64_t rx_ns = input_decision_last_rx_steady_ns_.load();
    return rx_ns > 0 && accept_ns > 0 && rx_ns >= accept_ns;
}

bool RobotLogicNode::cam1ViewClearFresh() const
{
    const int64_t seen_ns = cam1_view_last_steady_ns_.load();
    const int64_t now_ns = steadyNowNs();
    return cam1_view_clear_.load() && seen_ns > 0 && now_ns >= seen_ns &&
        now_ns - seen_ns <= 2000000000LL;
}

bool RobotLogicNode::mechanicalOutputZoneClearFresh() const
{
    const int64_t seen_ns = mechanical_output_zone_last_steady_ns_.load();
    const int64_t now_ns = steadyNowNs();
    return mechanical_output_zone_clear_.load() && seen_ns > 0 &&
        now_ns >= seen_ns && now_ns - seen_ns <= 2000000000LL;
}

bool RobotLogicNode::visionHeartbeatFresh()
{
    const int64_t seen_ns = vision_hb_last_steady_ns_.load();
    const int64_t now_ns = steadyNowNs();
    return vision_hb_seen_.load() && seen_ns > 0 && now_ns >= seen_ns &&
        now_ns - seen_ns <= 2000000000LL;
}

bool RobotLogicNode::cartridgeHeartbeatFresh() const
{
    const int64_t seen_ns = cartridge_hb_last_steady_ns_.load();
    const int64_t now_ns = steadyNowNs();
    return seen_ns > 0 && now_ns >= seen_ns && now_ns - seen_ns <= 2000000000LL;
}

bool RobotLogicNode::cartridgeSafetyFreshAndClear(bool require_pos2) const
{
    return cartridgeHeartbeatFresh() && !cartridge_busy_.load() &&
        (!require_pos2 || !cartridge_pos2_busy_.load());
}

bool RobotLogicNode::cartridgeDrainEvidenceFresh() const
{
    const int64_t now_ns = steadyNowNs();
    const int64_t sensor_ns = cartridge_sensors_last_steady_ns_.load();
    const int64_t drain_ns = cartridge_drain_last_steady_ns_.load();
    return cartridgeHeartbeatFresh() && sensor_ns > 0 && drain_ns > 0 &&
        now_ns >= sensor_ns && now_ns >= drain_ns &&
        now_ns - sensor_ns <= 2000000000LL &&
        now_ns - drain_ns <= 2000000000LL;
}

void RobotLogicNode::requestStartupHome(const char* source)
{
    startup_home_required_ = true;
    if (startup_home_pending_.exchange(true)) {
        RCLCPP_INFO(get_logger(),
            "[STARTUP_HOME] Already pending (%s) — duplicate suppressed", source);
        return;
    }
    startup_home_motion_done_ = false;
    startup_home_wait_since_ns_ = 0;
    system_started_ = false;
    RCLCPP_INFO(get_logger(),
        "[STARTUP_HOME] Requested by %s — pipeline held until HOME is physically verified",
        source);
    notifyStateChange();
}

bool RobotLogicNode::servicePendingStartupHome()
{
    if (!startup_home_pending_.load())
        return false;

    if (!system_running_.load() || manual_mode_.load()) {
        startup_home_pending_ = false;
        startup_home_motion_done_ = false;
        return false;
    }

    const std::string cmd = getMotionCmd();
    if (cmd == "STARTUP_HOME") {
        if (motion_in_progress_.load())
            return true;
        clearMotionCmd();
        if (!motion_result_.load()) {
            if (last_motion_interlock_rejected_.exchange(false)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[STARTUP_HOME] Admission temporarily vetoed by Cartridge — retry pending");
                return true;
            }
            startup_home_pending_ = false;
            startup_home_motion_done_ = false;
            startup_home_required_ = false;
            system_started_ = false;
            publishError("STARTUP_HOME_FAILED");
            RCLCPP_ERROR(get_logger(),
                "[STARTUP_HOME] HOME failed — automatic pipeline remains IDLE");
            return true;
        }
        startup_home_motion_done_ = true;
        startup_home_wait_since_ns_ = steadyNowNs();
        RCLCPP_INFO(get_logger(),
            "[STARTUP_HOME] Motion complete — waiting 0.5 s live HOME-zone confirmation");
    }

    if (startup_home_motion_done_.load()) {
        if (mechanicalOutputZoneClearFresh()) {
            startup_home_pending_ = false;
            startup_home_motion_done_ = false;
            startup_home_wait_since_ns_ = 0;
            startup_home_required_ = false;
            input_scan_recovery_required_ = false;
            output_scan_recovery_required_ = false;
            setInputScanAllowed(true);
            if (use_ai_for_control_.load()) {
                output_prescan_home_done_ = true;
                output_prescan_zone_verified_ = false;
                setOutputScanAllowed(false);
                setOutputScanAllowed(true);
            }
            system_started_ = true;
            RCLCPP_INFO(get_logger(),
                "[STARTUP_HOME] HOME physically verified — automatic pipeline released");
            notifyStateChange();
            return true;
        }
        const int64_t since_ns = startup_home_wait_since_ns_.load();
        const int64_t now_ns = steadyNowNs();
        if (since_ns > 0 && now_ns >= since_ns && now_ns - since_ns > 5000000000LL) {
            startup_home_pending_ = false;
            startup_home_motion_done_ = false;
            startup_home_required_ = false;
            system_started_ = false;
            publishError("STARTUP_HOME_ZONE_NOT_VERIFIED");
            RCLCPP_ERROR(get_logger(),
                "[STARTUP_HOME] Motion reported success but HOME zone was not verified within 5 s");
        }
        return true;
    }

    if (motion_in_progress_.load() || motion_busy_.load())
        return true;
    if (!cartridgeSafetyFreshAndClear(true)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[STARTUP_HOME] Waiting for fresh Cartridge heartbeat and FREE locks");
        return true;
    }
    sendMotionActionAsync("STARTUP_HOME");
    return true;
}

bool RobotLogicNode::consumeTemporaryMotionInterlockReject(
    const char* context, bool reopen_input_scan)
{
    if (!last_motion_interlock_rejected_.exchange(false))
        return false;
    if (reopen_input_scan && use_ai_for_control_.load())
        setInputScanAllowed(true);
    RCLCPP_WARN(get_logger(),
        "[%s] Motion admission temporarily vetoed by live Cartridge interlock; "
        "no Dobot primitive ran — state will retry", context);
    return true;
}

bool RobotLogicNode::servicePendingOutputTrayChangeHome()
{
    if (!output_tray_change_home_pending_.load())
        return false;

    if (waiting_for_new_output_.load()) {
        output_tray_change_home_pending_ = false;
        output_tray_change_home_motion_done_ = false;
        output_tray_change_home_wait_since_ns_ = 0;
        return false;
    }

    const std::string cmd = getMotionCmd();
    if (cmd == "OUTPUT_TRAY_CHANGE_HOME") {
        if (motion_in_progress_.load())
            return true;
        clearMotionCmd();
        if (!motion_result_.load()) {
            if (last_motion_interlock_rejected_.exchange(false)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[CAM1_FULL] HOME admission temporarily vetoed by Cartridge — retry pending");
                return true;
            }
            output_tray_change_home_pending_ = false;
            output_tray_change_home_motion_done_ = false;
            output_scan_recovery_required_ = true;
            publishError("OUTPUT_TRAY_CHANGE_HOME_FAILED");
            RCLCPP_ERROR(get_logger(),
                "[CAM1_FULL] Robot could not evacuate to HOME — State 4 request NOT sent");
            return true;
        }
        output_tray_change_home_motion_done_ = true;
        output_tray_change_home_wait_since_ns_ = steadyNowNs();
        RCLCPP_INFO(get_logger(),
            "[CAM1_FULL] HOME motion complete — waiting for live mechanical-zone dwell");
    }

    if (mechanicalOutputZoneClearFresh()) {
        output_tray_change_home_pending_ = false;
        output_tray_change_home_motion_done_ = false;
        output_tray_change_home_wait_since_ns_ = 0;
        RCLCPP_WARN(get_logger(),
            "[CAM1_FULL] Robot HOME mechanically verified — request State 4 now");
        publishDoneTrayOutput();
        return true;
    }

    // Once the HOME action succeeds, never send it again while Motion
    // Executor's 0.5 s joint-feedback dwell is still becoming valid. Re-sending
    // would reset that dwell forever and State 4 would never start.
    if (output_tray_change_home_motion_done_.load()) {
        const int64_t since_ns = output_tray_change_home_wait_since_ns_.load();
        const int64_t now_ns = steadyNowNs();
        if (since_ns > 0 && now_ns >= since_ns && now_ns - since_ns > 5000000000LL) {
            output_tray_change_home_pending_ = false;
            output_tray_change_home_motion_done_ = false;
            output_scan_recovery_required_ = true;
            publishError("OUTPUT_TRAY_CHANGE_HOME_ZONE_NOT_VERIFIED");
            RCLCPP_ERROR(get_logger(),
                "[CAM1_FULL] HOME action succeeded but mechanical output zone "
                "was not verified within 5 s — State 4 remains blocked");
        }
        return true;
    }

    if (motion_in_progress_.load() || motion_busy_.load())
        return true;
    if (!cartridgeSafetyFreshAndClear(true)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[CAM1_FULL] Waiting Cartridge FREE/fresh before HOME evacuation");
        return true;
    }

    output_prescan_zone_verified_ = false;
    setOutputScanAllowed(false);
    RCLCPP_WARN(get_logger(),
        "[CAM1_FULL] Robot at camera scan pose — moving HOME once to release State 4 zone");
    sendMotionActionAsync("OUTPUT_TRAY_CHANGE_HOME");
    return true;
}

void RobotLogicNode::setInputScanAllowed(bool allowed)
{
    input_scan_allowed_ = allowed;
    input_decision_accept_after_steady_ns_ = allowed
        ? steadyNowNs() + 1500000000LL
        : 0;
    input_decision_last_rx_steady_ns_ = 0;
    vision_empty_armed_ = false;
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        selected_input_row_ = ROW_UNSET;
        // Every action edge creates a new camera decision epoch. Never reuse
        // the row consumed before INPUT_TRAY_CHAMBER/BUFFER, even if a queued
        // legacy row message arrives before cam0 is physically clear.
        std::fill(row_full_.begin(), row_full_.end(), false);
    }
    if (!input_scan_allowed_pub_) return;
    auto msg = std_msgs::msg::Bool();
    msg.data = allowed;
    input_scan_allowed_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "[AI_SCAN] Input tray scan %s",
        allowed
            ? "ARMED — waiting for stable HOME/index 0 + 1.5s clean cam0 frame"
            : "BLOCKED (input pick motion)");
}

void RobotLogicNode::setOutputScanAllowed(bool allowed)
{
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        changed = output_scan_allowed_.load() != allowed;
        output_scan_allowed_ = allowed;
        // Repeating TRUE must not move the acceptance window forever.  Every
        // real edge starts a new epoch; FALSE is always destructive.
        if (changed || !allowed) {
            selected_output_slot_ = SLOT_UNSET;
            output_snapshot_valid_ = false;
            output_snapshot_present_ = false;
            output_snapshot_slot_ = SLOT_UNSET;
            output_snapshot_occupied_.fill(true);
            output_snapshot_frame_ns_ = 0;
            output_snapshot_rx_ns_ = 0;
            output_snapshot_accept_after_ns_ = allowed
                ? this->now().nanoseconds() + 1500000000LL
                : 0;
        }
        // Same snapshot policy as input rows: do not blank the GUI during the
        // place motion, but require a fresh slot decision after it completes.
        if (allowed)
            std::fill(output_slot_occupied_.begin(), output_slot_occupied_.end(), true);
    }
    if (!output_scan_allowed_pub_) return;
    auto msg = std_msgs::msg::Bool();
    msg.data = allowed;
    output_scan_allowed_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "[AI_SCAN] Output tray scan %s",
        allowed ? "ENABLED" : "BLOCKED (place/full decision)");
}

void RobotLogicNode::stateMachineLoop()
{
    RCLCPP_INFO(get_logger(), "[SM] Started");
    while (state_machine_running_ && rclcpp::ok()) {
        {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            handleCurrentState();
        }
        {
            std::unique_lock<std::mutex> cv_lock(state_cv_mutex_);
            state_cv_.wait_for(cv_lock, std::chrono::milliseconds(10), [this]() {
                return state_changed_.load() || !state_machine_running_;
            });
            state_changed_ = false;
        }
    }
    RCLCPP_INFO(get_logger(), "[SM] Stopped");
}

void RobotLogicNode::handleCurrentState()
{
    if (emergency_stop_) {
        publishSystemStatus("EMERGENCY_STOP");
        return;
    }
    if (system_paused_) {
        publishSystemStatus("PAUSED");
        return;
    }
    if (!system_enabled_ && current_state_ != SystemState::IDLE) {
        transitionTo(SystemState::IDLE);
        return;
    }

    // MANUAL mode: chỉ dùng JOG và lấy vị trí, không chạy state process
    if (manual_mode_) {
        publishSystemStatus("MANUAL");
        return;
    }

    // Motion watchdog
    if (motion_in_progress_) {
        double stuck = (this->now() - motion_started_at_).seconds();
        if (stuck > MOTION_WATCHDOG_SEC) {
            RCLCPP_ERROR(get_logger(), "[WATCHDOG] Motion '%s' stuck %.0fs — clearing",
                getMotionCmd().c_str(), stuck);
            motion_in_progress_ = false;
            motion_result_ = false;
            clearMotionCmd();
            publishError("MOTION_WATCHDOG_TIMEOUT");
            return;
        }
    }

    // Motion heartbeat
    if (!checkMotionAlive(5.0) &&
        current_state_ != SystemState::IDLE &&
        current_state_ != SystemState::ERROR_MOTION_LOST)
    {
        publishError("MOTION_NODE_LOST");
        transitionTo(SystemState::ERROR_MOTION_LOST);
        return;
    }

    // START does not release INIT until the async HOME result and the live
    // joint-feedback HOME dwell are both confirmed. This also serializes the
    // mode/start/homing callbacks so they cannot enqueue duplicate HOME goals.
    if (servicePendingStartupHome())
        return;

    // A cam1 FULL decision made at the nearby scan pose must first move the
    // robot to the mechanically clear HOME zone. Hold all normal pipeline
    // transitions until that one evacuation action is verified and State 4 is
    // requested.
    if (use_ai_for_control_.load() && servicePendingOutputTrayChangeHome())
        return;

    switch (current_state_) {
        case SystemState::IDLE:                    stateIdle();                  break;
        case SystemState::INIT_CHECK:              stateInitCheck();             break;
        case SystemState::INIT_LOAD_CHAMBER_DIRECT: stateInitLoadChamberDirect(); break;
        case SystemState::INIT_REFILL_BUFFER:      stateInitRefillBuffer();      break;
        case SystemState::AI_CHECK_INPUT_TRAY:     stateAiCheckInputTray();      break;
        case SystemState::WAIT_FILLING:            stateWaitFilling();           break;
        case SystemState::TAKE_CHAMBER_TO_SCALE:   stateTakeChamberToScale();    break;
        case SystemState::LOAD_CHAMBER_FROM_BUFFER: stateLoadChamberFromBuffer(); break;
        case SystemState::WAIT_RESUME_CHOICE:      stateWaitResumeChoice();      break;
        case SystemState::WAIT_SCALE_CHOICE:       stateWaitScaleChoice();       break;
        case SystemState::REFILL_BUFFER:           stateRefillBuffer();          break;
        case SystemState::PROCESSING_SCALE:        stateProcessingScale();       break;
        case SystemState::ERROR_SCALE_TIMEOUT:     stateErrorScaleTimeout();     break;
        case SystemState::PLACE_TO_OUTPUT:         statePlaceToOutput();         break;
        case SystemState::PLACE_TO_FAIL:           statePlaceToFail();           break;
        case SystemState::LAST_BATCH_WAIT:         stateLastBatchWait();         break;
        case SystemState::ERROR_SCALE:             stateErrorScale();            break;
        case SystemState::ERROR_MOTION_LOST:       stateErrorMotionLost();       break;
        default: break;
    }
}

// ============================================================================
// STATE: IDLE
// Waits for start signal OR (after drain) both trays ready.
// ============================================================================

void RobotLogicNode::stateIdle()
{
    publishSystemStatus("IDLE");

    if (!system_enabled_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "[IDLE] Robot DISABLED — press ENABLE");
        return;
    }

    // First start
    if (system_started_) {
        system_started_ = false;
        is_first_batch_ = true;
        transitionTo(SystemState::INIT_CHECK);
        return;
    }

    // Auto-restart after drain: need both input tray and output tray ready
    if (!manual_mode_ && system_running_ && !waiting_for_new_input_.load() && !waiting_for_new_output_.load()) {
        RCLCPP_INFO(get_logger(),
            "[IDLE] ✅ Both trays ready — auto-restart from Row 1");
        is_first_batch_ = true;
        transitionTo(SystemState::INIT_CHECK);
        return;
    }

    // Log what we're waiting for
    if (system_running_) {
        bool wi = waiting_for_new_input_.load();
        bool wo = waiting_for_new_output_.load();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "[IDLE] Waiting — input_tray:%s  output_tray:%s",
            wi ? "NOT_READY" : "OK",
            wo ? "NOT_READY" : "OK");
    }
}

// ============================================================================
// STATE: INIT_CHECK
// ============================================================================

void RobotLogicNode::stateInitCheck()
{
    publishSystemStatus("INIT_CHECK");
    RCLCPP_INFO(get_logger(), "[STATE] INIT_CHECK → INIT_LOAD_CHAMBER_DIRECT");
    transitionTo(SystemState::INIT_LOAD_CHAMBER_DIRECT);
}

// ============================================================================
// STATE: INIT_LOAD_CHAMBER_DIRECT
// Picks Row X → CHAMBER (first pick of the tray cycle)
// ============================================================================

void RobotLogicNode::stateInitLoadChamberDirect()
{
    publishSystemStatus("INIT_LOAD_CHAMBER_DIRECT");

    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion...");
        return;
    }

    // ── Motion result handling ──
    if (getMotionCmd() == "INPUT_TRAY_CHAMBER" ||
        getMotionCmd() == "AI_INPUT_TRAY_CHAMBER") {
        if (motion_in_progress_) return;
        clearMotionCmd();

        if (!motion_result_) {
            if (consumeTemporaryMotionInterlockReject("INIT_LOAD", true))
                return;
            // Fail-safe: a failed/aborted pick can leave the arm above the tray.
            // Keep vision blocked so an occluded frame cannot request tray change.
            input_scan_recovery_required_ = true;
            motion_fail_count_++;
            RCLCPP_ERROR(get_logger(), "[INIT_LOAD] ❌ Motion failed (%d/3)", motion_fail_count_);
            if (motion_fail_count_ >= 3) {
                publishError("MOTION_FAILED_3X");
                motion_fail_count_ = 0;
                system_enabled_ = false;
                transitionTo(SystemState::IDLE);
            }
            return;
        }
        if (use_ai_for_control_) setInputScanAllowed(true);
        motion_fail_count_ = 0;
        chamber_has_cartridge_ = true;
        chamber_is_empty_ = false;

        advanceAutoRow();

        if (stop_after_single_motion_) {
            stop_after_single_motion_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
        if (manual_mode_) {
            transitionTo(SystemState::IDLE);
        } else {
            feed_chamber_signal_ = false;  // consumed — require new signal on next cycle
            if (use_ai_for_control_) {
                input_check_next_state_ = SystemState::INIT_REFILL_BUFFER;
                transitionTo(SystemState::AI_CHECK_INPUT_TRAY);
            } else {
                transitionTo(SystemState::INIT_REFILL_BUFFER);
            }
        }
        return;
    }

    if (cartridge_pos1_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[MUTEX][POS1] State1/2 BUSY — INIT_LOAD blocked");
        return;
    }

    if (waiting_for_new_input_.load() || !new_tray_loaded_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[INIT_LOAD] Waiting for new input tray (new_tray_loaded=false)...");
        return;
    }

    // AUTO/AI: verify S7 ON từ cartridge sensor — defense-in-depth chống
    // flag bị set giả qua simulate buttons IN_READY/PICK_INPUT.
    if (!manual_mode_ && !s7_at_robot_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[INIT_LOAD] S7 OFF — chờ cartridge thực sự đưa khay đến Robot");
        return;
    }

    // Wait for feed_chamber signal on first batch
    if (!manual_mode_ && is_first_batch_ && !feed_chamber_signal_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "[INIT_LOAD] Waiting for feed_chamber signal...");
        return;
    }

    // ── Determine row to pick ──
    int row = -1;
    if (!use_ai_for_control_) {
        row = getNextAutoRow();
        if (row == ROW_UNSET || row == -1) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "[INIT_LOAD] Waiting for starting row selection (GUI)...");
            return;
        }
    } else {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        row = selected_input_row_;
        selected_input_row_ = -1;
        if (row <= 0) {
            for (size_t i = 0; i < row_full_.size(); ++i)
                if (row_full_[i]) { row = (int)i + 1; break; }
        }
        if (row <= 0) {
            // selected_row co the tam thoi -1 trong luc RowFilter warm-up hoac
            // giua hai callback. Quyen ket luan khay het thuoc ve topic
            // /vision/input_tray/empty da debounce; tai day chi cho vision.
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                "[INIT_LOAD] AI: chua co row ready — cho vision xac nhan");
            return;
        }
    }

    // Final edge mutex: State1/2 may start after the earlier check but before
    // this action send. Do not consume the row until Pos1 is actually free.
    if (cartridge_pos1_busy_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[INTERLOCK] Cartridge became BUSY — INPUT_TRAY_CHAMBER not sent");
        return;
    }
    RCLCPP_INFO(get_logger(), "[INIT_LOAD] 🤖 Pick Row %d → CHAMBER [ASYNC]", row);
    if (use_ai_for_control_) setInputScanAllowed(false);
    sendMotionActionAsync(
        use_ai_for_control_ ? "AI_INPUT_TRAY_CHAMBER" : "INPUT_TRAY_CHAMBER", row);

    // Row counter is advanced ONLY after motion succeeds (in the motion_result_ handler block above)
}

// ============================================================================
// STATE: INIT_REFILL_BUFFER
// Picks Row X+1 → BUFFER (second pick, init only)
// ============================================================================

void RobotLogicNode::stateInitRefillBuffer()
{
    publishSystemStatus("INIT_REFILL_BUFFER");

    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion...");
        return;
    }

    // ── Motion result handling ──
    if (getMotionCmd() == "INIT_INPUT_TRAY_BUFFER" ||
        getMotionCmd() == "AI_INIT_INPUT_TRAY_BUFFER") {
        if (motion_in_progress_) return;
        clearMotionCmd();

        if (!motion_result_) {
            if (consumeTemporaryMotionInterlockReject("INIT_REFILL", true))
                return;
            // The arm position is unknown after failure; do not trust camera.
            input_scan_recovery_required_ = true;
            motion_fail_count_++;
            RCLCPP_ERROR(get_logger(), "[INIT_REFILL] ❌ Motion failed (%d/3)", motion_fail_count_);
            if (motion_fail_count_ >= 3) {
                publishError("MOTION_FAILED_3X");
                motion_fail_count_ = 0;
                system_enabled_ = false;
                transitionTo(SystemState::IDLE);
            }
            return;
        }
        if (use_ai_for_control_) setInputScanAllowed(true);

        buffer_is_empty_ = false;
        refill_pending_ = false;
        is_first_batch_ = false;

        advanceAutoRow();

        if (stop_after_single_motion_) {
            stop_after_single_motion_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }
        if (manual_mode_) {
            transitionTo(SystemState::IDLE);
        } else {
            if (use_ai_for_control_) {
                input_check_next_state_ = SystemState::WAIT_FILLING;
                transitionTo(SystemState::AI_CHECK_INPUT_TRAY);
            } else {
                transitionTo(SystemState::WAIT_FILLING);
            }
        }
        return;
    }

    if (cartridge_pos1_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[MUTEX][POS1] State1/2 BUSY — INIT_REFILL blocked");
        return;
    }

    // If currently waiting for tray and no row available — skip buffer init
    // and go straight to WAIT_FILLING (chamber already loaded)
    if (waiting_for_new_input_.load() || !new_tray_loaded_.load()) {
        RCLCPP_WARN(get_logger(),
            "[INIT_REFILL] No input tray for buffer — skip, go WAIT_FILLING");
        buffer_is_empty_ = true;
        is_first_batch_ = false;
        transitionTo(SystemState::WAIT_FILLING);
        return;
    }

    // ── Determine row to pick ──
    int row = -1;
    if (!use_ai_for_control_) {
        row = getNextAutoRow();
        if (row == -1) {
            // No tray available: go to WAIT_FILLING with empty buffer
            RCLCPP_WARN(get_logger(),
                "[INIT_REFILL] Tray not ready — skip buffer init, go WAIT_FILLING");
            buffer_is_empty_ = true;
            is_first_batch_ = false;
            transitionTo(SystemState::WAIT_FILLING);
            return;
        }
    } else {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        for (size_t i = 0; i < row_full_.size(); ++i)
            if (row_full_[i]) { row = (int)i + 1; break; }
        if (row <= 0) {
            // Sau input pick, vision gate mo lai va can frame moi de quyet
            // dinh READY/EMPTY. Khong duoc suy "tam thoi chua co row" thanh
            // het khay; /vision/input_tray/empty se debounce va set waiting.
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[INIT_REFILL] AI: waiting for fresh row/EMPTY decision...");
            return;
        }
    }

    if (cartridge_pos1_busy_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[INTERLOCK] Cartridge became BUSY — INIT_INPUT_TRAY_BUFFER not sent");
        return;
    }
    RCLCPP_INFO(get_logger(), "[INIT_REFILL] 🤖 Pick Row %d → BUFFER [ASYNC]", row);
    if (use_ai_for_control_) setInputScanAllowed(false);
    sendMotionActionAsync(
        use_ai_for_control_ ? "AI_INIT_INPUT_TRAY_BUFFER" : "INIT_INPUT_TRAY_BUFFER", row);
}

// ============================================================================
// STATE: AI_CHECK_INPUT_TRAY
// Robot is confirmed at HOME/index 0 after init or normal refill. Hold the
// pipeline here until Vision publishes a fresh READY row or confirms EMPTY.
// ============================================================================

void RobotLogicNode::stateAiCheckInputTray()
{
    publishSystemStatus("AI_CHECK_INPUT_TRAY");

    if (!use_ai_for_control_) {
        transitionTo(input_check_next_state_);
        return;
    }
    if (waiting_for_new_input_.load()) {
        // EMPTY after the first chamber pick means there is no second row for
        // INIT_REFILL_BUFFER.  Preserve the loaded chamber and start filling.
        const auto next =
            (input_check_next_state_ == SystemState::INIT_REFILL_BUFFER)
                ? SystemState::WAIT_FILLING
                : input_check_next_state_;
        RCLCPP_WARN(get_logger(),
            "[AI_CHECK_TRAY] EMPTY confirmed at verified camera checkpoint -> tray change requested");
        transitionTo(next);
        return;
    }

    if (input_scan_recovery_required_.load() || !cam0DecisionWindowReady()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[AI_CHECK_TRAY] Waiting for HOME/index 0 + 1.5s clean cam0 checkpoint");
        return;
    }

    bool has_ready_row = false;
    {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        has_ready_row = std::any_of(
            row_full_.begin(), row_full_.end(), [](bool ready) { return ready; });
    }

    if (has_ready_row) {
        RCLCPP_INFO(get_logger(),
            "[AI_CHECK_TRAY] Fresh READY decision received -> continue pipeline");
        transitionTo(input_check_next_state_);
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[AI_CHECK_TRAY] At index 0 — waiting for fresh READY/EMPTY decision...");
}

// ============================================================================
// STATE: WAIT_FILLING
//
// Waits for fill_done from chamber.
// Also checks tray flags — if no tray available, we're in drain mode
// (no refill after LOAD_CHAMBER_FROM_BUFFER).
// ============================================================================

void RobotLogicNode::stateWaitFilling()
{
    publishSystemStatus("WAIT_FILLING");

    if (manual_mode_) {
        transitionTo(SystemState::IDLE);
        return;
    }

    // Pipeline fully drained: drain confirmation ends the batch immediately.
    // If a new tray arrives after drain, newTrayCallback clears the drain flag and
    // this state restarts the normal cycle from the new tray.
    bool pipeline_empty = !chamber_has_cartridge_ && buffer_is_empty_ && !scale_has_cartridge_;
    if (pipeline_empty) {
        if (!waiting_for_new_input_.load() && new_tray_loaded_.load()) {
            cartridge_drain_confirmed_ = false;
            RCLCPP_INFO(get_logger(),
                "[WAIT_FILLING] Pipeline empty + NEW tray available → INIT_LOAD_CHAMBER_DIRECT");
            transitionTo(SystemState::INIT_LOAD_CHAMBER_DIRECT);
            return;
        }

        if (waiting_for_new_input_.load()) {
            if (cartridge_drain_confirmed_.load()) {
                RCLCPP_INFO(get_logger(),
                    "[WAIT_FILLING] Pipeline drained + cartridge DRAIN confirmed → IDLE");
                if (requestOutputTrayChangeForDrain("WAIT_FILLING"))
                    transitionTo(SystemState::IDLE);
                return;
            }

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "[WAIT_FILLING] Pipeline empty, waiting for cartridge drain confirmation or new tray...");
            return;
        }
    }

    // Chamber commands outrank buffer refill.  If the chamber is empty and a
    // prepared buffer is waiting, honor feed_chamber before any pending refill.
    if (!chamber_has_cartridge_ && !buffer_is_empty_ && feed_chamber_signal_.load()) {
        RCLCPP_INFO(get_logger(),
            "[PRIORITY] feed_chamber before refill_pending -> LOAD_CHAMBER_FROM_BUFFER");
        transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
        return;
    }

    if (!chamber_has_cartridge_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[WAIT_FILLING] Chamber empty — nothing to fill");
        return;
    }

    if (fill_done_.load()) {
        const bool scale_result_pending = hasPendingScaleResults();
        if (scale_has_cartridge_ || scale_result_pending) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[PRIORITY] fill_done latched but scale not ready "
                "(occupied=%d result_pending=%d) — chamber transfer blocked",
                scale_has_cartridge_, scale_result_pending);
            return;
        }
        RCLCPP_INFO(get_logger(),
            "[PRIORITY] fill_done before refill_pending -> TAKE_CHAMBER_TO_SCALE");
        fill_done_ = false;
        transitionTo(SystemState::TAKE_CHAMBER_TO_SCALE);
        return;
    }

    // No chamber command is pending: refill the empty buffer opportunistically.
    if (buffer_is_empty_ && refill_pending_.load() &&
        !waiting_for_new_input_.load() && new_tray_loaded_.load())
    {
        cartridge_drain_confirmed_ = false;
        RCLCPP_INFO(get_logger(),
            "[PRIORITY] No chamber command -> consume refill_pending");
        transitionTo(SystemState::REFILL_BUFFER);
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[WAIT_FILLING] Waiting (fill_done=0 refill_pending=%d)",
        refill_pending_.load());
}

// ============================================================================
// STATE: TAKE_CHAMBER_TO_SCALE
// CHAMBER → SCALE (8 cartridges)
// Then → LOAD_CHAMBER_FROM_BUFFER (if buffer has cartridge)
//      → PROCESSING_SCALE (if buffer empty — drain mode)
// ============================================================================

void RobotLogicNode::stateTakeChamberToScale()
{
    publishSystemStatus("TAKE_CHAMBER_TO_SCALE");

    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion...");
        return;
    }

    if (getMotionCmd() == "CHAMBER_SCALE") {
        if (motion_in_progress_) return;
        clearMotionCmd();

        if (!motion_result_) {
            if (consumeTemporaryMotionInterlockReject("CHAMBER_SCALE", false))
                return;
            chamber_has_cartridge_ = true;  // assume didn't move
            return;
        }

        chamber_has_cartridge_ = false;
        chamber_is_empty_      = true;
        scale_has_cartridge_   = true;
        scale_wait_start_      = this->now();
        fill_done_             = false;  // reset to avoid stale trigger
        ++cartridge_counter_;

        RCLCPP_INFO(get_logger(), "[SCALE] 🏷️ Cartridge #%d on scale",
            cartridge_counter_.load());

        if (!buffer_is_empty_) {
            // Buffer has cartridge → load it into chamber next
            RCLCPP_INFO(get_logger(),
                "[PIPELINE] Buffer not empty → LOAD_CHAMBER_FROM_BUFFER");
            transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
        } else {
            // Buffer empty (drain mode) — go process scale, no chamber loading
            RCLCPP_WARN(get_logger(),
                "[PIPELINE] Buffer empty (drain) → PROCESSING_SCALE directly");
            transitionTo(SystemState::PROCESSING_SCALE);
        }
        return;
    }

    // Internal transfer: does not enter the shared input/output tray zones, so
    // it remains allowed while the cartridge state machine is active.
    const bool scale_result_pending = hasPendingScaleResults();
    if (scale_has_cartridge_ || scale_result_pending) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[SCALE] CHAMBER_SCALE blocked — scale must be empty and have no "
            "pending result (occupied=%d result_pending=%d)",
            scale_has_cartridge_, scale_result_pending);
        return;
    }
    RCLCPP_INFO(get_logger(), "[SCALE] 🤖 CHAMBER → SCALE [ASYNC]");
    sendMotionActionAsync("CHAMBER_SCALE");
}

// ============================================================================
// STATE: LOAD_CHAMBER_FROM_BUFFER
// BUFFER → CHAMBER
// Then:
//   - If tray available (!waiting_for_new_input_) → REFILL_BUFFER
//   - If no tray (drain mode)                     → PROCESSING_SCALE (wait for result)
// ============================================================================

void RobotLogicNode::stateLoadChamberFromBuffer()
{
    publishSystemStatus("LOAD_CHAMBER_FROM_BUFFER");

    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion...");
        return;
    }

    if (getMotionCmd() == "BUFFER_CHAMBER") {
        if (motion_in_progress_) return;
        clearMotionCmd();

        if (!motion_result_) {
            if (consumeTemporaryMotionInterlockReject("BUFFER_CHAMBER", false))
                return;
            RCLCPP_ERROR(get_logger(), "[LOAD_BUFFER] ❌ Motion failed");
            return;
        }

        buffer_is_empty_       = true;
        chamber_has_cartridge_ = true;
        chamber_is_empty_      = false;
        refill_pending_        = true;

        RCLCPP_INFO(get_logger(),
            "[LOAD_BUFFER] ✅ Buffer → Chamber done. Chamber filling started.");

        if (manual_mode_) {
            transitionTo(SystemState::IDLE);
            return;
        }

        feed_chamber_signal_ = false;  // consumed — require new signal on next cycle

        if (refill_pending_.load() && !waiting_for_new_input_.load()) {
            // BUFFER_CHAMBER now finishes at HOME/index 0. REFILL_BUFFER may
            // enter immediately and will naturally wait through the cam0 pose
            // dwell/cooldown until a fresh row decision arrives.
            RCLCPP_INFO(get_logger(),
                "[PIPELINE] Tray available at HOME checkpoint → REFILL_BUFFER");
            transitionTo(SystemState::REFILL_BUFFER);
        } else if (scale_has_cartridge_) {
            // Drain mode + scale còn cartridge cũ → đợi scale result
            RCLCPP_WARN(get_logger(),
                "[PIPELINE] Drain mode — no refill → PROCESSING_SCALE (wait scale + fill)");
            transitionTo(SystemState::PROCESSING_SCALE);
        } else {
            // Drain mode + scale empty (vd resume sau timeout, scale đã drained) →
            // chỉ còn việc đợi chamber fill_done.
            RCLCPP_INFO(get_logger(),
                "[PIPELINE] Drain mode + scale empty → WAIT_FILLING");
            transitionTo(SystemState::WAIT_FILLING);
        }
        return;
    }

    // Gate: feed_chamber signal required before each BUFFER→CHAMBER load.
    // Fill machine publishes liên tục khi đang fill; tạm dừng (vd thay mực) →
    // signal off → robot không đặt khay mới vào chamber. Manual mode bypass:
    // operator nhấn nút simulate feed_chamber để chạy.
    // Timeout 150s: skip BUFFER→CHAMBER, drain SCALE rồi vào WAIT_RESUME_CHOICE
    // để operator chọn cách resume.
    if (!manual_mode_ && !feed_chamber_signal_) {
        if (!feed_chamber_wait_active_) {
            feed_chamber_wait_start_  = this->now();
            feed_chamber_wait_active_ = true;
        }
        double elapsed = (this->now() - feed_chamber_wait_start_).seconds();
        if (LOAD_BUFFER_FEED_TIMEOUT_S > 0.0 && elapsed > LOAD_BUFFER_FEED_TIMEOUT_S) {
            RCLCPP_WARN(get_logger(),
                "[LOAD_BUFFER] ⏸ Cho feed_chamber %.0fs — hoi operator cach xu ly",
                elapsed);
            // KHONG dat skipped_buffer_load_ va KHONG di qua PROCESSING_SCALE:
            // truoc day duong do xa SCALE TRUOC khi hoi, tuc tu quyet dinh thay
            // operator roi moi bao. Gio vao thang trang thai hoi.
            feed_chamber_wait_active_  = false;
            transitionTo(SystemState::WAIT_RESUME_CHOICE);
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "[LOAD_BUFFER] ⏸ Cho /revpi/feed_chamber tu may Fill (%.0fs) — dung yen, "
            "se tu chay tiep khi topic tro lai", elapsed);
        return;
    }
    feed_chamber_wait_active_ = false;  // signal arrived → reset timer

    // Internal transfer: intentionally not gated by /cartridge/busy.
    RCLCPP_INFO(get_logger(), "[LOAD_BUFFER] 🤖 BUFFER → CHAMBER [ASYNC]");
    sendMotionActionAsync("BUFFER_CHAMBER");
}

// ============================================================================
// STATE: WAIT_RESUME_CHOICE
// Entered after PLACE_TO_OUTPUT/FAIL when LOAD_CHAMBER_FROM_BUFFER was skipped
// due to feed_chamber timeout. Operator chooses via GUI popup:
//   - goto INIT_LOAD_CHAMBER_DIRECT  (đã xử lý buffer thực tế → start fresh)
//   - goto LOAD_CHAMBER_FROM_BUFFER  (retry với buffer hiện tại)
// STOP/IDLE override luôn khả dụng qua emergency_stop / reset_state.
// ============================================================================

void RobotLogicNode::stateWaitResumeChoice()
{
    publishSystemStatus("WAIT_RESUME_CHOICE");
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "[RESUME] ⏸ Waiting operator: INIT_LOAD_CHAMBER_DIRECT or LOAD_CHAMBER_FROM_BUFFER");
}

// ============================================================================
// STATE: WAIT_SCALE_CHOICE
// Entered when PROCESSING_SCALE has not received any loadcell topic for 150s.
// Operator chooses via GUI popup:
//   - WAIT_FILLING     (đã lấy cartridge ra → back to wait fill cycle)
//   - PLACE_TO_OUTPUT  (force place as PASS)
//   - PLACE_TO_FAIL    (force place as FAIL)
// ============================================================================

void RobotLogicNode::stateWaitScaleChoice()
{
    publishSystemStatus("WAIT_SCALE_CHOICE");
    if (ignore_scale_) {
        RCLCPP_WARN(get_logger(), "[SCALE_CHOICE] IGNORE MODE: force PASS");
        forceScalePass("WAIT_SCALE_CHOICE");
        transitionTo(SystemState::PLACE_TO_OUTPUT);
        return;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "[SCALE_CHOICE] ⏸ No loadcell 150s — waiting operator: WAIT_FILLING / PLACE_TO_OUTPUT / PLACE_TO_FAIL");
}

// ============================================================================
// STATE: REFILL_BUFFER
// INPUT_TRAY (next row) → BUFFER
// Called after LOAD_CHAMBER_FROM_BUFFER, while chamber is filling.
// After done → PROCESSING_SCALE (wait for scale result + fill_done)
// ============================================================================

void RobotLogicNode::stateRefillBuffer()
{
    publishSystemStatus("REFILL_BUFFER");

    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion...");
        return;
    }

    // ── Motion result handling ──
    if (getMotionCmd() == "INPUT_TRAY_BUFFER" ||
        getMotionCmd() == "AI_INPUT_TRAY_BUFFER") {
        if (motion_in_progress_) return;
        clearMotionCmd();

        if (!motion_result_) {
            if (consumeTemporaryMotionInterlockReject("REFILL", true))
                return;
            // Keep scan locked until an explicit recovery/HOME.  Continuing
            // vision here previously converted robot occlusion into EMPTY.
            input_scan_recovery_required_ = true;
            motion_fail_count_++;
            RCLCPP_ERROR(get_logger(), "[REFILL] ❌ Motion failed (%d/3)", motion_fail_count_);
            if (motion_fail_count_ >= 3) {
                publishError("MOTION_FAILED_3X");
                motion_fail_count_ = 0;
                system_enabled_ = false;
                transitionTo(SystemState::IDLE);
                return;
            }
            // Don't crash — go process scale anyway
        } else {
            if (use_ai_for_control_) {
                setInputScanAllowed(true);
                // AI_INPUT_TRAY_BUFFER has returned HOME/index 0. Open both
                // cameras there while scale processing continues;
                // PLACE_TO_OUTPUT then consumes a fresh cam1 frame.
                output_prescan_home_done_ = true;
                output_prescan_zone_verified_ = false;
                setOutputScanAllowed(false);
                setOutputScanAllowed(true);
                RCLCPP_INFO(get_logger(),
                    "[SAFETY][CAM0_CAM1] Refill finished at HOME/index 0 — "
                    "checking input rows and collecting output snapshot in parallel");
            }
            buffer_is_empty_ = false;
            refill_pending_ = false;
            RCLCPP_INFO(get_logger(), "[BUFFER] ✅ Buffer refilled");

            advanceAutoRow();
        }

        if (stop_after_single_motion_) {
            stop_after_single_motion_ = false;
            transitionTo(SystemState::IDLE);
            return;
        }

        // After refill:
        //  - scale_has_cartridge_ true → wait scale result (normal pipelined cycle)
        //  - scale empty (vd resume sau timeout, scale đã drained) → wait fill_done
        if (scale_has_cartridge_) {
            RCLCPP_INFO(get_logger(), "[PIPELINE] Refill done → PROCESSING_SCALE");
            if (use_ai_for_control_) {
                input_check_next_state_ = SystemState::PROCESSING_SCALE;
                transitionTo(SystemState::AI_CHECK_INPUT_TRAY);
            } else {
                transitionTo(SystemState::PROCESSING_SCALE);
            }
        } else {
            RCLCPP_INFO(get_logger(),
                "[PIPELINE] Refill done, scale empty → WAIT_FILLING (đợi chamber fill_done)");
            if (use_ai_for_control_) {
                input_check_next_state_ = SystemState::WAIT_FILLING;
                transitionTo(SystemState::AI_CHECK_INPUT_TRAY);
            } else {
                transitionTo(SystemState::WAIT_FILLING);
            }
        }
        return;
    }

    if (cartridge_pos1_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[MUTEX][POS1] State1/2 BUSY — REFILL_BUFFER blocked");
        return;
    }

    // ── Check if we still have rows ──
    if (waiting_for_new_input_.load() || !new_tray_loaded_.load()) {
        if (use_ai_for_control_.load() && input_tray_empty_.load()) {
            // This is a positive Vision result, not an all-false cache:
            // Vision only confirms EMPTY after 15 clean HOME frames with the
            // physical tray present and no row containing exactly 8 parts.
            // The tray-change handshake can continue in parallel while the
            // already-loaded scale/chamber pipeline keeps moving.
            buffer_is_empty_ = true;
            if (scale_has_cartridge_) {
                RCLCPP_WARN(get_logger(),
                    "[REFILL] Fresh cam0 EMPTY confirmed — no valid row; "
                    "skip buffer refill → PROCESSING_SCALE");
                transitionTo(SystemState::PROCESSING_SCALE);
            } else {
                RCLCPP_WARN(get_logger(),
                    "[REFILL] Fresh cam0 EMPTY confirmed — no valid row and "
                    "scale empty; skip buffer refill → WAIT_FILLING");
                transitionTo(SystemState::WAIT_FILLING);
            }
            return;
        }
        if (cartridge_drain_confirmed_.load()) {
            // Cartridge confirmed no S1/S2/S3 tray after S2 -> S1 handoff — skip refill -> drain.
            RCLCPP_WARN(get_logger(),
                "[REFILL] Cartridge DRAIN confirmed — skip refill → PROCESSING_SCALE");
            transitionTo(SystemState::PROCESSING_SCALE);
            return;
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                "[REFILL] Waiting for cartridge S2→S1 handoff (empty=%d drain=%d new_tray=%d)...",
                input_trays_empty_.load(), cartridge_drain_confirmed_.load(), new_tray_loaded_.load());
            return;
        }
    }

    // ── Get next row ──
    int row = -1;
    if (!use_ai_for_control_) {
        row = getNextAutoRow();
        if (row == -1) {
            RCLCPP_WARN(get_logger(), "[REFILL] No row available → PROCESSING_SCALE");
            transitionTo(SystemState::PROCESSING_SCALE);
            return;
        }
    } else {
        std::lock_guard<std::mutex> lock(row_selection_mutex_);
        for (size_t i = 0; i < row_full_.size(); ++i)
            if (row_full_[i]) { row = (int)i + 1; break; }
        if (row <= 0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[REFILL] AI: waiting for fresh row/EMPTY decision...");
            return;
        }
    }

    if (cartridge_pos1_busy_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[INTERLOCK] Cartridge became BUSY — INPUT_TRAY_BUFFER not sent");
        return;
    }
    RCLCPP_INFO(get_logger(), "[REFILL] 🤖 Pick Row %d → BUFFER [ASYNC]", row);
    if (use_ai_for_control_) setInputScanAllowed(false);
    sendMotionActionAsync(
        use_ai_for_control_ ? "AI_INPUT_TRAY_BUFFER" : "INPUT_TRAY_BUFFER", row);

    // Row counter is advanced ONLY after motion succeeds
}

// ============================================================================
// STATE: PROCESSING_SCALE
// Waits for scale result, then routes to PLACE_TO_OUTPUT or PLACE_TO_FAIL.
// This state is entered both from REFILL_BUFFER and from LOAD_CHAMBER_FROM_BUFFER
// (drain mode). It does NOT start any motion — just waits.
// ============================================================================

void RobotLogicNode::stateProcessingScale()
{
    publishSystemStatus("PROCESSING_SCALE");

    if (manual_mode_) {
        transitionTo(SystemState::IDLE);
        return;
    }

    if (ignore_scale_) {
        RCLCPP_WARN(get_logger(), "[SCALE] IGNORE MODE: bypass scale -> PASS");
        forceScalePass("PROCESSING_SCALE");
        transitionTo(SystemState::PLACE_TO_OUTPUT);
        return;
    }

    // Bypass for simulation
    if (simulate_scale_ || force_pass_) {
        RCLCPP_INFO(get_logger(), "[SCALE] Bypass → Force PASS");
        stored_scale_result_.store(true);
        transitionTo(SystemState::PLACE_TO_OUTPUT);
        return;
    }

    bool result_pass;
    int  cartridge_id;
    if (processNextScaleResult(result_pass, cartridge_id)) {
        stored_scale_result_.store(result_pass);
        RCLCPP_INFO(get_logger(), "[SCALE] ✅ Cartridge #%d: %s",
            cartridge_id, result_pass ? "PASS" : "FAIL");
        if (result_pass)
            transitionTo(SystemState::PLACE_TO_OUTPUT);
        else
            transitionTo(SystemState::PLACE_TO_FAIL);
        return;
    }

    double elapsed = (this->now() - scale_wait_start_).seconds();

    if (elapsed > SCALE_TOPIC_TIMEOUT_S) {
        RCLCPP_WARN(get_logger(),
            "[SCALE] ⚠ No loadcell topic for %.0fs → WAIT_SCALE_CHOICE", elapsed);
        transitionTo(SystemState::WAIT_SCALE_CHOICE);
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[SCALE] ⏳ Waiting for result... (%.1fs / %.0fs)", elapsed, SCALE_TOPIC_TIMEOUT_S);
}

// ============================================================================
// STATE: ERROR_SCALE_TIMEOUT
// ============================================================================

void RobotLogicNode::stateErrorScaleTimeout()
{
    publishSystemStatus("ERROR_SCALE_TIMEOUT");

    if (ignore_scale_) {
        RCLCPP_WARN(get_logger(), "[ERROR_SCALE_TIMEOUT] IGNORE MODE: force PASS");
        forceScalePass("ERROR_SCALE_TIMEOUT");
        transitionTo(SystemState::PLACE_TO_OUTPUT);
        return;
    }

    if (scale_result_received_) {
        RCLCPP_INFO(get_logger(), "[SCALE] Result received after timeout — resuming");
        if (stored_scale_result_)
            transitionTo(SystemState::PLACE_TO_OUTPUT);
        else
            transitionTo(SystemState::PLACE_TO_FAIL);
        scale_result_received_ = false;
        stored_scale_result_.store(false);
        return;
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[ERROR] Waiting for scale result...");

    if (!system_enabled_) transitionTo(SystemState::IDLE);
}

void RobotLogicNode::routeAfterOutputPlacement()
{
    if (exitAfterScalePlaceRequested("PLACE_TO_OUTPUT"))
        return;
    // Skipped BUFFER→CHAMBER (feed_chamber timeout) → wait operator choice.
    if (skipped_buffer_load_.load()) {
        RCLCPP_WARN(get_logger(),
            "[PLACE_OUT] ⚠️ skipped_buffer_load active → WAIT_RESUME_CHOICE");
        transitionTo(SystemState::WAIT_RESUME_CHOICE);
        return;
    }

    // Commands originating from the chamber always win over a pending refill.
    // The refill flag remains latched and will be consumed after chamber work.
    if (chamber_has_cartridge_ && fill_done_.load()) {
        const bool scale_result_pending = hasPendingScaleResults();
        if (scale_has_cartridge_ || scale_result_pending) {
            RCLCPP_WARN(get_logger(),
                "[PRIORITY][PLACE_OUT] fill_done held — scale not ready "
                "(occupied=%d result_pending=%d)",
                scale_has_cartridge_, scale_result_pending);
            transitionTo(SystemState::WAIT_FILLING);
            return;
        }
        RCLCPP_INFO(get_logger(),
            "[PRIORITY][PLACE_OUT] fill_done before refill_pending -> TAKE_CHAMBER_TO_SCALE");
        fill_done_ = false;
        transitionTo(SystemState::TAKE_CHAMBER_TO_SCALE);
        return;
    }
    if (!chamber_has_cartridge_ && !buffer_is_empty_ && feed_chamber_signal_.load()) {
        RCLCPP_INFO(get_logger(),
            "[PRIORITY][PLACE_OUT] feed_chamber before refill_pending -> LOAD_CHAMBER_FROM_BUFFER");
        transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
        return;
    }

    const bool pipeline_empty = !chamber_has_cartridge_ && buffer_is_empty_;
    if (pipeline_empty) {
        if (waiting_for_new_input_.load()) {
            if (cartridge_drain_confirmed_.load()) {
                RCLCPP_WARN(get_logger(),
                    "[PIPELINE] 📦 Pipeline drained + Cartridge DRAIN confirmed");
                if (requestOutputTrayChangeForDrain("PLACE_TO_OUTPUT"))
                    transitionTo(SystemState::IDLE);
                else
                    transitionTo(SystemState::WAIT_FILLING);
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "[PIPELINE] Pipeline drained but Cartridge has not confirmed DRAIN — WAIT_FILLING");
                transitionTo(SystemState::WAIT_FILLING);
            }
        } else {
            RCLCPP_INFO(get_logger(),
                "[PIPELINE] Pipeline drained + new input tray available → INIT_LOAD_CHAMBER_DIRECT");
            transitionTo(SystemState::INIT_LOAD_CHAMBER_DIRECT);
        }
        return;
    }

    if (refill_pending_.load() && chamber_has_cartridge_ && buffer_is_empty_ &&
        !waiting_for_new_input_.load() && new_tray_loaded_.load()) {
        cartridge_drain_confirmed_ = false;
        RCLCPP_INFO(get_logger(),
            "[PRIORITY][PLACE_OUT] Chamber commands idle -> consume refill_pending");
        transitionTo(SystemState::REFILL_BUFFER);
        return;
    }

    transitionTo(SystemState::WAIT_FILLING);
}

// ============================================================================
// STATE: PLACE_TO_OUTPUT
// SCALE → OUTPUT slot
// After done:
//   - Check if pipeline is drained AND waiting_for_new_input_ → pub done_tray_output → IDLE
//   - Otherwise → WAIT_FILLING
// ============================================================================

void RobotLogicNode::statePlaceToOutput()
{
    publishSystemStatus("PLACE_TO_OUTPUT");

    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion...");
        return;
    }

    // ── Motion result handling ──
    if (getMotionCmd() == "OUTPUT_SCAN_HOME") {
        if (motion_in_progress_) return;
        clearMotionCmd();

        if (!motion_result_) {
            if (consumeTemporaryMotionInterlockReject("OUTPUT_SCAN_HOME", false)) {
                output_prescan_zone_verified_ = false;
                setOutputScanAllowed(false);
                return;
            }
            output_scan_recovery_required_ = true;
            output_prescan_home_done_ = false;
            output_prescan_zone_verified_ = false;
            setOutputScanAllowed(false);
            publishError("OUTPUT_SCAN_HOME_FAILED");
            RCLCPP_ERROR(get_logger(),
                "[SAFETY][CAM1] Pre-scan HOME failed — output placement remains locked");
            return;
        }

        output_prescan_home_done_ = true;
        output_prescan_zone_verified_ = false;
        RCLCPP_INFO(get_logger(),
            "[SAFETY][CAM1] Pre-scan HOME action complete — waiting for live "
            "joint feedback and 0.5 s stable-zone confirmation");
        return;
    }

    if (getMotionCmd() == "SCALE_OUTPUT") {
        if (motion_in_progress_) return;
        clearMotionCmd();

        int placed_slot = async_slot_.load();

        if (!motion_result_) {
            if (last_motion_interlock_rejected_.exchange(false)) {
                // Motion Executor vetoed before any Dobot/RobotMode primitive.
                // This is a temporary concurrent Cartridge lock, not an unknown
                // robot pose; wait and take a new frame instead of forcing HOME.
                // Goal admission briefly closes the camera-view gate, but no
                // Dobot primitive ran. Preserve the nearby scan checkpoint and
                // wait for its 0.5 s re-dwell instead of taking a long HOME.
                output_prescan_home_done_ = true;
                output_prescan_zone_verified_ = false;
                output_place_snapshot_requested_ = false;
                setOutputScanAllowed(false);
                RCLCPP_WARN(get_logger(),
                    "[PLACE] SCALE_OUTPUT admission vetoed by live Cartridge "
                    "interlock — no robot motion occurred; waiting to rescan/retry");
                return;
            }
            // Failed place may leave the arm covering output tray/slots.
            // Never reopen output decisions from an unknown robot pose.
            output_scan_recovery_required_ = true;
            output_prescan_home_done_ = false;
            output_prescan_zone_verified_ = false;
            setOutputScanAllowed(false);
            RCLCPP_ERROR(get_logger(), "[PLACE] ❌ SCALE_OUTPUT motion failed");
            return;
        }
        output_scan_recovery_required_ = false;
        output_prescan_home_done_ = false;
        output_prescan_zone_verified_ = false;
        if (use_ai_for_control_) setOutputScanAllowed(false);

        scale_has_cartridge_ = false;
        tray_count_++;
        RCLCPP_INFO(get_logger(), "[STATS] ✅ Tray count: %d (slot %d)", tray_count_.load(), placed_slot);

        // Advance slot counter
        current_auto_slot_ = placed_slot + 1;
        {
            std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
            selected_output_slot_ = SLOT_UNSET;
        }

        // Last output slot reached → output tray full
        if (placed_slot >= TOTAL_OUTPUT_SLOTS) {
            current_auto_slot_ = 1;
            if (!use_ai_for_control_) {
                // AUTO/MANUAL: Robot tự quyết định thay khay
                RCLCPP_WARN(get_logger(), "[OUTPUT] Slot %d reached — output tray full (AUTO mode)", TOTAL_OUTPUT_SLOTS);
                publishDoneTrayOutput();
            }
            // AI mode: Camera toàn quyền — không can thiệp
        }

        // AUTO: bao GUI slot KE TIEP se dat ngay sau khi dat xong, de den O
        // sang dung cho trong suot thoi gian cho fill (~100s). Truoc day chi
        // publish luc bat dau motion -> GUI sang slot VUA dat thay vi slot toi.
        if (!use_ai_for_control_ && selected_slot_pub_) {
            auto next_msg = std_msgs::msg::Int32();
            next_msg.data = current_auto_slot_;
            selected_slot_pub_->publish(next_msg);
        }

        if (use_ai_for_control_.load()) {
            // executeScaleOutput() already ends at index 0. Keep this state
            // active and use that natural HOME dwell as the second cam0+cam1
            // checkpoint after the newly placed cartridge.
            output_prescan_home_done_ = true;
            output_prescan_zone_verified_ = false;
            post_place_output_scan_started_ = false;
            post_place_output_check_pending_ = true;
            if (!waiting_for_new_input_.load() && new_tray_loaded_.load())
                setInputScanAllowed(true);
            setOutputScanAllowed(false);
            RCLCPP_INFO(get_logger(),
                "[POST_PLACE][CAM0_CAM1] SCALE_OUTPUT returned HOME — check input "
                "rows and verify output FULL/non-FULL (no extra motion)");
            return;
        }

        routeAfterOutputPlacement();
        return;
    }

    if (post_place_output_check_pending_.load()) {
        // The FULL coordinator runs before this state handler. Once it has
        // requested State 4, finish the normal pipeline routing while the
        // robot remains mechanically clear at HOME.
        if (waiting_for_new_output_.load()) {
            post_place_output_check_pending_ = false;
            post_place_output_scan_started_ = false;
            RCLCPP_WARN(get_logger(),
                "[POST_PLACE][CAM1] FULL confirmed at HOME — State 4 requested");
            routeAfterOutputPlacement();
            return;
        }
        if (!visionHeartbeatFresh()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                "[POST_PLACE][CAM1] Waiting — vision heartbeat missing/stale; robot held at HOME");
            return;
        }
        if (!mechanicalOutputZoneClearFresh() || !cam1ViewClearFresh()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[POST_PLACE][CAM0_CAM1] Waiting for stable HOME and clean camera window");
            return;
        }
        if (!post_place_output_scan_started_.exchange(true)) {
            setOutputScanAllowed(false);
            setOutputScanAllowed(true);
            RCLCPP_INFO(get_logger(),
                "[POST_PLACE][CAM0_CAM1] HOME verified — fresh camera checkpoint active");
            return;
        }

        const bool cam0_required = input_scan_allowed_.load() &&
            !waiting_for_new_input_.load() && new_tray_loaded_.load();
        if (cam0_required && !cam0CheckpointFrameReceived()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[POST_PLACE][CAM0] Waiting for first clean input-tray frame at HOME");
            return;
        }

        int next_empty_slot = SLOT_UNSET;
        if (claimFreshOutputSnapshot(next_empty_slot)) {
            // A coherent aggregate with a confirmed EMPTY slot proves non-full.
            // If all slots are occupied, wait for Vision's longer FULL debounce.
            post_place_output_check_pending_ = false;
            post_place_output_scan_started_ = false;
            setOutputScanAllowed(false);
            RCLCPP_INFO(get_logger(),
                "[POST_PLACE][CAM1] Tray NOT FULL confirmed at HOME "
                "(next empty O%d) — continue pipeline", next_empty_slot);
            routeAfterOutputPlacement();
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[POST_PLACE][CAM1] Waiting for coherent FULL/non-FULL camera decision...");
        return;
    }

    if (cartridge_pos2_busy_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[MUTEX][POS2] State3/4 BUSY — PLACE_TO_OUTPUT pending");
        return;
    }

    // Check this before any AI pre-scan/HOME action.  A CAM1_FULL request may
    // already have latched State4 while the scale result is arriving; moving
    // the robot then would revoke output-zone permission underneath State4.
    if (waiting_for_new_output_.load() || !new_trayoutput_loaded_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[INTERLOCK] Output tray is not confirmed ready — no pre-scan/place motion");
        return;
    }

    // ── Slot selection ──
    int slot = SLOT_UNSET;
    if (use_ai_for_control_) {
        // Vision or upstream camera failure is fail-closed.  A pipeline drain
        // after an already completed final placement remains a separate path.
        if (!visionHeartbeatFresh()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                "[SAFETY][CAM1] PLACE_TO_OUTPUT locked — vision heartbeat missing/stale");
            return;
        }
        if (output_scan_recovery_required_.load()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                "[SAFETY][CAM1] PLACE_TO_OUTPUT locked — manual HOME recovery required");
            return;
        }

        // Normal flow already collected this checkpoint after REFILL_BUFFER.
        // Drain/restart paths may not have passed that point; only those paths
        // use the dedicated HOME fallback below while the cartridge stays on
        // the scale.
        if (!output_prescan_home_done_.load()) {
            output_prescan_zone_verified_ = false;
            // A previous generic HOME/recovery may already have put the robot
            // at the verified checkpoint.  Start a fresh camera epoch in place
            // instead of commanding a redundant long HOME trajectory.
            if (cam1ViewClearFresh()) {
                output_prescan_home_done_ = true;
                setOutputScanAllowed(false);
                setOutputScanAllowed(true);
                RCLCPP_INFO(get_logger(),
                    "[SAFETY][CAM1] Robot already at verified HOME — start fresh "
                    "output snapshot without additional motion");
                return;
            }
            setOutputScanAllowed(false);
            RCLCPP_INFO(get_logger(),
                "[SAFETY][CAM1] Moving to verified HOME for a clean output snapshot");
            sendMotionActionAsync("OUTPUT_SCAN_HOME");
            return;
        }

        if (!cam1ViewClearFresh()) {
            setOutputScanAllowed(false);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[SAFETY][CAM1] At pre-scan checkpoint — waiting for fresh, stable "
                "/robot/cam1_view_clear");
            return;
        }

        if (!output_prescan_zone_verified_.exchange(true)) {
            // Both Robot Logic and Vision wait 1.5 s.  The aggregate frame stamp
            // must also be newer than this edge, so queued pre-HOME YOLO output
            // cannot become a motion command.
            setOutputScanAllowed(true);
            RCLCPP_INFO(get_logger(),
                "[SAFETY][CAM1] Camera checkpoint verified — cam1 opened; "
                "waiting 1.5 s + fresh frames");
            return;
        }

        if (!output_place_snapshot_requested_.load()) {
            requestOutputSnapshotNow();
            RCLCPP_INFO(get_logger(),
                "[SAFETY][CAM1] PLACE request armed — wait for next frame captured "
                "at this checkpoint (no extra HOME)");
            return;
        }

        if (!output_scan_allowed_.load() || !claimFreshOutputSnapshot(slot)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                "[SAFETY][CAM1] PLACE_TO_OUTPUT waiting — no coherent fresh "
                "tray/slot snapshot from the current camera epoch");
            return;
        }
        // claimFreshOutputSnapshot closes admission atomically.  Publish the
        // matching gate after releasing its mutex.
        output_place_snapshot_requested_ = false;
        setOutputScanAllowed(false);
    } else {
        std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
        {
            // AUTO: use counter managed by vision/logic
            if (selected_output_slot_ != SLOT_UNSET) {
                slot = selected_output_slot_;
            } else {
                // Vision node hasn't provided slot yet — use local counter
                slot = current_auto_slot_;
            }
        }
    }

    if (slot == SLOT_UNSET || slot <= 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
            "[OUTPUT] Waiting for slot selection...");
        return;
    }

    if (slot > TOTAL_OUTPUT_SLOTS) {
        // AUTO: chi bao ve gioi han tray, khong tham chieu occupied.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[OUTPUT] Slot O%d vuot TOTAL_OUTPUT_SLOTS — bo qua", slot);
        return;
    }

    // Check output tray ready (after done_tray_output, wait for new_trayoutput_loaded)
    if (waiting_for_new_output_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[OUTPUT] Waiting for new output tray...");
        return;
    }

    // Keep GUI OUTPUT SLOT synchronized with the actual destination selected
    // by AUTO/AI. The publisher existed previously but was never used, so the
    // GUI could not highlight the slot currently being placed.
    if (selected_slot_pub_) {
        auto slot_msg = std_msgs::msg::Int32();
        slot_msg.data = slot;
        selected_slot_pub_->publish(slot_msg);
    }

    // ── Send motion ──
    // Final edge guard: scale result/slot selection can arrive while a
    // cartridge state starts in parallel.  Re-check immediately before goal.
    if (cartridge_pos2_busy_.load() ||
        waiting_for_new_output_.load() || !new_trayoutput_loaded_.load() ||
        (use_ai_for_control_.load() &&
         (!cam1ViewClearFresh() || output_scan_recovery_required_.load() ||
          !visionHeartbeatFresh()))) {
        if (use_ai_for_control_ && !cam1ViewClearFresh())
            output_prescan_home_done_ = false;
        output_prescan_zone_verified_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[INTERLOCK] Output permission changed — SCALE_OUTPUT not sent");
        return;
    }
    RCLCPP_INFO(get_logger(), "[PLACE] 📦 SCALE → Slot %d [ASYNC]", slot);
    async_slot_ = slot;
    // Zone FALSE is expected as soon as this action is accepted; disarm the
    // pre-scan violation latch before Motion Executor starts moving.
    if (use_ai_for_control_) output_prescan_zone_verified_ = false;
    sendMotionActionAsync("SCALE_OUTPUT", slot);
}

// ============================================================================
// STATE: PLACE_TO_FAIL
// SCALE → FAIL position
// ============================================================================

void RobotLogicNode::statePlaceToFail()
{
    publishSystemStatus("PLACE_TO_FAIL");

    if (motion_busy_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE] Waiting for motion...");
        return;
    }

    if (getMotionCmd() == "SCALE_FAIL") {
        if (motion_in_progress_) return;
        clearMotionCmd();

        if (!motion_result_) {
            if (consumeTemporaryMotionInterlockReject("SCALE_FAIL", false))
                return;
            RCLCPP_ERROR(get_logger(), "[FAIL] ❌ SCALE_FAIL motion failed");
            return;
        }

        scale_has_cartridge_ = false;

        if (exitAfterScalePlaceRequested("PLACE_TO_FAIL"))
            return;

        // Skipped BUFFER→CHAMBER (feed_chamber timeout) → wait operator choice.
        // Manual mode falls through to IDLE branch below.
        if (!manual_mode_ && skipped_buffer_load_) {
            RCLCPP_WARN(get_logger(),
                "[PLACE_FAIL] ⚠️  skipped_buffer_load active → WAIT_RESUME_CHOICE");
            transitionTo(SystemState::WAIT_RESUME_CHOICE);
            return;
        }

        // Same priority as the PASS route: finish chamber work first and keep
        // refill_pending latched until a row is physically placed in buffer.
        if (!manual_mode_ && chamber_has_cartridge_ && fill_done_.load()) {
            const bool scale_result_pending = hasPendingScaleResults();
            if (scale_has_cartridge_ || scale_result_pending) {
                RCLCPP_WARN(get_logger(),
                    "[PRIORITY][PLACE_FAIL] fill_done held — scale not ready "
                    "(occupied=%d result_pending=%d)",
                    scale_has_cartridge_, scale_result_pending);
                transitionTo(SystemState::WAIT_FILLING);
                return;
            }
            RCLCPP_INFO(get_logger(),
                "[PRIORITY][PLACE_FAIL] fill_done before refill_pending -> TAKE_CHAMBER_TO_SCALE");
            fill_done_ = false;
            transitionTo(SystemState::TAKE_CHAMBER_TO_SCALE);
            return;
        }
        if (!manual_mode_ && !chamber_has_cartridge_ && !buffer_is_empty_ &&
            feed_chamber_signal_.load())
        {
            RCLCPP_INFO(get_logger(),
                "[PRIORITY][PLACE_FAIL] feed_chamber before refill_pending -> LOAD_CHAMBER_FROM_BUFFER");
            transitionTo(SystemState::LOAD_CHAMBER_FROM_BUFFER);
            return;
        }

        bool pipeline_empty = !chamber_has_cartridge_ && buffer_is_empty_;
        if (manual_mode_) {
            transitionTo(SystemState::IDLE);
        } else if (pipeline_empty) {
            if (waiting_for_new_input_.load()) {
                if (cartridge_drain_confirmed_.load()) {
                    RCLCPP_WARN(get_logger(),
                        "[PIPELINE] 📦 Pipeline drained (fail) + cartridge DRAIN confirmed -> Waiting in IDLE for new tray to auto-restart");
                    if (requestOutputTrayChangeForDrain("PLACE_TO_FAIL"))
                        transitionTo(SystemState::IDLE);
                    else
                        transitionTo(SystemState::WAIT_FILLING);
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                        "[PIPELINE] 📦 Pipeline drained (fail) but cartridge has not confirmed DRAIN → Waiting in WAIT_FILLING");
                    transitionTo(SystemState::WAIT_FILLING);
                }
            } else {
                RCLCPP_INFO(get_logger(),
                    "[PIPELINE] 📦 Pipeline drained (fail) + NEW tray available → Restarting pipeline cycle (INIT_LOAD_CHAMBER_DIRECT)");
                transitionTo(SystemState::INIT_LOAD_CHAMBER_DIRECT);
            }
        } else if (refill_pending_.load() && chamber_has_cartridge_ && buffer_is_empty_
                   && !waiting_for_new_input_.load() && new_tray_loaded_.load())
        {
            cartridge_drain_confirmed_ = false;
            RCLCPP_INFO(get_logger(),
                "[PRIORITY][PLACE_FAIL] Chamber commands idle -> consume refill_pending");
            transitionTo(SystemState::REFILL_BUFFER);
        } else {
            transitionTo(SystemState::WAIT_FILLING);
        }
        return;
    }

    RCLCPP_INFO(get_logger(), "[FAIL] 🤖 SCALE → FAIL [ASYNC]");
    sendMotionActionAsync("SCALE_FAIL");
}

// ============================================================================
// STATE: LAST_BATCH_WAIT
// ============================================================================

void RobotLogicNode::stateLastBatchWait()
{
    publishSystemStatus("LAST_BATCH_WAIT");
    if (chamber_has_cartridge_ && fill_done_) {
        const bool scale_result_pending = hasPendingScaleResults();
        if (scale_has_cartridge_ || scale_result_pending) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[LAST_BATCH] fill_done held — scale not ready "
                "(occupied=%d result_pending=%d)",
                scale_has_cartridge_, scale_result_pending);
            return;
        }
        fill_done_ = false;
        transitionTo(SystemState::TAKE_CHAMBER_TO_SCALE);
    }
}

// ============================================================================
// STATE: ERROR_SCALE
// ============================================================================

void RobotLogicNode::stateErrorScale()
{
    publishSystemStatus("ERROR_SCALE");
    publishError("SCALE ERROR");
    if (system_started_) {
        system_started_        = false;
        is_first_batch_        = true;
        chamber_has_cartridge_ = false;
        chamber_is_empty_      = true;
        scale_has_cartridge_   = false;
        transitionTo(SystemState::INIT_CHECK);
    }
}

// ============================================================================
// STATE: ERROR_MOTION_LOST
// ============================================================================

void RobotLogicNode::stateErrorMotionLost()
{
    publishSystemStatus("ERROR_MOTION_LOST");
    publishError("MOTION_EXECUTOR_LOST");
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
        "[ERROR] 🚨 Motion Executor lost — waiting for recovery...");
    if (checkMotionAlive(2.0)) {
        if (!motion_busy_.load()) {
            RCLCPP_WARN(get_logger(),
                "[RECOVERY] Heartbeat restored + motion_busy=false → IDLE");
            transitionTo(SystemState::IDLE);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[RECOVERY] Heartbeat restored but robot still motion_busy — remain locked");
        }
    }
}

// ============================================================================
// MOTION
// ============================================================================

bool RobotLogicNode::checkMotionAlive(double timeout_sec)
{
    const int64_t seen_ns = motion_hb_last_steady_ns_.load();
    const int64_t now_ns = steadyNowNs();
    if (!motion_hb_seen_.load() || seen_ns <= 0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
            "[WATCHDOG] Motion node heartbeat has not been received");
        return false;
    }
    const double dt = now_ns >= seen_ns
        ? static_cast<double>(now_ns - seen_ns) / 1.0e9
        : std::numeric_limits<double>::infinity();
    if (dt > timeout_sec) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
            "[WATCHDOG] Motion node lost! Last HB %.2fs ago", dt);
        return false;
    }
    return true;
}

bool RobotLogicNode::setDigitalOutput(int index, bool status)
{
    auto req = std::make_shared<DO::Request>();
    req->index  = index;
    req->status = status ? 1 : 0;
    auto res = callService<DO>(do_client_, req, "DO");
    if (!res || res->res != 0) {
        RCLCPP_ERROR(get_logger(), "[DO] Failed DO[%d]", index);
        return false;
    }
    if (index == 1 && gripper_cmd_pub_) {
        auto msg = std_msgs::msg::Bool(); msg.data = status;
        gripper_cmd_pub_->publish(msg);
    }
    if (index == 2 && picker_cmd_pub_) {
        auto msg = std_msgs::msg::Bool(); msg.data = status;
        picker_cmd_pub_->publish(msg);
    }
    return true;
}

bool RobotLogicNode::sendMotionAction(const std::string& cmd, int slot, int timeout_sec)
{
    if (!motion_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        publishError("ACTION_SERVER_OFFLINE");
        return false;
    }
    auto goal = ExecuteMotion::Goal();
    goal.command = cmd;
    goal.slot = slot;
    goal.allow_rollback = true;

    auto opts = rclcpp_action::Client<ExecuteMotion>::SendGoalOptions();
    auto fgh  = motion_action_client_->async_send_goal(goal, opts);
    if (fgh.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return false;
    auto gh = fgh.get();
    if (!gh) return false;

    auto fr = motion_action_client_->async_get_result(gh);
    if (fr.wait_for(std::chrono::seconds(timeout_sec)) != std::future_status::ready) {
        motion_action_client_->async_cancel_goal(gh);
        return false;
    }
    auto result = fr.get();
    return (result.code == rclcpp_action::ResultCode::SUCCEEDED) && result.result->success;
}

void RobotLogicNode::sendMotionActionAsync(const std::string& cmd, int slot)
{
    if (!motion_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        publishError("ACTION_SERVER_OFFLINE");
        motion_in_progress_ = false;
        motion_result_       = false;
        return;
    }

    auto goal = ExecuteMotion::Goal();
    goal.command = cmd;
    goal.slot = slot;
    goal.allow_rollback = true;

    auto opts = rclcpp_action::Client<ExecuteMotion>::SendGoalOptions();

    opts.goal_response_callback =
        [this, cmd](const GoalHandleExecuteMotion::SharedPtr& gh) {
            if (!gh) {
                RCLCPP_ERROR(get_logger(), "[ASYNC] ❌ Goal '%s' rejected", cmd.c_str());
                motion_in_progress_ = false;
                motion_result_       = false;
                last_motion_interlock_rejected_ = false;
                state_changed_       = true;
                state_cv_.notify_one();
            } else {
                RCLCPP_INFO(get_logger(), "[ASYNC] ✅ Goal '%s' accepted", cmd.c_str());
            }
        };

    opts.result_callback =
        [this, cmd](const GoalHandleExecuteMotion::WrappedResult& result) {
            const bool interlock_rejected = result.result &&
                result.result->message == "CARTRIDGE_INTERLOCK_BUSY_OR_STALE";
            last_motion_interlock_rejected_ = interlock_rejected;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    motion_result_ = result.result->success;
                    RCLCPP_INFO(get_logger(), "[ASYNC] ✅ '%s' done: %s",
                        cmd.c_str(), result.result->message.c_str());
                    // HOME is the explicit recovery point after an aborted
                    // pick/place: the arm is back at a known camera-safe pose.
                    if (cmd == "HOME" && result.result->success && use_ai_for_control_) {
                        input_scan_recovery_required_ = false;
                        output_scan_recovery_required_ = false;
                        output_prescan_home_done_ = true;
                        output_prescan_zone_verified_ = false;
                        setInputScanAllowed(true);
                        setOutputScanAllowed(false);
                        setOutputScanAllowed(true);
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    motion_result_ = false;
                    RCLCPP_ERROR(get_logger(), "[ASYNC] ❌ '%s' aborted", cmd.c_str());
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    motion_result_ = false;
                    RCLCPP_WARN(get_logger(), "[ASYNC] ⚠️ '%s' canceled", cmd.c_str());
                    break;
                default:
                    motion_result_ = false;
                    break;
            }
            motion_in_progress_ = false;
            state_changed_       = true;
            state_cv_.notify_one();
        };

    motion_in_progress_  = true;
    motion_result_       = false;
    last_motion_interlock_rejected_ = false;
    setMotionCmd(cmd);
    motion_started_at_   = this->now();
    motion_action_client_->async_send_goal(goal, opts);
}

// ============================================================================
// TRANSITION
// ============================================================================

void RobotLogicNode::transitionTo(SystemState new_state)
{
    SystemState old_state = current_state_;
    bool entered_ai_output_place = false;

    if (old_state == SystemState::PLACE_TO_OUTPUT &&
        new_state != SystemState::PLACE_TO_OUTPUT) {
        // Normal post-place routes clear these first. This additionally makes
        // operator GOTO/STOP transitions unable to resurrect an old pending
        // HOME snapshot when PLACE_TO_OUTPUT is entered again later.
        post_place_output_check_pending_ = false;
        post_place_output_scan_started_ = false;
    }

    if (new_state == SystemState::INIT_CHECK ||
        new_state == SystemState::INIT_LOAD_CHAMBER_DIRECT ||
        new_state == SystemState::TAKE_CHAMBER_TO_SCALE)
    {
        if (!validateRobotState()) {
            publishError("Robot Disconnected or Disabled");
            system_running_ = false;
            system_started_ = false;
            return;
        }
    }

    {
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        if (current_state_ != new_state) {
            current_state_ = new_state;
            state_changed_ = true;
            if (new_state == SystemState::PROCESSING_SCALE)
                scale_wait_start_ = this->now();
            entered_ai_output_place =
                new_state == SystemState::PLACE_TO_OUTPUT &&
                use_ai_for_control_.load();
        }
    }
    if (entered_ai_output_place) {
        if (output_prescan_home_done_.load()) {
            if (cam1ViewClearFresh() && output_scan_allowed_.load()) {
                requestOutputSnapshotNow();
                RCLCPP_INFO(get_logger(),
                    "[SAFETY][CAM1] PLACE_TO_OUTPUT edge — waiting for the next "
                    "cam1 frame captured now (no additional HOME)");
            } else {
                output_place_snapshot_requested_ = false;
            }
            RCLCPP_INFO(get_logger(),
                "[SAFETY][CAM1] PLACE_TO_OUTPUT entered — reuse the warm camera "
                "checkpoint after REFILL_BUFFER (no extra HOME)");
        } else {
            output_place_snapshot_requested_ = false;
            output_prescan_zone_verified_ = false;
            setOutputScanAllowed(false);
            RCLCPP_WARN(get_logger(),
                "[SAFETY][CAM1] PLACE_TO_OUTPUT has no valid camera checkpoint — "
                "fallback scan recovery may be required");
        }
    }
    state_cv_.notify_all();

    RCLCPP_INFO(get_logger(), "[STATE] %s → %s",
        stateToString(old_state).c_str(),
        stateToString(new_state).c_str());
    publishSystemStatus(stateToString(new_state));
}

bool RobotLogicNode::checkConnection() { return true; }

bool RobotLogicNode::validateRobotState()
{
    if (!checkConnection()) return false;
    if (manual_mode_) return true;
    if (!system_enabled_) {
        RCLCPP_ERROR(get_logger(), "[VALIDATION] Robot NOT ENABLED");
        return false;
    }
    return true;
}

std::string RobotLogicNode::stateToString(SystemState state)
{
    switch (state) {
        case SystemState::IDLE:                    return "IDLE";
        case SystemState::INIT_CHECK:              return "INIT_CHECK";
        case SystemState::INIT_LOAD_CHAMBER_DIRECT: return "INIT_LOAD_CHAMBER_DIRECT";
        case SystemState::INIT_REFILL_BUFFER:      return "INIT_REFILL_BUFFER";
        case SystemState::AI_CHECK_INPUT_TRAY:     return "AI_CHECK_INPUT_TRAY";
        case SystemState::WAIT_FILLING:            return "WAIT_FILLING";
        case SystemState::TAKE_CHAMBER_TO_SCALE:   return "TAKE_CHAMBER_TO_SCALE";
        case SystemState::LOAD_CHAMBER_FROM_BUFFER: return "LOAD_CHAMBER_FROM_BUFFER";
        case SystemState::WAIT_RESUME_CHOICE:      return "WAIT_RESUME_CHOICE";
        case SystemState::WAIT_SCALE_CHOICE:       return "WAIT_SCALE_CHOICE";
        case SystemState::REFILL_BUFFER:           return "REFILL_BUFFER";
        case SystemState::PROCESSING_SCALE:        return "PROCESSING_SCALE";
        case SystemState::ERROR_SCALE_TIMEOUT:     return "ERROR_SCALE_TIMEOUT";
        case SystemState::PLACE_TO_OUTPUT:         return "PLACE_TO_OUTPUT";
        case SystemState::PLACE_TO_FAIL:           return "PLACE_TO_FAIL";
        case SystemState::LAST_BATCH_WAIT:         return "LAST_BATCH_WAIT";
        case SystemState::ERROR_SCALE:             return "ERROR_SCALE";
        case SystemState::ERROR_INPUT_TRAY_EMPTY:  return "ERROR_INPUT_TRAY_EMPTY";
        case SystemState::ERROR_MOTION_LOST:       return "ERROR_MOTION_LOST";
        default: return "UNKNOWN";
    }
}

void RobotLogicNode::publishSystemStatus(const std::string& status)
{
    static std::string last;
    static std::mutex  mtx;
    static std::chrono::steady_clock::time_point last_pub{};

    // Publisher khong latch, nen truoc day subscriber vao tre — dien hinh la
    // restart GUI — khong nhan duoc gi cho toi khi robot doi state lan ke tiep,
    // o "Trang thai robot" tren panel dung yen o gia tri cu hoac trong. Phat lai
    // moi 1 s de no bat kip. Van chi ghi log khi gia tri that su doi, va phia
    // GUI dedupe truoc khi emit nen heartbeat khong lam QML fire lai.
    bool changed = false;
    {
        std::lock_guard<std::mutex> lk(mtx);
        const auto now = std::chrono::steady_clock::now();
        changed = (status != last);
        if (!changed && now - last_pub < std::chrono::seconds(1))
            return;
        last     = status;
        last_pub = now;
    }

    auto msg = std_msgs::msg::String();
    msg.data = status;
    system_status_pub_->publish(msg);
    if (changed)
        RCLCPP_INFO(get_logger(), "[STATUS] %s", status.c_str());
}

void RobotLogicNode::publishError(const std::string& error)
{
    auto msg = std_msgs::msg::String();
    msg.data = error;
    error_pub_->publish(msg);
    RCLCPP_ERROR(get_logger(), "[ERROR] %s", error.c_str());
}

bool RobotLogicNode::sendMotionCommand(const std::string& command, int timeout_sec)
{
    return sendMotionAction(command, 0, timeout_sec);
}

template <typename ServiceT>
typename ServiceT::Response::SharedPtr RobotLogicNode::callService(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request,
    const std::string& service_name)
{
    using namespace std::chrono_literals;
    if (!client->wait_for_service(100ms)) {
        RCLCPP_ERROR(get_logger(), "[SERVICE] %s not available", service_name.c_str());
        return nullptr;
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "[SERVICE] %s timeout", service_name.c_str());
        return nullptr;
    }
    try { return future.get(); }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[SERVICE] %s exception: %s", service_name.c_str(), e.what());
        return nullptr;
    }
}

// ============================================================================
// MAIN
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
