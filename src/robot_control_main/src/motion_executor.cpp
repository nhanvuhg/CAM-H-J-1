/**
 * @file motion_executor.cpp
 * @brief Handles all Dobot motion commands and digital outputs
 * 
 * Responsibilities:
 * - Execute motion commands from state machine
 * - Call Dobot services (JointMovJ, RelMovL, DO, Sync)
 * - Report motion completion/failure
 * 
 * Topics Subscribed:
 * - /robot/motion_command (String) - Command format: "TYPE:PARAM" (e.g., "PICK_ROW:3")
 * 
 * Topics Published:
 * - /robot/motion_result (Bool) - True = success, False = failure
 * - /robot/motion_status (String) - Current motion status
 * - /robot/motion_busy (Bool) - Is motion in progress
 * - /robot/motion_heartbeat (Header) - Node aliveness with timestamp

 * - /robot/gripper_cmd (Bool) - Gripper state feedback
 * - /robot/picker_cmd (Bool) - Picker state feedback
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_interfaces/action/execute_motion.hpp"



// Dobot Messages
#include "dobot_msgs_v3/srv/enable_robot.hpp"
#include "dobot_msgs_v3/srv/get_pose.hpp"
#include "dobot_msgs_v3/srv/get_angle.hpp"
#include "dobot_msgs_v3/srv/joint_mov_j.hpp"
#include "dobot_msgs_v3/srv/mov_l.hpp"
#include "dobot_msgs_v3/srv/mov_j.hpp"
#include "dobot_msgs_v3/srv/rel_mov_l.hpp"
#include "dobot_msgs_v3/srv/rel_mov_l_user.hpp"
#include "dobot_msgs_v3/srv/do.hpp"
#include "dobot_msgs_v3/srv/robot_mode.hpp"
#include "dobot_msgs_v3/srv/speed_l.hpp"
#include "dobot_msgs_v3/srv/speed_factor.hpp"
#include "dobot_msgs_v3/srv/acc_l.hpp"
#include "dobot_msgs_v3/srv/speed_j.hpp"
#include "dobot_msgs_v3/srv/acc_j.hpp"
#include "dobot_msgs_v3/srv/sync.hpp"
#include "dobot_msgs_v3/srv/clear_error.hpp"
#include "dobot_msgs_v3/srv/get_error_id.hpp"
#include "dobot_msgs_v3/srv/pause.hpp"
#include "dobot_msgs_v3/srv/continues.hpp"
#include "dobot_msgs_v3/srv/emergency_stop.hpp"
#include "dobot_msgs_v3/srv/stop_script.hpp"
#include "dobot_msgs_v3/srv/reset_robot.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <thread>

using namespace std::chrono_literals;

using EnableRobot = dobot_msgs_v3::srv::EnableRobot;
using GetPose = dobot_msgs_v3::srv::GetPose;
using GetAngle = dobot_msgs_v3::srv::GetAngle;
using JointMovJ = dobot_msgs_v3::srv::JointMovJ;
using MovL = dobot_msgs_v3::srv::MovL;
using MovJ = dobot_msgs_v3::srv::MovJ;
using RelMovL = dobot_msgs_v3::srv::RelMovL;
using RelMovLUser = dobot_msgs_v3::srv::RelMovLUser;
using DO = dobot_msgs_v3::srv::DO;
using RobotMode = dobot_msgs_v3::srv::RobotMode;
using SpeedL = dobot_msgs_v3::srv::SpeedL;
using SpeedFactor = dobot_msgs_v3::srv::SpeedFactor;
using AccL = dobot_msgs_v3::srv::AccL;
using SpeedJ = dobot_msgs_v3::srv::SpeedJ;
using AccJ = dobot_msgs_v3::srv::AccJ;
using SyncSrv = dobot_msgs_v3::srv::Sync;
using ClearError = dobot_msgs_v3::srv::ClearError;
using GetErrorID = dobot_msgs_v3::srv::GetErrorID;
using Pause = dobot_msgs_v3::srv::Pause;
using Continues = dobot_msgs_v3::srv::Continues;
using EmergencyStop = dobot_msgs_v3::srv::EmergencyStop;
using StopScript = dobot_msgs_v3::srv::StopScript;
using ResetRobot = dobot_msgs_v3::srv::ResetRobot;

// ============================================================================
// MOTION EXECUTOR NODE
// ============================================================================

class MotionExecutorNode : public rclcpp::Node {
public:
    MotionExecutorNode() : Node("motion_executor") {
        RCLCPP_INFO(get_logger(), "[MOTION] === Motion Executor Node Starting ===");
        
        loadMotionParameters();
        initServiceClients();
        
        // Publishers
        pub_busy_ = create_publisher<std_msgs::msg::Bool>("/robot/motion_busy", 10);

        pub_heartbeat_ = create_publisher<std_msgs::msg::Header>("/robot/motion_heartbeat", 10);
        pub_gripper_ = create_publisher<std_msgs::msg::Bool>("/robot/gripper_cmd", 10);
        pub_picker_ = create_publisher<std_msgs::msg::Bool>("/robot/picker_cmd", 10);
        pub_cyl_loadcell_ = create_publisher<std_msgs::msg::Bool>("/robot/cyl_loadcell_cmd", 10);

        
        // Subscriptions
        sub_gripper_status_ = create_subscription<std_msgs::msg::Bool>(
            "/robot/gripper_status", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                last_gripper_status_ = msg->data;
            });
            
        sub_picker_status_ = create_subscription<std_msgs::msg::Bool>(
            "/robot/picker_status", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                last_picker_status_ = msg->data;
            });

        sub_cyl_loadcell_status_ = create_subscription<std_msgs::msg::Bool>(
            "/robot/cyl_loadcell_status", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                last_cyl_loadcell_status_ = msg->data;
            });

        speed_ratio_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/robot/speed_ratio", rclcpp::QoS(10).reliable().transient_local(),
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                current_speed_ratio_ = std::clamp(msg->data, 1, 100);
                RCLCPP_INFO(get_logger(), "[SPEED] Speed ratio updated: %d%% (GUI already sent SpeedFactor to hardware)", current_speed_ratio_);
            });

        soft_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/system/soft_stop", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) requestMotionStop("/system/soft_stop");
            });

        stop_button_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/system/stop_button", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) requestMotionStop("/system/stop_button");
            });

        // Heartbeat Timer (500ms)
        heartbeat_timer_ = create_wall_timer(500ms, [this]() {
            std_msgs::msg::Header h;
            h.stamp = this->now();
            h.frame_id = "motion_executor";
            pub_heartbeat_->publish(h);
        });

        // Action Server
        action_server_ = rclcpp_action::create_server<ExecuteMotion>(
            this,
            "/robot/execute_motion",
            std::bind(&MotionExecutorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionExecutorNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&MotionExecutorNode::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "[MOTION] === Motion Executor Node Ready (Action Server Enabled) ===");
    }


private:
    static constexpr int TOTAL_OUTPUT_SLOTS = 10;

    // ========================================================================
    // MOTION DATA
    // ========================================================================
    std::vector<std::vector<double>> joint_sequences_;
    std::vector<std::vector<double>> relmovl_sequences_;
    std::vector<std::pair<int, int>> digital_output_steps_;
    
    std::vector<double> safe_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    std::atomic<bool> motion_in_progress_{false};
    // Abort flag — set bởi handle_cancel (STOP), check trong tất cả motion helpers
    // để thoát NGAY khi STOP thay vì chạy hết sequence.
    std::atomic<bool> abort_motion_{false};

    int current_fail_slot_{1};

    // Speed ratio from GUI (set via /robot/speed_ratio topic)
    int current_speed_ratio_{14};  // default matches GUI saved value

    // ========================================================================
    // ROS INTERFACES
    // ========================================================================    // ROS INTERFACES
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_busy_;

    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_heartbeat_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_gripper_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_picker_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cyl_loadcell_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gripper_status_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_picker_status_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cyl_loadcell_status_;
    std::atomic<bool> last_gripper_status_{false};
    std::atomic<bool> last_picker_status_{false};
    std::atomic<bool> last_cyl_loadcell_status_{false};

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr speed_ratio_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_button_sub_;
    
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // Action Server Members
    using ExecuteMotion = robot_control_interfaces::action::ExecuteMotion;
    using GoalHandleExecuteMotion = rclcpp_action::ServerGoalHandle<ExecuteMotion>;
    rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;

    
    // Service Clients
    rclcpp::Client<EnableRobot>::SharedPtr enable_client_;
    rclcpp::Client<ClearError>::SharedPtr clear_error_client_;
    rclcpp::Client<GetPose>::SharedPtr pose_client_;
    rclcpp::Client<GetAngle>::SharedPtr angle_client_;
    rclcpp::Client<JointMovJ>::SharedPtr joint_client_;
    rclcpp::Client<MovL>::SharedPtr movl_client_;
    rclcpp::Client<MovJ>::SharedPtr movj_client_;
    rclcpp::Client<RelMovL>::SharedPtr relmovl_client_;
    rclcpp::Client<RelMovLUser>::SharedPtr relmovluser_client_;
    rclcpp::Client<DO>::SharedPtr do_client_;
    rclcpp::Client<RobotMode>::SharedPtr robot_mode_client_;
    rclcpp::Client<SpeedL>::SharedPtr speedl_client_;
    rclcpp::Client<SpeedFactor>::SharedPtr speedfactor_client_;
    rclcpp::Client<AccL>::SharedPtr accl_client_;
    rclcpp::Client<SpeedJ>::SharedPtr speedj_client_;
    rclcpp::Client<AccJ>::SharedPtr accj_client_;
    rclcpp::Client<SyncSrv>::SharedPtr sync_client_;
    rclcpp::Client<GetErrorID>::SharedPtr error_client_;
    rclcpp::Client<Pause>::SharedPtr pause_client_;
    rclcpp::Client<Continues>::SharedPtr continue_client_;
    rclcpp::Client<EmergencyStop>::SharedPtr emergency_stop_client_;
    rclcpp::Client<StopScript>::SharedPtr stop_script_client_;
    rclcpp::Client<ResetRobot>::SharedPtr reset_robot_client_;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    void initServiceClients() {
        // ROS 2 Humble create_client() takes an rmw_qos_profile_t.
        const auto qos = rmw_qos_profile_services_default;
        
        enable_client_ = create_client<EnableRobot>("/nova5/dobot_bringup/EnableRobot", qos);
        clear_error_client_ = create_client<ClearError>("/nova5/dobot_bringup/ClearError", qos);
        pose_client_ = create_client<GetPose>("/nova5/dobot_bringup/GetPose", qos);
        angle_client_ = create_client<GetAngle>("/nova5/dobot_bringup/GetAngle", qos);
        joint_client_ = create_client<JointMovJ>("/nova5/dobot_bringup/JointMovJ", qos);
        movl_client_ = create_client<MovL>("/nova5/dobot_bringup/MovL", qos);
        movj_client_ = create_client<MovJ>("/nova5/dobot_bringup/MovJ", qos);
        relmovl_client_ = create_client<RelMovL>("/nova5/dobot_bringup/RelMovL", qos);
        relmovluser_client_ = create_client<RelMovLUser>("/nova5/dobot_bringup/RelMovLUser", qos);
        do_client_ = create_client<DO>("/nova5/dobot_bringup/DO", qos);
        robot_mode_client_ = create_client<RobotMode>("/nova5/dobot_bringup/RobotMode", qos);
        speedl_client_ = create_client<SpeedL>("/nova5/dobot_bringup/SpeedL", qos);
        speedfactor_client_ = create_client<SpeedFactor>("/nova5/dobot_bringup/SpeedFactor", qos);
        accl_client_ = create_client<AccL>("/nova5/dobot_bringup/AccL", qos);
        speedj_client_ = create_client<SpeedJ>("/nova5/dobot_bringup/SpeedJ", qos);
        accj_client_ = create_client<AccJ>("/nova5/dobot_bringup/AccJ", qos);
        sync_client_ = create_client<SyncSrv>("/nova5/dobot_bringup/Sync", qos);
        error_client_ = create_client<GetErrorID>("/nova5/dobot_bringup/GetErrorID", qos);
        pause_client_ = create_client<Pause>("/nova5/dobot_bringup/Pause", qos);
        continue_client_ = create_client<Continues>("/nova5/dobot_bringup/Continue", qos);
        emergency_stop_client_ = create_client<EmergencyStop>("/nova5/dobot_bringup/EmergencyStop", qos);
        stop_script_client_ = create_client<StopScript>("/nova5/dobot_bringup/StopScript", qos);
        reset_robot_client_ = create_client<ResetRobot>("/nova5/dobot_bringup/ResetRobot", qos);
        
        RCLCPP_INFO(get_logger(), "[MOTION] Service clients initialized");
    }
    
    void loadMotionParameters() {
        this->declare_parameter("motion_sequence", std::vector<std::string>{});
        std::vector<std::string> seq_lines;
        this->get_parameter("motion_sequence", seq_lines);
        
        this->declare_parameter("safe_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->get_parameter("safe_pose", safe_pose_);


        joint_sequences_.clear();
        relmovl_sequences_.clear();
        digital_output_steps_.clear();

        for (const auto& line : seq_lines) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.empty()) continue;

            const std::string& type = tokens[0];

            if (type == "J") {
                std::vector<double> joints;
                for (size_t i = 1; i < tokens.size(); ++i) {
                    try { 
                        joints.push_back(std::stod(tokens[i])); 
                    } catch (...) {}
                }
                if (joints.size() == 6) {
                    joint_sequences_.push_back(joints);
                }
            } else if (type == "R" && tokens.size() >= 4) {
                try {
                    relmovl_sequences_.emplace_back(std::vector<double>{
                        std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3])});
                } catch (...) {}
            } else if (type == "D" && tokens.size() >= 3) {
                try {
                    int do_index = std::stoi(tokens[1]);
                    int do_status = std::stoi(tokens[2]);
                    digital_output_steps_.emplace_back(do_index, do_status);
                } catch (...) {}
            }
        }

        RCLCPP_INFO(get_logger(), "[MOTION] Loaded: %zu joints, %zu relmovl, %zu DO",
                    joint_sequences_.size(), relmovl_sequences_.size(), digital_output_steps_.size());
    }

    // ========================================================================
    // SERVICE CALL HELPER
    // ========================================================================
    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr callService(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request,
        const std::string& service_name) 
    {
        if (!client->wait_for_service(2s)) {
            RCLCPP_ERROR(get_logger(), "[SERVICE] %s not available", service_name.c_str());
            return nullptr;
        }

        auto future = client->async_send_request(request);
        
        if (future.wait_for(5s) == std::future_status::ready) {
            try {
                return future.get();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "[SERVICE] %s exception: %s", service_name.c_str(), e.what());
                return nullptr;
            }
        }
        
        RCLCPP_ERROR(get_logger(), "[SERVICE] %s timeout", service_name.c_str());
        return nullptr;
    }

    bool continueIfPaused(const std::string& context) {
        if (!continue_client_ || !continue_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "[%s] Continue service not ready", context.c_str());
            return false;
        }
        auto req = std::make_shared<Continues::Request>();
        auto res = callService<Continues>(continue_client_, req, "Continue");
        if (!res) return false;
        RCLCPP_WARN(get_logger(), "[%s] RobotMode=10 (PAUSED) — Continue result: %d",
                    context.c_str(), res->res);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        return res->res == 0;
    }

    // ========================================================================
    // MOTION PRIMITIVES
    // ========================================================================
    bool moveToIndex(size_t index, int speed_override = -1) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveToIndex] aborted"); return false; }
        if (index >= joint_sequences_.size()) {
            RCLCPP_ERROR(get_logger(), "[MOTION] Invalid index: %zu (max: %zu)", 
                         index, joint_sequences_.size() - 1);
            return false;
        }

        if (!prepareJointMotion(speed_override)) {
            RCLCPP_WARN(get_logger(), "[MOTION] Failed to prepare Joint Motion (Speed/Acc)");
        }

        auto req = std::make_shared<JointMovJ::Request>();
        const auto& joints = joint_sequences_[index];
        req->j1 = joints[0];
        req->j2 = joints[1];
        req->j3 = joints[2];
        req->j4 = joints[3];
        req->j5 = joints[4];
        req->j6 = joints[5];

        RCLCPP_INFO(get_logger(), "[MOTION] JointMovJ -> Index %zu", index);

        auto res = callService<JointMovJ>(joint_client_, req, "JointMovJ");
        if (!res) return false;
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[MOTION] JointMovJ failed (err: %d)", res->res);
            return false;
        }

        return sync();
    }

    bool moveR(double dx, double dy, double dz, int speed_override = -1) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveR] aborted"); return false; }
        if (!prepareLinearMotion(speed_override)) {
            RCLCPP_ERROR(get_logger(), "[moveR] Prepare failed");
            return false;
        }

        auto current_pose = getCurrentPose();
        if (current_pose.size() < 6) {
            RCLCPP_ERROR(get_logger(), "[moveR] No current pose");
            return false;
        }

        // Full pose log for debugging
        RCLCPP_INFO(get_logger(),
            "[moveR] Current: X=%.2f Y=%.2f Z=%.2f Rx=%.2f Ry=%.2f Rz=%.2f",
            current_pose[0], current_pose[1], current_pose[2],
            current_pose[3], current_pose[4], current_pose[5]);

        auto req = std::make_shared<MovL::Request>();
        req->x  = current_pose[0] + dx;
        req->y  = current_pose[1] + dy;
        req->z  = current_pose[2] + dz;
        req->rx = current_pose[3];
        req->ry = current_pose[4];
        req->rz = current_pose[5];
        req->param_value.clear();

        RCLCPP_INFO(get_logger(),
            "[moveR] Target:  X=%.2f Y=%.2f Z=%.2f Rx=%.2f Ry=%.2f Rz=%.2f",
            req->x, req->y, req->z, req->rx, req->ry, req->rz);

        auto res = callService<MovL>(movl_client_, req, "MovL");
        if (!res) {
            RCLCPP_ERROR(get_logger(), "[moveR] Service call returned nullptr (timeout/unavailable)");
            return false;
        }
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[moveR] Rejected, code=%d", res->res);
            return false;
        }

        return sync();
    }


    bool moveJ_Absolute(const std::vector<double>& pose) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveJ_Absolute] aborted"); return false; }
        if (pose.size() < 6) return false;
        
        if (!prepareJointMotion()) {
            RCLCPP_WARN(get_logger(), "[MOTION] Failed to prepare Joint Motion (Speed/Acc)");
        }

        auto req = std::make_shared<MovJ::Request>();
        req->x = pose[0];
        req->y = pose[1];
        req->z = pose[2];
        req->rx = pose[3];
        req->ry = pose[4];
        req->rz = pose[5];
        req->param_value.clear();
        
        auto res = callService<MovJ>(movj_client_, req, "MovJ");
        if (!res) return false;
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[moveJ] Failed (err: %d)", res->res);
            return false;
        }

        return sync();
    }

    bool moveL_Absolute(const std::vector<double>& pose) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveL_Absolute] aborted"); return false; }
        if (pose.size() < 6) return false;
        
        if (!prepareLinearMotion()) {
            RCLCPP_WARN(get_logger(), "[MOTION] Failed to prepare Linear Motion (Speed/Acc)");
        }

        auto req = std::make_shared<MovL::Request>();
        req->x = pose[0];
        req->y = pose[1];
        req->z = pose[2];
        req->rx = pose[3];
        req->ry = pose[4];
        req->rz = pose[5];
        req->param_value.clear();
        
        auto res = callService<MovL>(movl_client_, req, "MovL");
        if (!res) return false;
        if (res->res != 0) {
            RCLCPP_ERROR(get_logger(), "[moveL] Failed (err: %d)", res->res);
            return false;
        }

        return sync();
    }


    bool setDigitalOutput(int index, bool status) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[setDigitalOutput] aborted"); return false; }
        // Publish to ROS topics first for the Festo Gripper Node
        if (index == 1 && pub_gripper_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = status;
            pub_gripper_->publish(msg);
            
            // Wait for feedback from Python node
            auto start = std::chrono::steady_clock::now();
            while (last_gripper_status_ != status && rclcpp::ok()) {
                if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(2000)) {
                    RCLCPP_WARN(get_logger(), "[DO] Timeout waiting for Gripper status feedback!");
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }
        if (index == 2 && pub_picker_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = status;
            pub_picker_->publish(msg);
            
            // Wait for feedback from Python node
            auto start = std::chrono::steady_clock::now();
            while (last_picker_status_ != status && rclcpp::ok()) {
                if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(2000)) {
                    RCLCPP_WARN(get_logger(), "[DO] Timeout waiting for Picker status feedback!");
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // Cyl_loadcell nằm trên CPX ch8/9 (qua festo_gripper_controller), KHÔNG phải Dobot DO.
        // index==6: true = EXTEND (KẸP/ch9), false = RETRACT (NHẢ/ch8).
        if (index == 6 && pub_cyl_loadcell_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = status;
            pub_cyl_loadcell_->publish(msg);

            // Wait for feedback from Python node (festo_gripper_controller)
            auto start = std::chrono::steady_clock::now();
            while (last_cyl_loadcell_status_ != status && rclcpp::ok()) {
                if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(2000)) {
                    RCLCPP_WARN(get_logger(), "[DO] Timeout waiting for Cyl_loadcell status feedback!");
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
            return true;   // cyl_loadcell thuần CPX — không gọi Dobot DO port 6
        }

        // Try to trigger Dobot's hardware DO (optional, might fail if not configured)
        auto req = std::make_shared<DO::Request>();
        req->index = index;
        req->status = status ? 1 : 0;

        auto res = callService<DO>(do_client_, req, "DO");
        if (!res) {
            RCLCPP_WARN(get_logger(), "[DO] Hardware DO service call failed.");
        } else if (res->res != 0) {
            RCLCPP_WARN(get_logger(), "[DO] Hardware DO Failed (err: %d), but ROS topic published.", res->res);
        }

        // Always return true to keep the motion sequence running
        return true;
    }

    bool sync() {
        // Sync() returns -10000 on this firmware (command unsupported).
        // Instead, poll RobotMode until robot is no longer in motion (mode != 7).
        // Mode values: 5=standby, 7=running, 9=error
        if (!robot_mode_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "[SYNC] RobotMode service not ready, using sleep fallback");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            return true;
        }

        // VITAL FIX: Give the Dobot controller time to transition into mode 7
        // After sending a move command, the controller is still in mode 5 for ~50-150ms 
        // before it officially starts trajectory execution.
        rclcpp::sleep_for(std::chrono::milliseconds(250));

        bool continue_sent = false;
        const int max_attempts = 600; // 60 seconds max
        for (int i = 0; i < max_attempts; ++i) {
            // Thoát ngay nếu STOP/cancel — không chờ thêm
            if (shouldAbort()) {
                RCLCPP_WARN(get_logger(), "[SYNC] aborted (STOP) — return false");
                return false;
            }
            auto req = std::make_shared<RobotMode::Request>();
            auto future = robot_mode_client_->async_send_request(req);
            if (future.wait_for(1s) != std::future_status::ready) {
                RCLCPP_WARN(get_logger(), "[SYNC] RobotMode request timed out, retrying...");
                continue;
            }
            try {
                auto res = future.get();
                if (res && res->res == 0) {
                    int mode = std::stoi(res->mode);
                    if (mode == 5) {
                        RCLCPP_DEBUG(get_logger(), "[SYNC] Robot idle (mode=5) after %d polls - SUCCESS", i);
                        return true;
                    } else if (mode == 10 && !continue_sent) {
                        continue_sent = true;
                        continueIfPaused("SYNC");
                        continue;
                    } else if (mode != 7) {
                        RCLCPP_ERROR(get_logger(), "[SYNC] Motion INTERRUPTED! Mode=%d (not 7 or 5)", mode);
                        return false;
                    }
                }
            } catch (...) {}
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_ERROR(get_logger(), "[SYNC] Timeout waiting for motion to complete! Robot still in mode 7.");
        return false;
    }

    std::vector<double> getCurrentPose() {
        auto req = std::make_shared<GetPose::Request>();
        req->user = 0;
        req->tool = 0;  // tool=0 matches MovL/MovJ base frame
        
        auto res = callService<GetPose>(pose_client_, req, "GetPose");
        
        if (!res) {
            RCLCPP_ERROR(get_logger(), "[getCurrentPose] Service call failed");
            return {};
        }
        
        std::string pose_str = res->pose;
        pose_str.erase(std::remove(pose_str.begin(), pose_str.end(), '{'), pose_str.end());
        pose_str.erase(std::remove(pose_str.begin(), pose_str.end(), '}'), pose_str.end());
        
        std::vector<double> pose;
        std::stringstream ss(pose_str);
        std::string item;
        
        while (std::getline(ss, item, ',')) {
            try {
                pose.push_back(std::stod(item));
            } catch (...) {}
        }
        
        if (pose.size() < 6) {
            RCLCPP_ERROR(get_logger(), "[getCurrentPose] Invalid pose size: %zu", pose.size());
            return {};
        }

        return pose;
    }

    // Wait (delay) — chèn vào giữa các motion command, vd:
    //   moveToIndex(5);
    //   wait(2.0);            // chờ 2 giây
    //   moveR(0, 0, -30);
    // Interruptible: thoát sớm khi rclcpp shutdown. Returns false nếu bị shutdown.
    bool wait(double seconds) {
        if (seconds <= 0.0) return true;
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[wait] aborted at start"); return false; }
        RCLCPP_INFO(get_logger(), "[MOTION] wait %.2fs", seconds);
        auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::milliseconds(static_cast<int>(seconds * 1000));
        while (std::chrono::steady_clock::now() < deadline) {
            if (shouldAbort()) {
                RCLCPP_WARN(get_logger(), "[wait] aborted mid-sleep");
                return false;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }


    bool prepareLinearMotion(int speed_override = -1) {
        int spd = (speed_override > 0) ? speed_override : current_speed_ratio_;  // Use system or override speed
        RCLCPP_INFO(get_logger(), "[MOTION] prepareLinearMotion speed=%d%%", spd);
        // Set SpeedL
        auto speed_req = std::make_shared<SpeedL::Request>();
        speed_req->r = spd;
        if (!callService<SpeedL>(speedl_client_, speed_req, "SpeedL")) return false;

        // Set AccL
        auto acc_req = std::make_shared<AccL::Request>();
        acc_req->r = spd;
        if (!callService<AccL>(accl_client_, acc_req, "AccL")) return false;
        
        return true;
    }

    bool prepareJointMotion(int speed_override = -1) {
        int spd = (speed_override > 0) ? speed_override : current_speed_ratio_;  // Use system or override speed
        RCLCPP_INFO(get_logger(), "[MOTION] prepareJointMotion speed=%d%%", spd);
        // Set SpeedJ
        auto speed_req = std::make_shared<SpeedJ::Request>();
        speed_req->r = spd;
        if (!callService<SpeedJ>(speedj_client_, speed_req, "SpeedJ")) return false;

        // Set AccJ
        auto acc_req = std::make_shared<AccJ::Request>();
        acc_req->r = spd;
        if (!callService<AccJ>(accj_client_, acc_req, "AccJ")) return false;
        
        return true;
    }

    // RAII Guard for motion_busy
    struct MotionBusyGuard {
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub;
        MotionBusyGuard(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p) : pub(p) {
            std_msgs::msg::Bool msg; msg.data = true; pub->publish(msg);
        }
        ~MotionBusyGuard() {
            std_msgs::msg::Bool msg; msg.data = false; pub->publish(msg);
        }
    };

    // ========================================================================
    // COMMAND HANDLER

    // ========================================================================



    // ========================================================================
    // ACTION SERVER CALLBACKS
    // ========================================================================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteMotion::Goal> goal)
    {
        (void)uuid;
        // CAS-claim cờ ngay tại handle_goal — chặn TOCTOU giữa 2 goal đến song song.
        // executeAction sẽ KHÔNG set lại motion_in_progress_, chỉ clear khi xong.
        bool expected = false;
        if (!motion_in_progress_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(get_logger(), "[ACTION] Rejecting goal: motion in progress");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(get_logger(), "[ACTION] Received goal: %s (slot: %d)",
            goal->command.c_str(), goal->slot);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_WARN(get_logger(), "[ACTION] Received request to cancel goal — stop Dobot motion");
        requestMotionStop("action_cancel");
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void handle_accepted(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
        // Execute in separate thread
        std::thread{std::bind(&MotionExecutorNode::executeAction, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Helper: motion helpers gọi để check có cần thoát ngay không.
    // True = ROS shutdown HOẶC handle_cancel đã set abort_motion_ (STOP).
    bool shouldAbort() const {
        return abort_motion_.load() || !rclcpp::ok();
    }

    void requestMotionStop(const std::string& source) {
        abort_motion_.store(true);
        RCLCPP_WARN(get_logger(), "[STOP] %s -> abort flag + Dobot stop/reset", source.c_str());

        if (stop_script_client_ && stop_script_client_->service_is_ready()) {
            auto req = std::make_shared<StopScript::Request>();
            stop_script_client_->async_send_request(req,
                [this](rclcpp::Client<StopScript>::SharedFuture f) {
                    try {
                        RCLCPP_WARN(this->get_logger(), "[STOP] Dobot StopScript result: %d", f.get()->res);
                    } catch (...) {
                        RCLCPP_WARN(this->get_logger(), "[STOP] Dobot StopScript failed");
                    }
                });
        } else {
            RCLCPP_WARN(get_logger(), "[STOP] StopScript service not ready");
        }

        // Normal STOP avoids Pause(): Pause can leave ServoP/Cartesian jog
        // paused after recovery even though Joint MoveJog still works.
        if (reset_robot_client_ && reset_robot_client_->service_is_ready()) {
            auto req = std::make_shared<ResetRobot::Request>();
            reset_robot_client_->async_send_request(req,
                [this](rclcpp::Client<ResetRobot>::SharedFuture f) {
                    try {
                        RCLCPP_WARN(this->get_logger(), "[STOP] Dobot ResetRobot result: %d", f.get()->res);
                    } catch (...) {
                        RCLCPP_WARN(this->get_logger(), "[STOP] Dobot ResetRobot failed");
                    }

                    if (clear_error_client_ && clear_error_client_->service_is_ready()) {
                        auto clear_req = std::make_shared<ClearError::Request>();
                        clear_error_client_->async_send_request(clear_req,
                            [this](rclcpp::Client<ClearError>::SharedFuture cf) {
                                try {
                                    RCLCPP_WARN(this->get_logger(), "[STOP] Dobot ClearError result: %d", cf.get()->res);
                                } catch (...) {
                                    RCLCPP_WARN(this->get_logger(), "[STOP] Dobot ClearError failed");
                                }

                                if (enable_client_ && enable_client_->service_is_ready()) {
                                    auto enable_req = std::make_shared<EnableRobot::Request>();
                                    enable_req->load = 0.0;
                                    enable_client_->async_send_request(enable_req,
                                        [this](rclcpp::Client<EnableRobot>::SharedFuture ef) {
                                            try {
                                                RCLCPP_WARN(this->get_logger(), "[STOP] Dobot EnableRobot result: %d", ef.get()->res);
                                            } catch (...) {
                                                RCLCPP_WARN(this->get_logger(), "[STOP] Dobot EnableRobot failed");
                                            }
                                        });
                                } else {
                                    RCLCPP_WARN(this->get_logger(), "[STOP] EnableRobot service not ready");
                                }
                            });
                    } else {
                        RCLCPP_WARN(this->get_logger(), "[STOP] ClearError service not ready");
                    }
                });
        } else {
            RCLCPP_WARN(get_logger(), "[STOP] ResetRobot service not ready");
        }
    }

    void executeAction(const std::shared_ptr<GoalHandleExecuteMotion> goal_handle)
    {
    // Wrap toàn bộ thân trong try/catch: thread này detach, nếu exception unwinds
    // mà không reset motion_in_progress_/publishBusy thì node treo busy vĩnh viễn.
    try {
    RCLCPP_INFO(get_logger(), "[ACTION] Executing motion goal...");
    // Reset abort flag mỗi lần goal mới — flag chỉ giữ trong scope của 1 goal.
    abort_motion_.store(false);

    // ✅ Chỉ clear error khi robot đang ở mode lỗi (mode 9)
    {
        auto mode_req = std::make_shared<RobotMode::Request>();
        auto mode_future = robot_mode_client_->async_send_request(mode_req);
        
        if (mode_future.wait_for(2s) == std::future_status::ready) {
            try {
                auto mode_res = mode_future.get();
                int mode = (mode_res && mode_res->res == 0) ? std::stoi(mode_res->mode) : -1;
                
                if (mode == 9) {
                    RCLCPP_WARN(get_logger(), "[ACTION] Robot in error mode (9) — clearing...");
                    
                    auto clr = std::make_shared<ClearError::Request>();
                    callService<ClearError>(clear_error_client_, clr, "ClearError");
                    std::this_thread::sleep_for(200ms);
                    
                    auto en = std::make_shared<EnableRobot::Request>();
                    en->load = 0.0;
                    callService<EnableRobot>(enable_client_, en, "EnableRobot");
                    std::this_thread::sleep_for(300ms);
                    
                    RCLCPP_INFO(get_logger(), "[ACTION] Robot re-enabled after error clear");
                } else if (mode == 10) {
                    continueIfPaused("ACTION");
                }
                // mode 5 = standby, mode 7 = running — không cần làm gì
            } catch (...) {
                RCLCPP_WARN(get_logger(), "[ACTION] Failed to parse robot mode — proceeding anyway");
            }
        } else {
            RCLCPP_WARN(get_logger(), "[ACTION] RobotMode check timeout — proceeding anyway");
        }
    }

    auto feedback = std::make_shared<ExecuteMotion::Feedback>();
    auto result = std::make_shared<ExecuteMotion::Result>();
    const auto goal = goal_handle->get_goal();

    // motion_in_progress_ đã được claim atomic ở handle_goal — không set lại.
    publishBusy(true);

    feedback->state = "RUNNING";
    feedback->progress = 0.1f;
    goal_handle->publish_feedback(feedback);

    bool success = false;
    std::string type = goal->command;
    int param = goal->slot;

    if (type == "INPUT_TRAY_CHAMBER")  success = executeInputTrayChamber(param);
    else if (type == "INPUT_TRAY_BUFFER") success = executeInputTrayBuffer(param);
    else if (type == "INIT_INPUT_TRAY_BUFFER") success = executeInitInputTrayBuffer(param);
    // AI variants include the deterministic camera-safe checkpoint.  The
    // action only succeeds after index 0 is reached, so vision cannot reopen
    // merely because the pick portion ended or was aborted above the tray.
    else if (type == "AI_INPUT_TRAY_CHAMBER")
        success = executeInputTrayChamber(param) && moveToIndex(0);
    else if (type == "AI_INPUT_TRAY_BUFFER")
        success = executeInputTrayBuffer(param) && moveToIndex(0);
    else if (type == "AI_INIT_INPUT_TRAY_BUFFER")
        success = executeInitInputTrayBuffer(param);  // already ends at index 0
    else if (type == "CHAMBER_SCALE")  success = executeChamberScale();
    else if (type == "SCALE_OUTPUT")   success = executeScaleOutput(param);
    else if (type == "SCALE_FAIL")     success = executeScaleFail();
    else if (type == "BUFFER_CHAMBER") success = executeBufferChamber();
    else if (type == "HOME")           success = moveToIndex(0);
    // MOVE_SCAN_POSE: đưa tay về Index 28 — waypoint sau khi nhấc khỏi input row,
    // KHÔNG che camera khay input. Dùng để re-scan input tray sau REFILL_BUFFER
    // (AI mode) trước khi quyết định pick tiếp hay thay khay.
    else if (type == "MOVE_SCAN_POSE") success = moveToIndex(28);
    else RCLCPP_ERROR(get_logger(), "[ACTION] Unknown command: %s", type.c_str());

    if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "CANCELLED";
        goal_handle->canceled(result);
        publishBusy(false);
        motion_in_progress_ = false;
        return;
    }

    result->success = success;

    if (success) {
        result->message = "COMPLETED";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "[ACTION] Goal succeeded");
    } else {
        result->message = "FAILED";
        if (goal->allow_rollback) {
            feedback->state = "ROLLBACK";
            goal_handle->publish_feedback(feedback);
            rollbackSafe();
            result->message = "ROLLED_BACK";
        }
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "[ACTION] Goal aborted/failed");
    }

    publishBusy(false);
    motion_in_progress_ = false;
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "[ACTION] Exception in executeAction: %s", e.what());
        try {
            auto r = std::make_shared<ExecuteMotion::Result>();
            r->success = false;
            r->message = std::string("EXCEPTION: ") + e.what();
            if (goal_handle && goal_handle->is_active()) goal_handle->abort(r);
        } catch (...) {}
        publishBusy(false);
        motion_in_progress_ = false;
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "[ACTION] Unknown exception in executeAction");
        try {
            auto r = std::make_shared<ExecuteMotion::Result>();
            r->success = false;
            r->message = "UNKNOWN_EXCEPTION";
            if (goal_handle && goal_handle->is_active()) goal_handle->abort(r);
        } catch (...) {}
        publishBusy(false);
        motion_in_progress_ = false;
    }
    }

    void rollbackSafe()
    {
        RCLCPP_ERROR(get_logger(), "[ROLLBACK] Executing SAFE rollback");
        
        // 1. Move to Safe Coordinates (Absolute)
        if (safe_pose_.size() == 6 && (std::abs(safe_pose_[0]) > 0.1 || std::abs(safe_pose_[1]) > 0.1 || std::abs(safe_pose_[2]) > 0.1)) {
            RCLCPP_INFO(get_logger(), "[ROLLBACK] Moving to safe coordinates: %.1f, %.1f, %.1f", 
                        safe_pose_[0], safe_pose_[1], safe_pose_[2]);
            moveL_Absolute(safe_pose_);
        } else {
            RCLCPP_WARN(get_logger(), "[ROLLBACK] Safe pose not set, moving to HOME index 0");
            moveToIndex(0);
        }

        // 2. KHÔNG reset gripper/picker khi rollback — giữ nguyên trạng thái hiện tại
        //    (operator nhấn STOP không được làm rơi khay đang kẹp).
        //    Nếu cần release sau STOP, operator chọn NHẢ trên GUI thủ công.

        RCLCPP_WARN(get_logger(), "[ROLLBACK] Completed (gripper/picker giữ nguyên state)");
    }


    void publishBusy(bool busy)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = busy;
        pub_busy_->publish(msg);
    }


    // Hàm di chuyển theo trục chuẩn của mặt bàn (Base/User 0)
    bool moveBase(double dx, double dy, double dz) {
        if (shouldAbort()) { RCLCPP_WARN(get_logger(), "[moveBase] aborted"); return false; }
        if (!prepareLinearMotion()) return false;
        auto current_pose = getCurrentPose();
        if (current_pose.size() < 6) return false;
        
        auto req = std::make_shared<MovL::Request>();
        req->x  = current_pose[0] + dx;
        req->y  = current_pose[1] + dy;
        req->z  = current_pose[2] + dz;
        req->rx = current_pose[3];
        req->ry = current_pose[4];
        req->rz = current_pose[5];
        req->param_value.clear();

        auto res = callService<MovL>(movl_client_, req, "MovL");
        if (!res || res->res != 0) return false;
        return sync();
    }

    bool executeInputTrayChamber(int row) {
        RCLCPP_INFO(get_logger(), "[MOTION] Input Tray Row %d → Chamber", row);
        if (row < 1 || row > 5) return false;
        if (!moveToIndex(0)) return false;
        // MoveJ đến row1new1 (Index 1)
        if (!moveToIndex(6)) return false;
        if (!moveToIndex(1)) return false;
        
        // Tính tiến theo row index DỰA TRÊN TRỤC CỦA TAY MÁY (Khay đặt theo góc của tay)
        if (row > 1) {
            double dx = (row - 1) * (-104.56); // Đi dọc theo khay (hướng đâm thẳng của tay)
            double dy = (row - 1) * -10.14;      // Đi ngang khay (hướng vuông góc với tay)
            double dz = (row - 1) * 0.7;
            if (!moveR(dx, dy, dz)) return false;
        }
        
        // --- INPUT ROW → CHAMBER: Picker bốc khay từ input stack rồi nhả vào chamber ---
        if (!moveR(0, 0, -109,9)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp khay tại input row
        if (!wait(0.5)) return false;
        if (!moveR(0, 0, 120,8)) return false;
        if (!moveToIndex(28)) return false;
        if (!moveToIndex(29)) return false;
        if (!moveToIndex(7)) return false;
        if (!wait(0.5)) return false;
        if (!moveR(0, 97.5, 0,5)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả khay vào chamber
        if (!wait(0.5)) return false;
        if (!moveR(0, -56, 0)) return false;
        if (!moveR(-10, 20.5, 0,8)) return false;
        if (!wait(0.5)) return false;
        if (!moveR(0, -75, 0)) return false;
        if (!moveToIndex(37)) return false;
        if (!moveToIndex(29)) return false;
        return true;
    }

    bool executeInputTrayBuffer(int row) {
        RCLCPP_INFO(get_logger(), "[MOTION] Input Tray Row %d → Buffer", row);
        if (row < 1 || row > 5) return false;
        if (!moveToIndex(6)) return false; // Tạm thời bỏ move đến vị trí an toàn
        // MoveJ đến row1new1 (Index 1)
        if (!moveToIndex(1)) return false;
        
        // Tính tiến theo row index DỰA TRÊN TRỤC CỦA TAY MÁY (Khay đặt theo góc của tay)
        if (row > 1) {
            double dx = (row - 1) * (-104.56); // Đi dọc theo khay (hướng đâm thẳng của tay)
            double dy = (row - 1) * -10.14;      // Đi ngang khay (hướng vuông góc với tay)
            double dz = (row - 1) * 0.7;
            if (!moveR(dx, dy, dz)) return false;
        }
        
        // --- INPUT ROW → BUFFER: Picker bốc cart từ input stack rồi nhả vào buffer ---
        if (!moveR(0, 0, -109,9)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp khay tại input row
        if (!wait(0.5)) return false;
        if (!moveR(0, 0, 120,8)) return false;
        if (!moveToIndex(28)) return false;
        if (!moveToIndex(8)) return false;
        if (!moveR(0, 0, -54,9)) return false;
        if (!wait(0.2)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả khay vào buffer
        if (!wait(0.5)) return false;
        if (!moveR(0, 0, 140)) return false;
        return true;
    }

    bool executeInitInputTrayBuffer(int row) {
        RCLCPP_INFO(get_logger(), "[MOTION] Init Input Tray Row %d → Buffer → Home", row);
        if (!executeInputTrayBuffer(row)) return false;
        if (!moveToIndex(0)) return false;
        return true;
    }

    bool executeChamberScale() {
        RCLCPP_INFO(get_logger(), "[MOTION] Chamber → Scale");
        if (!moveToIndex(7)) return false;
        if (!moveR(0, 97.5, 0)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp khay tại chamber
        if (!wait(0.9)) return false;
        if (!moveR(0, -70, 0)) return false;
        if (!setDigitalOutput(6, true)) return false;  // 1 nhả loadcell cartridge
        if (!wait(0.2)) return false;
        if (!moveToIndex(9)) return false;
        if (!moveR(0, 0, -104,8)) return false;
        if (!wait(0.5)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả khay lên scale
        if (!wait(0.5)) return false;
        if (!moveR(0, 0, 150)) return false;
        if (!moveToIndex(31)) return false;
        return true;
    }

    bool executeScaleOutput(int slot) {
        RCLCPP_INFO(get_logger(), "[MOTION] Scale → Output Slot %d", slot);
        if (slot < 1 || slot > TOTAL_OUTPUT_SLOTS) return false;
        if (!moveToIndex(10)) return false;
        if (!moveR(0, 0, -89,5)) return false;
        if (!setDigitalOutput(1, true)) return false; // Picker GẮP — kẹp khay đang ở scale
        if (!wait(0.2)) return false;
        if (!moveR(0, 0, 100,8)) return false;
        if (!moveToIndex(11)) return false;      /// buoc dem cho 10
        if (!moveToIndex(32)) return false;
        if (!moveToIndex(33)) return false;
        if (!moveR(0, 0, -38,5)) return false;
        if (!wait(0.2)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — đặt khay xuống vị trí trung gian
        if (!wait(0.2)) return false;
        if (!setDigitalOutput(2, false)) return false;  // Gripper NHẢ — đảm bảo gripper mở trước khi pick up
        if (!wait(0.2)) return false;
        if (!moveToIndex(32)) return false;
        if (!setDigitalOutput(6, false)) return false; //2 kẹp cartridge
        if (!wait(3.3)) return false;
        if (!setDigitalOutput(6, true)) return false;  //3 nhả cartridge
        if (!moveToIndex(12)) return false; 
        if (!moveR(0, 0, -59.5,5)) return false;
        if (!setDigitalOutput(2, true)) return false;   // Gripper GẮP — kẹp khay tại Index 11
        if (!wait(0.5)) return false;
        if (!moveR(0, 0, 70,8)) return false;
        if (!moveToIndex(13)) return false;
        if (!moveToIndex(13 + slot)) return false;
        if (!moveR(0, 0, -130,8)) return false;
        if (!setDigitalOutput(2, false)) return false;  // Gripper NHẢ — thả khay vào output slot
        if (!wait(0.5)) return false;
        if (!moveR(0, 0, 130)) return false;
        if (!moveToIndex(13)) return false;
        if (!moveToIndex(0)) return false;
        return true;
    }

    bool executeScaleFail() {
        RCLCPP_INFO(get_logger(), "[MOTION] Scale → Fail Position %d", current_fail_slot_);
        if (!moveToIndex(23 + current_fail_slot_)) return false;
        current_fail_slot_++;
        if (current_fail_slot_ > 4) current_fail_slot_ = 1;
        if (!moveR(0, 0, -30)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả cart fail vào ngăn loại
        if (!moveR(0, 0, 30)) return false;
        if (!moveToIndex(0)) return false;
        return true;
    }

    bool executeBufferChamber() {
        RCLCPP_INFO(get_logger(), "[MOTION] Buffer → Chamber");
        if (!moveToIndex(8)) return false;
        if (!moveR(0, 0, -59.8)) return false;
        if (!setDigitalOutput(1, true)) return false;   // Picker GẮP — kẹp cart tại buffer
        if (!wait(0.5)) return false;
        if (!moveR(0, 0, 120)) return false;
        if (!moveToIndex(35)) return false;
        if (!moveToIndex(7)) return false;
        if (!wait(0.5)) return false;
        if (!moveR(0, 97.5, 0,5)) return false;
        if (!setDigitalOutput(1, false)) return false;  // Picker NHẢ — thả cart vào chamber
        if (!wait(0.5)) return false;
        if (!moveR(-1, -56, 0)) return false;
        if (!moveR(-10, 20.5, 0,8)) return false;
        if (!wait(0.5)) return false;
        if (!moveR(0, -90, 0)) return false;
        if (!moveToIndex(37)) return false;
        return true;
    }


};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
