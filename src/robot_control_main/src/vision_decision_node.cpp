/**
 * @file vision_decision_node.cpp
 * @brief Handles all YOLO/AI vision processing and ROI-based decision making
 * 
 * Responsibilities:
 * - Subscribe to YOLO bounding boxes (cam0, cam1)
 * - Process ROI detection for input tray rows
 * - Process ROI detection for output tray slots
 * - Publish selected row/slot decisions
 * 
 * Topics Published:
 * - /vision/input_tray/selected_row (Int32)
 * - /vision/input_tray/row_status (Int32MultiArray) - 5 values: 0=empty, 1=full
 * - /vision/input_tray/empty (Bool)
 * - /vision/output_tray/selected_slot (Int32)
 * - /vision/output_tray/slot_status (Int32MultiArray) - 10 values: 0=empty, 1=occupied
 * 
 * Topics Subscribed:
 * - cam0HP/yolo/bounding_boxes (Detection2DArray) - Input tray
 * - cam1HP/yolo/bounding_boxes (Detection2DArray) - Output tray
 * - /robot/set_mode (Int32) - To know current mode (1=AUTO, 2=AI, 3=MANUAL)
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"


#include <vector>
#include <array>
#include <deque>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>
#include <numeric>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ROIQuad + hinh hoc neo ROI theo bbox khay. Tach ra header vi phan nay co the
// — va can duoc — test doc lap voi ROS.
#include "robot_control_main/roi_anchor.hpp"

using vision_msgs::msg::Detection2D;
using vision_msgs::msg::Detection2DArray;

// ============================================================================
// UTILITY STRUCTURES
// ============================================================================

struct RowFilter {
    size_t window = 3;
    int max_fall = 2;
    int ready_consec = 4;

    std::deque<int> hist;
    int last_filtered = 0;
    int ready_streak = 0;

    int filter_count(int raw_count) {
        hist.push_back(raw_count);
        if (hist.size() > window) hist.pop_front();

        std::vector<int> tmp(hist.begin(), hist.end());
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        int med = tmp[tmp.size() / 2];

        int y = med;
        if (y < last_filtered - max_fall) {
            y = last_filtered - max_fall;
        }
        last_filtered = y;
        return y;
    }

    bool update_ready(bool raw_ready) {
        if (raw_ready) ready_streak++;
        else ready_streak = 0;
        return ready_streak >= ready_consec;
    }

    void clear() {
        hist.clear();
        last_filtered = 0;
        ready_streak = 0;
    }
};

enum class SlotStableState : int {
    EMPTY = 0,
    OCC_OK = 1,
    MIS = 2
};

// ============================================================================
// DETECTION HELPERS (Nova5-style: NMS + IoU greedy slot assignment)
// ============================================================================

static double IoU(const Box& A, const Box& B) {
    double xA = std::max(A.x1, B.x1), yA = std::max(A.y1, B.y1);
    double xB = std::min(A.x2, B.x2), yB = std::min(A.y2, B.y2);
    double inter = std::max(0.0, xB - xA) * std::max(0.0, yB - yA);
    double aA = (A.x2 - A.x1) * (A.y2 - A.y1);
    double aB = (B.x2 - B.x1) * (B.y2 - B.y1);
    return inter / std::max(1e-6, aA + aB - inter);
}

static int select_contiguous_empty(const std::vector<int>& empty_slots, int total_slots) {
    std::vector<bool> is_empty(total_slots + 1, false);
    for (int id : empty_slots)
        if (id >= 1 && id <= total_slots) is_empty[id] = true;
    for (int i = 1; i <= total_slots; ++i) {
        if (!is_empty[i]) continue;
        bool all_full_before = true;
        for (int j = 1; j < i; ++j)
            if (is_empty[j]) { all_full_before = false; break; }
        return all_full_before ? i : -1;
    }
    return -1;
}

// [HP] System convention: AUTO=1, AI=2, MANUAL=3 (match robot_controller.cpp
// + cartridge_providesystem). Previous enum had MANUAL=0 → mode 3 from GUI
// fell through to default AUTO branch (fill rows true), making AI threshold
// never apply.
enum class ControlMode : uint8_t {
    AUTO   = 1,
    AI     = 2,
    MANUAL = 3
};

// ============================================================================
// VISION DECISION NODE
// ============================================================================

using namespace std::chrono_literals;

class VisionDecisionNode : public rclcpp::Node {
public:
    VisionDecisionNode() : Node("vision_decision_node") {
        RCLCPP_INFO(get_logger(), "[VISION] === Vision Decision Node Starting ===");
        
        
        // Initialize filters
        row_filters_.assign(5, RowFilter{});
        row_full_.assign(5, false);
        
        // Publishers
        pub_selected_row_ = create_publisher<std_msgs::msg::Int32>(
            "/vision/input_tray/selected_row", 10);
        pub_ai_row_ = create_publisher<std_msgs::msg::Int32>(
            "/camera/ai/selected_row", 10);
        pub_row_status_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/vision/input_tray/row_status", 10);
        pub_input_empty_ = create_publisher<std_msgs::msg::Bool>(
            "/vision/input_tray/empty", 10);
        // [HP] Layer-1 tray-presence check: true khi có khay trong khung hình
        pub_input_present_ = create_publisher<std_msgs::msg::Bool>(
            "/vision/input_tray/present", 10);
        pub_selected_slot_ = create_publisher<std_msgs::msg::Int32>(
            "/vision/output_tray/selected_slot", 10);
        pub_ai_slot_ = create_publisher<std_msgs::msg::Int32>(
            "/camera/ai/selected_slot", 10);
        pub_slot_status_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/vision/output_tray/slot_status", 10);
        pub_output_present_ = create_publisher<std_msgs::msg::Bool>(
            "/vision/output_tray/present", 10);
        pub_heartbeat_ = create_publisher<std_msgs::msg::Header>(
            "/vision/heartbeat", 10);
        // Topic rieng, KHONG dung chung /robot/error: robot_logic ban
        // EMERGENCY STOP / MOTION_* vao do, publish lap lai se de mat.
        // Latched de GUI bat sau van thay ngay trang thai ROI.
        pub_roi_status_ = create_publisher<std_msgs::msg::String>(
            "/vision/roi_status", rclcpp::QoS(1).transient_local());

        // Phai xong TRUOC khi subscribe: callback khong duoc phep chay voi ROI
        // chua load. Hien spin() chi bat dau sau constructor nen chua the xay
        // ra, nhung dung phu thuoc vao dieu do.
        loadROIs();
        publishRoiStatus();

        // Tray change publishers
        // Camera → Cartridge trực tiếp (AI mode output tray full)
        pub_change_tray_output_ = create_publisher<std_msgs::msg::Bool>(
            "/robot/done_tray_output", 10);
        // Camera → Robot logic (để robot biết tray đang thay)
        pub_output_tray_full_ = create_publisher<std_msgs::msg::Bool>(
            "/vision/output_tray/full", 10);

        // Ghi truc tiep vao LOG ACTIVITY cua CartridgePage: GUI da subscribe
        // /providesystem/gui_notify (JSON level/title/detail). Banner da tat
        // trong QML nen level "warn" chi hien dong do trong Activity Log.
        pub_gui_notify_ = create_publisher<std_msgs::msg::String>(
            "/providesystem/gui_notify", 10);

        
        // Subscriptions with SensorDataQoS for low latency
        sub_cam0_ = create_subscription<Detection2DArray>(
            "cam0HP/yolo/bounding_boxes", rclcpp::SensorDataQoS(),
            std::bind(&VisionDecisionNode::camera1Callback, this, std::placeholders::_1));
            
        sub_cam1_ = create_subscription<Detection2DArray>(
            "cam1HP/yolo/bounding_boxes", rclcpp::SensorDataQoS(),
            std::bind(&VisionDecisionNode::camera2Callback, this, std::placeholders::_1));
        
        // Mode subscription
        sub_mode_ = create_subscription<std_msgs::msg::Int32>(
            "/robot/set_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                current_mode_ = static_cast<ControlMode>(msg->data);
                RCLCPP_INFO(get_logger(), "[VISION] Mode changed to: %d", msg->data);
            });

        inx_camera_position_mm_ =
            declare_parameter<double>("inx_camera_position_mm", -60.0);
        inx_camera_tolerance_mm_ =
            declare_parameter<double>("inx_camera_tolerance_mm", 2.0);
        sub_servo_positions_ = create_subscription<std_msgs::msg::String>(
            "/providesystem/servo_positions", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                bool in_range = false;
                double inx = 0.0;
                try {
                    const YAML::Node positions = YAML::Load(msg->data);
                    if (positions["1"]) {
                        inx = positions["1"].as<double>();
                        in_range = std::abs(inx - inx_camera_position_mm_)
                                   <= inx_camera_tolerance_mm_;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                        "[VISION] Khong parse duoc servo_positions: %s", e.what());
                }

                inx_position_last_ns_.store(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                const bool was_ready = inx_camera_ready_.load();
                if (in_range) {
                    if (++inx_ready_streak_ >= INX_READY_CONFIRM_SAMPLES)
                        inx_camera_ready_.store(true);
                } else {
                    inx_ready_streak_ = 0;
                    inx_camera_ready_.store(false);
                }
                const bool now_ready = inx_camera_ready_.load();
                if (was_ready != now_ready) {
                    RCLCPP_WARN(get_logger(),
                        "[VISION] Cam0 gate INX: %.2fmm -> %s (target %.2f +/- %.2fmm)",
                        inx, now_ready ? "READY" : "BLOCKED",
                        inx_camera_position_mm_, inx_camera_tolerance_mm_);
                }
            });

        sub_input_scan_allowed_ = create_subscription<std_msgs::msg::Bool>(
            "/vision/input_scan_allowed",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                const bool was_allowed = input_scan_allowed_.exchange(msg->data);
                if (!was_allowed && msg->data) {
                    // Input pick vua ket thuc: cho YOLO nap bbox sach. Cac
                    // motion chamber/loadcell/output sau do KHONG khoa scan.
                    vision_resume_after_ =
                        std::chrono::steady_clock::now() + 1500ms;
                }
                if (!msg->data) {
                    input_empty_streak_ = 0;
                    input_tray_empty_ = false;
                    auto empty = std_msgs::msg::Bool();
                    empty.data = false;
                    pub_input_empty_->publish(empty);
                }
                RCLCPP_INFO(get_logger(), "[VISION] Input scan %s",
                    msg->data ? "ENABLED" : "BLOCKED during input pick");
            });

        sub_output_scan_allowed_ = create_subscription<std_msgs::msg::Bool>(
            "/vision/output_scan_allowed",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                const bool was_allowed = output_scan_allowed_.exchange(msg->data);
                if (!was_allowed && msg->data) {
                    output_resume_after_ =
                        std::chrono::steady_clock::now() + 1500ms;
                }
                if (!msg->data) {
                    std::lock_guard<std::mutex> lock(slot_detection_mutex_);
                    selected_output_slot_ = -1;
                    slot_stable_state_.fill(SlotStableState::OCC_OK);
                    slot_empty_streak_.fill(0);
                    slot_occ_streak_.fill(0);
                    slot_mis_streak_.fill(0);
                    output_full_streak_ = 0;
                }
                RCLCPP_INFO(get_logger(), "[VISION] Output scan %s",
                    msg->data ? "ENABLED" : "BLOCKED during place/full");
            });

        sub_new_input_tray_ = create_subscription<std_msgs::msg::Bool>(
            "/cartridge_providesystem/new_tray_loaded", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                input_empty_streak_ = 0;
                input_tray_empty_ = false;
                if (!msg->data) {
                    // Physical tray removal is authoritative and remains valid
                    // even while robot-occlusion gating freezes camera frames.
                    input_present_streak_ = 0;
                    input_tray_present_ = false;
                    selected_input_row_ = -1;
                    std::fill(row_full_.begin(), row_full_.end(), false);
                    for (auto& filter : row_filters_) filter.clear();

                    auto row = std_msgs::msg::Int32();
                    row.data = -1;
                    pub_selected_row_->publish(row);
                    pub_ai_row_->publish(row);
                    auto status = std_msgs::msg::Int32MultiArray();
                    status.data.assign(5, 0);
                    pub_row_status_->publish(status);
                    auto present = std_msgs::msg::Bool();
                    present.data = false;
                    pub_input_present_->publish(present);
                    RCLCPP_INFO(get_logger(),
                        "[VISION] Input tray removed -> cleared READY snapshot");
                }
                auto empty = std_msgs::msg::Bool();
                empty.data = false;
                pub_input_empty_->publish(empty);
                if (!msg->data) return;
                RCLCPP_INFO(get_logger(),
                    "[VISION] New input tray -> reset/clear EMPTY debounce");
            });


        // Heartbeat Timer
        heartbeat_timer_ = create_wall_timer(500ms, [this]() {
            std_msgs::msg::Header h;
            h.stamp = this->now();
            h.frame_id = "vision_decision";
            pub_heartbeat_->publish(h);
        });



        RCLCPP_INFO(get_logger(), "[VISION] === Vision Decision Node Ready ===");
    }

private:
    // ========================================================================
    // CONSTANTS
    // ========================================================================
    static constexpr int    INPUT_ROW_THRESHOLD    = 8;
    static constexpr float  DETECTION_SCORE_THRESH = 0.60f;
    // Output tray frame (cam1 class 0) only. The model scores the empty roller
    // conveyor as a tray at around 0.83 — see the 0.83 false frame captured on
    // 2026-08-08 with no tray present — so the tray class needs a higher bar
    // than the cartridges do. Deliberately NOT applied to cartridge/
    // cartridgefall: a genuine cartridge scoring between 0.60 and 0.86 must
    // still mark its slot occupied, or the robot would place on top of it.
    // Double, not float: hypothesis.score is float64, and a float copy of 0.86
    // rounds up, which would reject a detection scoring exactly 0.86.
    static constexpr double OUTPUT_TRAY_SCORE_THRESH = 0.86;
    static constexpr int    SLOT_CONFIRM_FRAMES    = 2;
    static constexpr int    INPUT_EMPTY_CONFIRM_FRAMES = 15;
    static constexpr int    INX_READY_CONFIRM_SAMPLES = 3;
    static constexpr int64_t INX_POSITION_MAX_AGE_NS = 500000000LL;
    // Phai khop: nut O1-O10 tren GUI, pose index 14-23 trong
    // joint_pose_params.yaml, va so slot trong config/vision_roi.yaml.
    static constexpr size_t N_OUTPUT_SLOTS         = 10;

    // ========================================================================
    // MODE STATE
    // ========================================================================
    // [HP] Default AI để khi vision khởi động trước GUI mode publish, vẫn dùng
    // YOLO threshold thay vì AUTO branch (fill rows true bỏ qua YOLO).
    std::atomic<ControlMode> current_mode_{ControlMode::AI};
    std::atomic<bool> inx_camera_ready_{false};
    std::atomic<int64_t> inx_position_last_ns_{0};
    int inx_ready_streak_{0};
    double inx_camera_position_mm_{-60.0};
    double inx_camera_tolerance_mm_{2.0};
    std::atomic<bool> input_scan_allowed_{true};
    std::chrono::steady_clock::time_point vision_resume_after_{};
    std::atomic<bool> output_scan_allowed_{true};
    std::chrono::steady_clock::time_point output_resume_after_{};

    // ========================================================================
    // INPUT TRAY STATE
    // ========================================================================
    std::vector<ROIQuad> input_tray_rois_;
    std::vector<RowFilter> row_filters_;
    std::vector<bool> row_full_;
    std::atomic<bool> input_tray_empty_{false};
    int input_empty_streak_{0};
    int selected_input_row_{-1};
    // [HP] Layer-1: tray-present ROI (bao toàn bộ khay) + state
    ROIQuad input_tray_outer_roi_;
    std::atomic<bool> input_tray_present_{false};
    int input_present_streak_{0};   // debounce class-0 tray bbox (2 frame on, instant off)
    TrayAnchorTracker input_anchor_;

    // ========================================================================
    // OUTPUT TRAY STATE
    // ========================================================================
    ROIQuad output_tray_outer_roi_;
    std::array<ROIQuad, N_OUTPUT_SLOTS> output_tray_rois_;
    std::array<SlotStableState, N_OUTPUT_SLOTS> slot_stable_state_;
    std::array<int, N_OUTPUT_SLOTS> slot_empty_streak_;
    std::array<int, N_OUTPUT_SLOTS> slot_occ_streak_;
    std::array<int, N_OUTPUT_SLOTS> slot_mis_streak_;
    std::atomic<bool> output_tray_present_{false};
    int output_tray_present_streak_{0};
    int selected_output_slot_{-1};
    std::mutex slot_detection_mutex_;
    TrayAnchorTracker output_anchor_;

    // ROI neo theo bbox khay, bat rieng tung khay. false = ROI tinh nhu cu.
    bool input_anchor_enabled_{false};
    bool output_anchor_enabled_{false};

    // Rong = ROI OK. Non-empty -> GUI hien canh bao.
    std::string roi_error_;

    // ========================================================================
    // PERFORMANCE TRACKING
    // ========================================================================
    std::atomic<uint64_t> callback_count_{0};
    std::atomic<uint64_t> total_callback_time_us_{0};

    // ========================================================================
    // ROS INTERFACES
    // ========================================================================
    rclcpp::Subscription<Detection2DArray>::SharedPtr sub_cam0_;
    rclcpp::Subscription<Detection2DArray>::SharedPtr sub_cam1_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_scan_allowed_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_output_scan_allowed_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_new_input_tray_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_servo_positions_;
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_selected_row_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_ai_row_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_row_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_input_empty_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_input_present_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_selected_slot_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_ai_slot_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_slot_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_output_present_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_heartbeat_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_roi_status_;
    
    // Tray change publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_change_tray_output_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_output_tray_full_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_gui_notify_;
    
    // Debounce counters for tray change
    std::atomic<int> output_full_streak_{0};
    std::atomic<bool> tray_output_change_sent_{false};
    static constexpr int TRAY_CHANGE_CONFIRM_FRAMES = 10;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    // ROIs live in config/vision_roi.yaml and retain the original 640x480
    // picking coordinates. The file records ref_width/ref_height; we scale
    // them to the current 640x360 camera/bbox space so the 16:9 output does
    // not require manually picking every ROI again.
    ROIQuad quadFromYaml(const YAML::Node& n, double sx, double sy) {
        if (!n.IsSequence() || n.size() != 4)
            throw std::runtime_error("ROI phai co dung 4 goc");
        std::vector<std::pair<int, int>> corners;
        for (const auto& p : n) {
            corners.emplace_back(
                static_cast<int>(std::lround(p[0].as<double>() * sx)),
                static_cast<int>(std::lround(p[1].as<double>() * sy)));
        }
        return ROIQuad::FromCorners(corners);
    }

    // `anchor: [x1, y1, x2, y2]` — AABB cua bbox class-0 tren dung khung hinh
    // da dung de cham ROI. Thieu khoa nay thi tray-anchor khong bat duoc.
    RoiAnchor anchorFromYaml(const YAML::Node& n, double sx, double sy) {
        RoiAnchor a{};
        if (!n || !n.IsSequence() || n.size() != 4) return a;
        const double x1 = n[0].as<double>() * sx;
        const double y1 = n[1].as<double>() * sy;
        const double x2 = n[2].as<double>() * sx;
        const double y2 = n[3].as<double>() * sy;
        a.x1 = static_cast<float>(std::min(x1, x2));
        a.y1 = static_cast<float>(std::min(y1, y2));
        a.x2 = static_cast<float>(std::max(x1, x2));
        a.y2 = static_cast<float>(std::max(y1, y2));
        a.valid = (a.w() > 1.0F && a.h() > 1.0F);
        return a;
    }

    // Tra ve bbox class-0 diem cao nhat co tam nam trong `gate`. Ban Pi lay
    // detection class-0 DAU TIEN gap duoc; mot false-positive tray o mep anh la
    // du keo lech toan bo ROI, nen o day chon theo diem.
    static bool findTrayRect(const Detection2DArray& msg, double score_thresh,
                             const ROIQuad& gate, Box& out) {
        bool found = false;
        double best_score = -1.0;
        for (const auto& det : msg.detections) {
            if (det.results.empty()) continue;
            const auto& h = det.results[0].hypothesis;
            if (h.class_id != "0" || h.score < score_thresh) continue;
            const float cx = static_cast<float>(det.bbox.center.position.x);
            const float cy = static_cast<float>(det.bbox.center.position.y);
            if (!gate.bbox_contains(cx, cy) || !gate.contains(cx, cy)) continue;
            if (h.score <= best_score) continue;
            best_score = h.score;
            const double hw = det.bbox.size_x / 2.0;
            const double hh = det.bbox.size_y / 2.0;
            out = Box{cx - hw, cy - hh, cx + hw, cy + hh};
            found = true;
        }
        return found;
    }

    void logAnchor(const char* tag, bool enabled, const RoiTransform& tf,
                   bool tray_seen, const TrayAnchorTracker& tracker) {
        if (!enabled) return;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "[VISION] %s anchor: %s scale=%.3f,%.3f offset=%.1f,%.1f "
            "tray_seen=%d rejected=%d",
            tag, tf.identity ? "CHUA HOI TU (dang dung ROI tinh)" : "active",
            tf.sx, tf.sy, tf.ox, tf.oy, tray_seen ? 1 : 0,
            tracker.rejected_streak());
    }

    void loadROIs() {
        const std::string default_path =
            ament_index_cpp::get_package_share_directory("robot_control_main")
            + "/config/vision_roi.yaml";
        const std::string path = declare_parameter<std::string>("roi_config", default_path);
        const int img_w = declare_parameter<int>("image_width", 640);
        const int img_h = declare_parameter<int>("image_height", 360);
        // "static" = ROI tinh nhu truoc. "tray" = neo ROI theo bbox khay.
        // Mac dinh static: bat sang tray la doi hanh vi vision, phai la quyet
        // dinh co y sau khi da doi chieu bang roi_preview.py.
        const std::string anchor_mode =
            declare_parameter<std::string>("roi_anchor_mode", "static");
        const double anchor_alpha =
            declare_parameter<double>("roi_anchor_alpha", 0.25);
        const double anchor_max_dev =
            declare_parameter<double>("roi_anchor_max_scale_dev", 0.25);

        slot_stable_state_.fill(SlotStableState::EMPTY);
        slot_empty_streak_.fill(0);
        slot_occ_streak_.fill(0);
        slot_mis_streak_.fill(0);

        try {
            YAML::Node cfg = YAML::LoadFile(path);
            const double ref_w = cfg["ref_width"].as<double>();
            const double ref_h = cfg["ref_height"].as<double>();
            const double sx = img_w / ref_w, sy = img_h / ref_h;
            if (sx != 1.0 || sy != 1.0) {
                RCLCPP_WARN(get_logger(),
                    "[VISION] ROI scale %.3f x %.3f (ref %.0fx%.0f -> image %dx%d)"
                    " — kiem tra image_width/height co khop bbox thuc te",
                    sx, sy, ref_w, ref_h, img_w, img_h);
            }

            input_tray_outer_roi_ = quadFromYaml(cfg["input_tray"]["outer"], sx, sy);
            input_tray_rois_.clear();
            for (const auto& row : cfg["input_tray"]["rows"])
                input_tray_rois_.push_back(quadFromYaml(row, sx, sy));

            output_tray_outer_roi_ =
                quadFromYaml(cfg["output_tray"]["outer"], sx, sy);
            size_t n_slots = 0;
            for (const auto& slot : cfg["output_tray"]["slots"]) {
                if (n_slots >= N_OUTPUT_SLOTS) break;
                output_tray_rois_[n_slots++] = quadFromYaml(slot, sx, sy);
            }
            // Slot thieu ROI -> ep OCC_OK de select_contiguous_empty khong bao
            // gio chon no (robot khong duoc dat vao slot chua dinh nghia).
            for (size_t i = n_slots; i < N_OUTPUT_SLOTS; ++i)
                slot_stable_state_[i] = SlotStableState::OCC_OK;
            if (n_slots < N_OUTPUT_SLOTS) {
                RCLCPP_ERROR(get_logger(),
                    "[VISION] %s chi co %zu/%zu slot — cac slot thieu bi khoa (coi nhu day)",
                    path.c_str(), n_slots, N_OUTPUT_SLOTS);
                addRoiError("thieu slot: " + std::to_string(n_slots) + "/"
                            + std::to_string(N_OUTPUT_SLOTS) + " (slot thieu bi khoa)");
            }
            if (input_tray_rois_.size() != 5) {
                RCLCPP_ERROR(get_logger(), "[VISION] ROI chi co %zu/5 row",
                             input_tray_rois_.size());
                addRoiError("thieu row: " + std::to_string(input_tray_rois_.size()) + "/5");
            }

            const RoiAnchor in_anchor =
                anchorFromYaml(cfg["input_tray"]["anchor"], sx, sy);
            const RoiAnchor out_anchor =
                anchorFromYaml(cfg["output_tray"]["anchor"], sx, sy);
            input_anchor_.configure(in_anchor, anchor_alpha, anchor_max_dev);
            output_anchor_.configure(out_anchor, anchor_alpha, anchor_max_dev);

            // Hai khay doc lap nhau: thieu anchor ben nay khong co ly do gi
            // phai tat luon ben kia. Cho phep chay tung khay mot de trien khai
            // dan, khay nao chua co anchor thi chay ROI tinh nhu cu.
            const bool want_anchor = (anchor_mode == "tray");
            input_anchor_enabled_ = want_anchor && in_anchor.valid;
            output_anchor_enabled_ = want_anchor && out_anchor.valid;
            if (want_anchor && !(in_anchor.valid && out_anchor.valid)) {
                // Bao ra ngoai chu khong im lang: van hanh phai biet khay nao
                // dang duoc bu lech va khay nao thi khong.
                RCLCPP_WARN(get_logger(),
                    "[VISION] roi_anchor_mode=tray nhung %s thieu 'anchor' "
                    "(input=%d output=%d) — khay thieu chay ROI tinh",
                    path.c_str(), in_anchor.valid ? 1 : 0, out_anchor.valid ? 1 : 0);
                addRoiError(std::string("thieu anchor: ") +
                            (in_anchor.valid ? "" : "input_tray ") +
                            (out_anchor.valid ? "" : "output_tray ") +
                            "— khay do dang chay ROI tinh");
            }
            if (anchor_mode != "tray" && anchor_mode != "static") {
                RCLCPP_WARN(get_logger(),
                    "[VISION] roi_anchor_mode='%s' khong hop le — dung 'static'",
                    anchor_mode.c_str());
            }

            RCLCPP_INFO(get_logger(),
                "[VISION] ROI loaded: 2 outer + %zu rows + %zu/%zu slots tu %s"
                " | anchor cam0=%s cam1=%s alpha=%.2f max_scale_dev=%.2f",
                input_tray_rois_.size(), n_slots, N_OUTPUT_SLOTS, path.c_str(),
                input_anchor_enabled_ ? "tray" : "static",
                output_anchor_enabled_ ? "tray" : "static",
                anchor_alpha, anchor_max_dev);
        } catch (const std::exception& e) {
            // Fail-safe on trung tinh: khong row -> khong bao gio chon row;
            // moi slot OCC_OK -> khong bao gio chon slot. Node van song de
            // heartbeat/GUI thay loi thay vi crash-respawn loop.
            RCLCPP_ERROR(get_logger(),
                "[VISION] KHONG load duoc ROI (%s): %s — vision decision bi khoa toan bo",
                path.c_str(), e.what());
            input_tray_rois_.clear();
            input_tray_outer_roi_ = ROIQuad{};
            output_tray_outer_roi_ = ROIQuad{};
            input_anchor_enabled_ = false;
            output_anchor_enabled_ = false;
            input_anchor_.configure(RoiAnchor{}, anchor_alpha, anchor_max_dev);
            output_anchor_.configure(RoiAnchor{}, anchor_alpha, anchor_max_dev);
            slot_stable_state_.fill(SlotStableState::OCC_OK);
            addRoiError(std::string("KHONG load duoc — vision bi khoa: ") + e.what());
        }
    }

    // Gop nhieu loi thay vi de cai sau de cai truoc: YAML thieu ca row lan slot
    // thi phai thay ca hai, khong thi sua xong cai nay moi lo ra cai kia.
    void addRoiError(const std::string& msg) {
        roi_error_ += (roi_error_.empty() ? "ROI " : " | ") + msg;
    }

    void publishRoiStatus() {
        std_msgs::msg::String m;
        m.data = roi_error_;   // rong = OK
        pub_roi_status_->publish(m);
    }

    bool isInxCameraReady() const {
        if (!inx_camera_ready_.load()) return false;
        const int64_t now_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        const int64_t last_ns = inx_position_last_ns_.load();
        return last_ns > 0 && (now_ns - last_ns) <= INX_POSITION_MAX_AGE_NS;
    }

    void publishInputGateBlocked() {
        // Reset decision internals, but publish nothing while blocked.  The GUI
        // must keep the last trustworthy camera snapshot (for example row 4
        // and row 5 READY) while the robot is consuming row 4.  Publishing
        // zero status here made row 5 incorrectly disappear from the GUI.
        input_empty_streak_ = 0;
        input_present_streak_ = 0;
        input_tray_empty_ = false;
        input_tray_present_ = false;
        selected_input_row_ = -1;
        std::fill(row_full_.begin(), row_full_.end(), false);
        for (auto& filter : row_filters_) filter.clear();
    }

    // ========================================================================
    // INPUT TRAY CALLBACK (from robot_logic_node.cpp)
    // ========================================================================
    void camera1Callback(const Detection2DArray::SharedPtr msg) {
        auto start = std::chrono::high_resolution_clock::now();
        
        if (current_mode_ == ControlMode::MANUAL) return;
        bool use_ai = (current_mode_ == ControlMode::AI);

        // Khay chi dung he toa do camera khi InX da ve vi tri safe -60mm.
        // Ngoai vi tri nay moi detection cam0 deu co the la khay dang di chuyen.
        if (use_ai && (!isInxCameraReady() ||
                       !input_scan_allowed_.load() ||
                       std::chrono::steady_clock::now() < vision_resume_after_)) {
            publishInputGateBlocked();
            return;
        }

        int total_in_tray = 0;
        if (!use_ai) {
            // AUTO: skip YOLO entirely, assume tray present + all rows full
            input_tray_present_.store(true);
            std::fill(row_full_.begin(), row_full_.end(), true);
        } else {
            // [HP Layer-1] TRAY ROI = spatial filter — chỉ giữ detection BÊN TRONG
            // khay vận hành. Loại nhiễu/khay khác cạnh ra khỏi row counting.
            // [HP Layer-2] Row counting áp dụng RowFilter cho stability.
            std::vector<int> row_counts(5, 0);
            bool tray_frame_seen = false;

            // Pass 0: neo ROI theo bbox khay. Cong doan nay chay TRUOC vong dem
            // cartridge vi row ROI cua vong do phai la ROI da bu lech.
            Box tray_rect{};
            const bool tray_rect_found = findTrayRect(
                *msg, DETECTION_SCORE_THRESH, input_tray_outer_roi_, tray_rect);
            const RoiTransform tf = input_anchor_enabled_
                ? input_anchor_.update(tray_rect_found ? &tray_rect : nullptr)
                : RoiTransform{};
            const ROIQuad outer_roi = transformQuad(input_tray_outer_roi_, tf);
            std::vector<ROIQuad> row_rois;
            row_rois.reserve(input_tray_rois_.size());
            for (const auto& r : input_tray_rois_)
                row_rois.push_back(transformQuad(r, tf));
            logAnchor("cam0", input_anchor_enabled_, tf, tray_rect_found, input_anchor_);

            for (const auto& det : msg->detections) {
                if (det.results.empty()) continue;
                const std::string& class_id = det.results[0].hypothesis.class_id;
                float score = det.results[0].hypothesis.score;
                float cx = det.bbox.center.position.x;
                float cy = det.bbox.center.position.y;
                // [HP HEF] class "0" = whole TRAY bbox — nguon DUY NHAT xac dinh
                // khay co mat (giong camera2Callback ben output). TUYET DOI khong
                // suy "khay co mat" tu so cartridge: khay pick sach (0 cartridge)
                // se thanh present=false, empty streak reset mai va khong bao gio
                // chot duoc done_tray_input -> khong thay khay.
                if (class_id == "0" && score >= DETECTION_SCORE_THRESH) {
                    // Co y dung outer ROI TINH: neu gate tray-present chay tren
                    // ROI da bu theo chinh bbox tray thi thanh vong lap kin, ROI
                    // co the truot theo mot detection sai roi tu xac nhan minh.
                    if (input_tray_outer_roi_.bbox_contains(cx, cy))
                        tray_frame_seen = true;
                    continue;
                }
                // [HP HEF6] class "1" = CARTRIDGE.
                if (class_id != "1" || score < DETECTION_SCORE_THRESH) continue;

                // L1: spatial filter — bỏ nếu nằm ngoài TRAY ROI
                if (!outer_roi.bbox_contains(cx, cy)) continue;
                if (!outer_roi.contains(cx, cy)) continue;
                total_in_tray++;

                // L2: tìm row ROI khớp
                for (size_t i = 0; i < row_rois.size(); ++i) {
                    if (!row_rois[i].bbox_contains(cx, cy)) continue;
                    if (row_rois[i].contains(cx, cy)) { row_counts[i]++; break; }
                }
            }

            // Khay bi nhac ra -> quen vi tri da hoi tu. Khay moi dat vao co the
            // lech han so voi khay cu, keo transform cu sang la sai ngay khung
            // hinh dau tien.
            if (!tray_frame_seen) input_anchor_.reset();

            // Tray present = thay bbox KHUNG KHAY (class 0) trong TRAY ROI.
            // Can 2 frame de xac nhan khi dat khay vao; mat detection thi khoa
            // ngay (fail-safe, giong output_tray_present_ ben camera2Callback).
            if (tray_frame_seen) {
                input_present_streak_++;
                if (input_present_streak_ >= SLOT_CONFIRM_FRAMES)
                    input_tray_present_.store(true);
            } else {
                input_present_streak_ = 0;
                input_tray_present_.store(false);
            }

            for (size_t i = 0; i < 5; ++i) {
                int raw_count  = row_counts[i];
                int filtered   = row_filters_[i].filter_count(raw_count);
                // [HP] Row READY chỉ khi ĐÚNG bằng sức chứa (8) — STRICT theo yêu
                // cầu vận hành: máy fill cần đủ 8 cartridge/hàng mới đủ ÁP SUẤT ÂM
                // để hoạt động. count > 8 là nhân đôi/nhiễu detection, count < 8 là
                // hàng thiếu → cả hai NOT ready (bỏ qua hàng, coi như empty dù còn
                // cartridge lẻ). Không row nào ready → thay khay (by design).
                // A row can only be READY inside a confirmed physical tray.
                // Previously the row count alone was sufficient, so detections
                // from the empty fixture/background could light R4/R5 even
                // though class-0 tray presence was false.
                bool raw_ready = input_tray_present_.load() &&
                                 (raw_count == INPUT_ROW_THRESHOLD);
                bool stable    = row_filters_[i].update_ready(raw_ready);
                row_full_[i]   = stable;
                RCLCPP_DEBUG(get_logger(),
                    "[ROW %zu] raw=%d filtered=%d thr=%d READY(raw)=%s READY(stable)=%s (streak=%d/%d) [tray_total=%d]",
                    i + 1, raw_count, filtered, INPUT_ROW_THRESHOLD,
                    raw_ready ? "YES" : "NO", stable ? "YES" : "NO",
                    row_filters_[i].ready_streak, row_filters_[i].ready_consec,
                    total_in_tray);
            }
        }
        
        // EMPTY (= can thay khay) ⟺ KHUNG KHAY co mat + KHONG row nao ready,
        // giu lien tiep INPUT_EMPTY_CONFIRM_FRAMES frame. Theo yeu cau van hanh:
        // row <8 cartridge bi bo qua (khong du ap suat am cho may fill), nen khay
        // 5 hang x 7 cai => khong row nao ready => THAY KHAY du con cartridge le.
        // Cac lop gate truoc khi duoc dem frame:
        //   - isInxCameraReady(): InX phai o vi tri check camera -60mm (camera
        //     gan tren cum servo, dang di chuyen thi detection khong tin duoc)
        //   - input_scan_allowed_: khoa rieng trong 3 motion pick input; cac
        //     motion chamber/loadcell/output khong lam cham quyet dinh AI
        //   - vision_resume_after_: 1500ms sau khi input pick ket thuc
        //   - RowFilter warm-up: khong bao EMPTY khi raw da thay 8/row nhung
        //     stable chua kip len (tung lam done_tray_input som).
        const bool any_row_ready = std::any_of(
            row_full_.begin(), row_full_.end(), [](bool full) { return full; });
        const bool safe_to_conclude_empty =
            input_scan_allowed_.load() &&
            std::chrono::steady_clock::now() >= vision_resume_after_;
        if (use_ai && safe_to_conclude_empty &&
            input_tray_present_.load() && !any_row_ready) {
            input_empty_streak_++;
        } else {
            input_empty_streak_ = 0;
        }
        const bool was_empty_confirmed = input_tray_empty_;
        input_tray_empty_ =
            (input_empty_streak_ >= INPUT_EMPTY_CONFIRM_FRAMES);
        if (input_tray_empty_ && !was_empty_confirmed && total_in_tray > 0) {
            // Tray change with cartridges remaining is NORMAL / BY DESIGN
            // (rows with <8 cartridges are skipped — fill machine needs a full
            // row of 8 for negative pressure). Trace it as a plain activity
            // entry: level "silent_ok" -> normal line in Activity Log, no red,
            // no banner.
            RCLCPP_INFO(get_logger(),
                "[VISION] 📥 INPUT EMPTY confirmed: no row has %d/%d cartridges — "
                "tray change requested with %d cartridges left on tray",
                INPUT_ROW_THRESHOLD, INPUT_ROW_THRESHOLD, total_in_tray);
            const std::string detail =
                "No row has " + std::to_string(INPUT_ROW_THRESHOLD) + "/" +
                std::to_string(INPUT_ROW_THRESHOLD) +
                " cartridges - tray change requested with " +
                std::to_string(total_in_tray) + " cartridges left on tray";
            std_msgs::msg::String notify;
            notify.data =
                "{\"level\":\"silent_ok\",\"title\":\"INPUT TRAY CHANGE\",\"detail\":\"" +
                detail + "\"}";
            pub_gui_notify_->publish(notify);
        }
        
        // Find first available (full) row for picking
        selected_input_row_ = -1;
        for (size_t i = 0; i < row_full_.size(); ++i) {
            if (row_full_[i]) {
                selected_input_row_ = static_cast<int>(i + 1);  // 1-indexed
                break;
            }
        }
        
        // Publish results
        auto row_msg = std_msgs::msg::Int32();
        row_msg.data = selected_input_row_;
        pub_selected_row_->publish(row_msg);
        pub_ai_row_->publish(row_msg);
        
        auto empty_msg = std_msgs::msg::Bool();
        empty_msg.data = input_tray_empty_;
        pub_input_empty_->publish(empty_msg);

        auto present_msg = std_msgs::msg::Bool();
        present_msg.data = input_tray_present_.load();
        pub_input_present_->publish(present_msg);
        
        auto status_msg = std_msgs::msg::Int32MultiArray();
        status_msg.data.resize(5);
        for (size_t i = 0; i < 5; ++i) {
            status_msg.data[i] = row_full_[i] ? 1 : 0;
        }
        pub_row_status_->publish(status_msg);

        
        // Performance tracking
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        callback_count_.fetch_add(1);
        total_callback_time_us_.fetch_add(duration_us);
        
        if (callback_count_ % 100 == 0) {
            uint64_t avg_us = total_callback_time_us_ / callback_count_;
            RCLCPP_INFO(get_logger(), "[PERF] Cam0 avg: %lu µs", avg_us);
        }
    }

    // ========================================================================
    // OUTPUT TRAY CALLBACK (from robot_logic_node.cpp)
    // ========================================================================
    void camera2Callback(const Detection2DArray::SharedPtr msg) {
        if (current_mode_ == ControlMode::MANUAL) return;
        
        if (current_mode_ == ControlMode::AUTO) {
            // AUTO: Robot logic tự quản lý slot, camera không can thiệp
            return;
        }

        if (!output_scan_allowed_.load() ||
            std::chrono::steady_clock::now() < output_resume_after_) {
            // Freeze the last trustworthy GUI snapshot during robot occlusion.
            // Internal slot state was reset by the gate callback and a fresh
            // decision will be published after the cooldown.
            return;
        }

        // Layer 1: model cam1 co 3 class, class 0 la toan bo output tray.
        // Khong co tray thi TUYET DOI khong duoc suy ra "10 slot trong" tu
        // viec khong thay cartridge — robot se dat vao khoang trong khong co khay.
        // Tam class-0 phai nam trong outer ROI; model thinh thoang nhan nham
        // ong/tui toi mau o mep phai thanh tray voi confidence > 0.60.
        // outer ROI TINH, khong phai ban da bu — xem chu thich cung cho o
        // camera1Callback: gate tray-present khong duoc phu thuoc transform mà
        // chinh no sinh ra.
        Box tray_rect{};
        const bool raw_tray_present = findTrayRect(
            *msg, OUTPUT_TRAY_SCORE_THRESH, output_tray_outer_roi_, tray_rect);

        // Can 2 frame de xac nhan luc dat khay vao; mat detection thi khoa ngay
        // (fail-safe). Khi khoa, coi moi slot la occupied de khong consumer nao
        // dien giai slot_status=0 thanh vi tri co the dat.
        if (raw_tray_present) {
            output_tray_present_streak_++;
            if (output_tray_present_streak_ >= SLOT_CONFIRM_FRAMES)
                output_tray_present_.store(true);
        } else {
            output_tray_present_streak_ = 0;
            output_tray_present_.store(false);
            output_anchor_.reset();
        }

        const RoiTransform tf = output_anchor_enabled_
            ? output_anchor_.update(raw_tray_present ? &tray_rect : nullptr)
            : RoiTransform{};
        logAnchor("cam1", output_anchor_enabled_, tf, raw_tray_present, output_anchor_);

        auto present_msg = std_msgs::msg::Bool();
        present_msg.data = output_tray_present_.load();
        pub_output_present_->publish(present_msg);

        if (!output_tray_present_.load()) {
            std::lock_guard<std::mutex> lock(slot_detection_mutex_);
            selected_output_slot_ = -1;
            slot_stable_state_.fill(SlotStableState::OCC_OK);
            slot_empty_streak_.fill(0);
            slot_occ_streak_.fill(0);
            slot_mis_streak_.fill(0);
            output_full_streak_ = 0;
            tray_output_change_sent_ = false;

            auto slot_msg = std_msgs::msg::Int32();
            slot_msg.data = -1;
            pub_selected_slot_->publish(slot_msg);
            pub_ai_slot_->publish(slot_msg);

            auto status_msg = std_msgs::msg::Int32MultiArray();
            status_msg.data.assign(N_OUTPUT_SLOTS, 1);
            pub_slot_status_->publish(status_msg);
            return;
        }

        // Layer 2: Slot bi chiem khi BAT KY detection nao nam trong zone: tam bbox trong
        // quad, hoac bbox de len slot voi IoU >= 0.1. Khong ghep 1-1 nhu
        // Nova5 cu — mot cartridgefall nam vat ngang 2 slot phai chan CA HAI,
        // greedy 1-1 chi chan mot va de robot dat de len phan con lai.
        struct DetRec { Box box; float cx, cy; };
        std::vector<DetRec> dets;
        for (const auto& det : msg->detections) {
            if (det.results.empty()) continue;
            int cid = -1;
            try { cid = std::stoi(det.results[0].hypothesis.class_id); } catch (...) { continue; }
            // Model 3 class: 0=tray (khong tinh), 1=cartridge, 2=cartridgefall.
            if (cid != 1 && cid != 2) continue;
            if (det.results[0].hypothesis.score < DETECTION_SCORE_THRESH) continue;
            double cx = det.bbox.center.position.x;
            double cy = det.bbox.center.position.y;
            double hw = det.bbox.size_x / 2.0, hh = det.bbox.size_y / 2.0;
            dets.push_back({{cx - hw, cy - hh, cx + hw, cy + hh},
                            (float)cx, (float)cy});
        }

        std::array<SlotStableState, N_OUTPUT_SLOTS> instant_state;
        instant_state.fill(SlotStableState::EMPTY);
        for (size_t s = 0; s < N_OUTPUT_SLOTS; ++s) {
            const ROIQuad roi = transformQuad(output_tray_rois_[s], tf);
            Box slot_box{(double)roi.min_x, (double)roi.min_y,
                         (double)roi.max_x, (double)roi.max_y};
            for (const auto& d : dets) {
                if (roi.contains(d.cx, d.cy) || IoU(slot_box, d.box) >= 0.1) {
                    instant_state[s] = SlotStableState::OCC_OK;
                    break;
                }
            }
        }
        
        // Step 2: Update debouncing with mutex
        int local_selected_slot = -1;
        
        {
            std::lock_guard<std::mutex> lock(slot_detection_mutex_);
            
            for (size_t s = 0; s < N_OUTPUT_SLOTS; ++s) {
                if (instant_state[s] == SlotStableState::EMPTY) {
                    slot_empty_streak_[s]++;
                    slot_occ_streak_[s] = 0;
                    slot_mis_streak_[s] = 0;
                    if (slot_empty_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::EMPTY;
                } else if (instant_state[s] == SlotStableState::OCC_OK) {
                    slot_occ_streak_[s]++;
                    slot_empty_streak_[s] = 0;
                    slot_mis_streak_[s] = 0;
                    if (slot_occ_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::OCC_OK;
                } else {
                    slot_mis_streak_[s]++;
                    slot_empty_streak_[s] = 0;
                    slot_occ_streak_[s] = 0;
                    if (slot_mis_streak_[s] >= SLOT_CONFIRM_FRAMES)
                        slot_stable_state_[s] = SlotStableState::MIS;
                }
            }

            // Select first EMPTY slot (contiguous fill) — Nova5-style
            std::vector<int> empty_slots;
            for (size_t i = 0; i < N_OUTPUT_SLOTS; ++i)
                if (slot_stable_state_[i] == SlotStableState::EMPTY)
                    empty_slots.push_back(static_cast<int>(i + 1));
            local_selected_slot = select_contiguous_empty(
                empty_slots, static_cast<int>(N_OUTPUT_SLOTS));
        }
        
        selected_output_slot_ = local_selected_slot;
        
        // Publish results
        auto slot_msg = std_msgs::msg::Int32();
        slot_msg.data = local_selected_slot;
        pub_selected_slot_->publish(slot_msg);
        pub_ai_slot_->publish(slot_msg);
        
        auto status_msg = std_msgs::msg::Int32MultiArray();
        status_msg.data.resize(N_OUTPUT_SLOTS);
        for (size_t i = 0; i < N_OUTPUT_SLOTS; ++i) {
            status_msg.data[i] = (slot_stable_state_[i] == SlotStableState::EMPTY) ? 0 : 1;
        }
        pub_slot_status_->publish(status_msg);
        
        // Output tray full detection with debounce
        bool output_full = (local_selected_slot == -1);  // No empty slot found
        if (output_full) {
            output_full_streak_++;
            if (output_full_streak_ >= TRAY_CHANGE_CONFIRM_FRAMES && !tray_output_change_sent_) {
                auto change_msg = std_msgs::msg::Bool();
                change_msg.data = true;
                // Gửi trực tiếp tới cartridge system
                pub_change_tray_output_->publish(change_msg);
                // Thông báo robot logic để set waiting_for_new_output_
                pub_output_tray_full_->publish(change_msg);
                tray_output_change_sent_ = true;
                output_scan_allowed_ = false;
                RCLCPP_WARN(get_logger(), "[VISION] 📦 OUTPUT TRAY FULL - Change tray signal sent to cartridge + robot");
            }
        } else {
            output_full_streak_ = 0;
            tray_output_change_sent_ = false;  // Reset when new tray placed
        }
    }
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
