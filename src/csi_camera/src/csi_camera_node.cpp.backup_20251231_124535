#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <atomic>

/**
 * ============================================================================
 * DUAL CSI CAMERA NODE - Raspberry Pi 5 + 2x GMSL2 Boards
 * ============================================================================
 *
 * Uses rpicam-still for continuous capture (more reliable than rpicam-vid)
 * Each camera runs independently - no I2C switching needed
 *
 * Published Topics:
 *   - /cam0HP/image_raw (sensor_msgs/Image) - Camera 0 frames
 *   - /cam1HP/image_raw (sensor_msgs/Image) - Camera 1 frames
 *   - /camera/status    (std_msgs/String)   - System status
 *
 * ============================================================================
 */

class DualCSICameraNode : public rclcpp::Node
{
public:
    DualCSICameraNode() : Node("dual_csi_camera_node"), running_(false)
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "🎥 Dual CSI Camera Node - rpicam-still");
        RCLCPP_INFO(this->get_logger(), "========================================");

        // Declare parameters
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);
        this->declare_parameter("fps", 15);

        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();

        // Calculate frame interval
        frame_interval_ms_ = 1000 / fps_;

        RCLCPP_INFO(this->get_logger(), "Config: %dx%d @ %d fps (interval: %dms)", 
                    width_, height_, fps_, frame_interval_ms_);

        // Create publishers
        pub_cam0_ = this->create_publisher<sensor_msgs::msg::Image>("/cam0HP/image_raw", 10);
        pub_cam1_ = this->create_publisher<sensor_msgs::msg::Image>("/cam1HP/image_raw", 10);
        pub_status_ = this->create_publisher<std_msgs::msg::String>("/camera/status", 10);

        // Cleanup any existing camera processes
        cleanup_processes();

        // Start capture threads
        running_ = true;
        cam0_thread_ = std::thread(&DualCSICameraNode::camera_capture_loop, this, 0);
        cam1_thread_ = std::thread(&DualCSICameraNode::camera_capture_loop, this, 1);

        // Status publishing timer
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&DualCSICameraNode::publish_status, this));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "✅ Both camera threads started!");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~DualCSICameraNode()
    {
        RCLCPP_INFO(this->get_logger(), "🛑 Shutting down...");
        running_ = false;

        // Wait for threads to finish
        if (cam0_thread_.joinable()) cam0_thread_.join();
        if (cam1_thread_.joinable()) cam1_thread_.join();

        cleanup_processes();
        RCLCPP_INFO(this->get_logger(), "✅ Shutdown complete");
    }

private:
    void cleanup_processes()
    {
        RCLCPP_INFO(this->get_logger(), "  🧹 Cleaning up processes...");
        std::system("pkill -9 rpicam-still 2>/dev/null");
        std::system("pkill -9 rpicam-vid 2>/dev/null");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "  ✓ Cleanup done");
    }

    void camera_capture_loop(int cam_id)
    {
        RCLCPP_INFO(this->get_logger(), "  🎬 Camera %d thread started", cam_id);

        // Temp file for this camera
        std::string temp_file = "/tmp/cam" + std::to_string(cam_id) + "_frame.jpg";
        
        // Build rpicam-still command
        // -t 0 means capture immediately, --immediate skips preview delay
        std::string cmd = "rpicam-still --camera " + std::to_string(cam_id) + 
                          " -t 1 --immediate --nopreview" +
                          " --width " + std::to_string(width_) +
                          " --height " + std::to_string(height_) +
                          " -o " + temp_file + " 2>/dev/null";

        int frame_count = 0;
        int error_count = 0;
        const int max_errors = 10;

        while (running_ && rclcpp::ok()) {
            auto start_time = std::chrono::steady_clock::now();

            // Capture frame using rpicam-still
            int ret = std::system(cmd.c_str());
            
            if (ret != 0) {
                error_count++;
                if (error_count <= 3 || error_count % 10 == 0) {
                    RCLCPP_WARN(this->get_logger(), "Camera %d capture failed (error #%d)", 
                                cam_id, error_count);
                }
                if (error_count >= max_errors) {
                    RCLCPP_ERROR(this->get_logger(), "Camera %d: too many errors, stopping", cam_id);
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // Reset error count on success
            error_count = 0;

            // Read the captured image
            cv::Mat frame = cv::imread(temp_file, cv::IMREAD_COLOR);
            
            if (frame.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Camera %d: empty frame", cam_id);
                continue;
            }

            // Create ROS message
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = (cam_id == 0) ? "camera_input_tray" : "camera_output_tray";

            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

            // Publish to respective topic
            if (cam_id == 0 && pub_cam0_) {
                pub_cam0_->publish(*msg);
            } else if (cam_id == 1 && pub_cam1_) {
                pub_cam1_->publish(*msg);
            }

            frame_count++;
            if (frame_count % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Camera %d: published %d frames", 
                            cam_id, frame_count);
            }

            // Calculate sleep time to maintain target FPS
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
            int sleep_ms = frame_interval_ms_ - elapsed_ms;
            
            if (sleep_ms > 0) {
                rclcpp::sleep_for(std::chrono::milliseconds(sleep_ms));
            }
        }

        RCLCPP_INFO(this->get_logger(), "  🏁 Camera %d thread finished (published %d frames)", 
                    cam_id, frame_count);
    }

    void publish_status()
    {
        std_msgs::msg::String msg;
        msg.data = running_ ? "Dual cameras operational" : "Cameras stopped";
        pub_status_->publish(msg);
    }

    // Members
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam0_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::thread cam0_thread_;
    std::thread cam1_thread_;
    std::atomic<bool> running_;

    int width_;
    int height_;
    int fps_;
    int frame_interval_ms_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualCSICameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
