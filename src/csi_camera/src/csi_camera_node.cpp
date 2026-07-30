// One IMX477 CSI camera per process for NVIDIA Jetson Orin Nano.
//
// The V3Link setup on this machine requires each nvarguscamerasrc pipeline to
// live in a separate process. The launch file therefore starts two instances
// of this executable, one for each sensor.

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class JetsonCsiCameraNode : public rclcpp::Node
{
public:
  JetsonCsiCameraNode()
  : Node("csi_camera_node")
  {
    sensor_id_ = declare_parameter<int>("sensor_id", 0);
    sensor_mode_ = declare_parameter<int>("sensor_mode", 2);
    capture_width_ = declare_parameter<int>("capture_width", 1920);
    capture_height_ = declare_parameter<int>("capture_height", 1080);
    capture_fps_ = declare_parameter<int>("capture_fps", 30);
    publish_fps_ = declare_parameter<int>("publish_fps", 10);
    output_width_ = declare_parameter<int>("output_width", 640);
    output_height_ = declare_parameter<int>("output_height", 480);
    flip_method_ = declare_parameter<int>("flip_method", 0);
    ee_mode_ = declare_parameter<int>("ee_mode", 0);
    isp_digital_gain_min_ =
      declare_parameter<double>("isp_digital_gain_min", 1.0);
    isp_digital_gain_max_ =
      declare_parameter<double>("isp_digital_gain_max", 1.0);
    image_topic_ =
      declare_parameter<std::string>("image_topic", "/camera/image_raw");
    health_topic_ =
      declare_parameter<std::string>("health_topic", "/camera/health");
    frame_id_ =
      declare_parameter<std::string>("frame_id", "camera_optical_frame");
    reconnect_delay_ms_ = declare_parameter<int>("reconnect_delay_ms", 2000);

    validate_parameters();

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(image_topic_, image_qos);
    health_publisher_ =
      create_publisher<std_msgs::msg::String>(health_topic_, 10);
    health_timer_ = create_wall_timer(2s, [this]() {publish_health();});

    RCLCPP_INFO(
      get_logger(),
      "Jetson CSI sensor-id=%d mode=%d: %dx%d@%d -> %dx%d, publish <= %d Hz",
      sensor_id_, sensor_mode_, capture_width_, capture_height_, capture_fps_,
      output_width_, output_height_, publish_fps_);

    running_.store(true);
    capture_thread_ = std::thread(&JetsonCsiCameraNode::capture_loop, this);
  }

  ~JetsonCsiCameraNode() override
  {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

private:
  void validate_parameters()
  {
    sensor_id_ = std::max(0, sensor_id_);
    sensor_mode_ = std::max(0, sensor_mode_);
    capture_width_ = std::max(1, capture_width_);
    capture_height_ = std::max(1, capture_height_);
    capture_fps_ = std::max(1, capture_fps_);
    publish_fps_ = std::max(1, publish_fps_);
    output_width_ = std::max(1, output_width_);
    output_height_ = std::max(1, output_height_);
    flip_method_ = std::clamp(flip_method_, 0, 7);
    ee_mode_ = std::max(0, ee_mode_);
    isp_digital_gain_min_ = std::max(1.0, isp_digital_gain_min_);
    isp_digital_gain_max_ =
      std::max(isp_digital_gain_min_, isp_digital_gain_max_);
    reconnect_delay_ms_ = std::max(250, reconnect_delay_ms_);
  }

  std::string make_pipeline() const
  {
    // Upstream settings intentionally match camera-2x.desktop:
    // sensor-mode=2, 1080p30, ee-mode=0 and fixed ISP digital gain.
    std::ostringstream pipeline;
    pipeline
      << "nvarguscamerasrc sensor-id=" << sensor_id_
      << " sensor-mode=" << sensor_mode_
      << " ee-mode=" << ee_mode_
      << " ispdigitalgainrange=\""
      << isp_digital_gain_min_ << " " << isp_digital_gain_max_ << "\" ! "
      << "video/x-raw(memory:NVMM),"
      << "width=(int)" << capture_width_ << ","
      << "height=(int)" << capture_height_ << ","
      << "framerate=(fraction)" << capture_fps_ << "/1,"
      << "format=(string)NV12 ! "
      // Keep the two-stage nvvidconv/queue topology from camera-2x.desktop.
      // A leaky queue directly after nvarguscamerasrc can return an NVMM
      // surface while Argus still owns it, which led to
      // NvBufSurfaceFromFd/INVALID_SETTINGS failures with two cameras.
      << "queue ! "
      << "nvvidconv flip-method=" << flip_method_ << " ! "
      << "video/x-raw,"
      << "width=(int)" << output_width_ << ","
      << "height=(int)" << output_height_ << " ! "
      << "queue ! "
      << "nvvidconv ! "
      << "video/x-raw,"
      << "format=(string)BGRx ! "
      << "videoconvert ! video/x-raw,format=(string)BGR ! "
      << "appsink drop=true max-buffers=1 sync=false";
    return pipeline.str();
  }

  bool open_camera(cv::VideoCapture & capture)
  {
    const std::string pipeline = make_pipeline();
    set_status("OPENING");
    RCLCPP_INFO(
      get_logger(), "Opening Argus sensor-id=%d in this process", sensor_id_);

    if (!capture.open(pipeline, cv::CAP_GSTREAMER)) {
      set_status("OPEN_FAILED");
      RCLCPP_ERROR(
        get_logger(), "Could not open sensor-id=%d. Pipeline: %s",
        sensor_id_, pipeline.c_str());
      return false;
    }
    return true;
  }

  void capture_loop()
  {
    const auto min_publish_period =
      std::chrono::duration<double>(1.0 / static_cast<double>(publish_fps_));

    while (running_.load() && rclcpp::ok()) {
      cv::VideoCapture capture;
      if (!open_camera(capture)) {
        sleep_interruptible(reconnect_delay_ms_);
        continue;
      }

      set_status("WAITING_FOR_FRAME");
      int consecutive_read_failures = 0;
      auto fps_window_start = std::chrono::steady_clock::now();
      auto last_publish = fps_window_start - std::chrono::seconds(1);
      uint64_t fps_window_frames = 0;

      while (running_.load() && rclcpp::ok()) {
        cv::Mat frame;
        if (!capture.read(frame) || frame.empty()) {
          ++consecutive_read_failures;
          if (consecutive_read_failures >= 3) {
            RCLCPP_WARN(
              get_logger(), "Sensor-id=%d lost Argus stream; reconnecting",
              sensor_id_);
            set_status("STREAM_LOST");
            break;
          }
          continue;
        }

        consecutive_read_failures = 0;
        streaming_.store(true);
        captured_frames_.fetch_add(1);
        ++fps_window_frames;

        const auto steady_now = std::chrono::steady_clock::now();
        const auto fps_elapsed =
          std::chrono::duration<double>(steady_now - fps_window_start).count();
        if (fps_elapsed >= 2.0) {
          capture_fps_measured_.store(
            static_cast<double>(fps_window_frames) / fps_elapsed);
          fps_window_frames = 0;
          fps_window_start = steady_now;
        }

        if (steady_now - last_publish < min_publish_period) {
          continue;
        }
        last_publish = steady_now;

        sensor_msgs::msg::Image message;
        message.header.stamp = now();
        message.header.frame_id = frame_id_;
        message.height = static_cast<uint32_t>(frame.rows);
        message.width = static_cast<uint32_t>(frame.cols);
        message.encoding = "bgr8";
        message.is_bigendian = false;
        message.step = static_cast<sensor_msgs::msg::Image::_step_type>(
          frame.cols * frame.elemSize());
        message.data.resize(
          static_cast<size_t>(message.step) * static_cast<size_t>(message.height));
        for (int row = 0; row < frame.rows; ++row) {
          std::memcpy(
            message.data.data() + static_cast<size_t>(row) * message.step,
            frame.ptr(row), message.step);
        }
        image_publisher_->publish(message);
        published_frames_.fetch_add(1);
        set_status("STREAMING");
      }

      streaming_.store(false);
      capture_fps_measured_.store(0.0);
      capture.release();
      if (running_.load() && rclcpp::ok()) {
        set_status("RECONNECTING");
        sleep_interruptible(reconnect_delay_ms_);
      }
    }

    streaming_.store(false);
    set_status("STOPPED");
  }

  void sleep_interruptible(int milliseconds)
  {
    int remaining = milliseconds;
    while (running_.load() && rclcpp::ok() && remaining > 0) {
      const int step = std::min(remaining, 100);
      std::this_thread::sleep_for(std::chrono::milliseconds(step));
      remaining -= step;
    }
  }

  void publish_health()
  {
    std_msgs::msg::String message;
    std::ostringstream text;
    text << get_status()
         << " sensor_id=" << sensor_id_
         << " capture_fps=" << std::fixed << std::setprecision(1)
         << capture_fps_measured_.load()
         << " captured=" << captured_frames_.load()
         << " published=" << published_frames_.load();
    message.data = text.str();
    health_publisher_->publish(message);
  }

  void set_status(const std::string & status)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_ = status;
  }

  std::string get_status()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
  }

  int sensor_id_{0};
  int sensor_mode_{2};
  int capture_width_{1920};
  int capture_height_{1080};
  int capture_fps_{30};
  int publish_fps_{10};
  int output_width_{640};
  int output_height_{480};
  int flip_method_{0};
  int ee_mode_{0};
  double isp_digital_gain_min_{1.0};
  double isp_digital_gain_max_{1.0};
  int reconnect_delay_ms_{2000};
  std::string image_topic_;
  std::string health_topic_;
  std::string frame_id_;

  std::mutex status_mutex_;
  std::string status_{"STARTING"};
  std::atomic<bool> running_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<double> capture_fps_measured_{0.0};
  std::atomic<uint64_t> captured_frames_{0};
  std::atomic<uint64_t> published_frames_{0};
  std::thread capture_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_publisher_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetsonCsiCameraNode>());
  rclcpp::shutdown();
  return 0;
}
