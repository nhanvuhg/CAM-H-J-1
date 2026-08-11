#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>


#include "yolo_tensorrt_ros2/yolo_trt_engine.hpp"

using yolo_trt::CudaRuntimeError;
using yolo_trt::Detection;
using yolo_trt::YoloTrtEngine;


class YoloTensorRtNode final : public rclcpp::Node
{
public:
  YoloTensorRtNode()
  : Node("yolo_tensorrt_node")
  {
    engine_path_ = declare_parameter<std::string>("engine_path", "");
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
    detections_topic_ =
      declare_parameter<std::string>("detections_topic", "/yolo/bounding_boxes");
    health_topic_ =
      declare_parameter<std::string>("health_topic", "~/health");
    class_names_ =
      declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{});
    confidence_threshold_ =
      static_cast<float>(declare_parameter<double>("confidence_threshold", 0.30));
    nms_threshold_ =
      static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
    max_detections_ = declare_parameter<int>("max_detections", 300);
    device_id_ = declare_parameter<int>("device_id", 0);
    max_inference_fps_ = declare_parameter<double>("max_inference_fps", 0.0);

    if (engine_path_.empty()) {
      throw std::runtime_error("Parameter engine_path must not be empty");
    }
    if (!(confidence_threshold_ > 0.0F && confidence_threshold_ <= 1.0F)) {
      throw std::runtime_error("confidence_threshold must be in (0, 1]");
    }
    if (!(nms_threshold_ > 0.0F && nms_threshold_ <= 1.0F)) {
      throw std::runtime_error("nms_threshold must be in (0, 1]");
    }
    max_detections_ = std::max(1, max_detections_);
    if (max_inference_fps_ < 0.0 || max_inference_fps_ > 120.0) {
      throw std::runtime_error("max_inference_fps must be in range 0..120");
    }
    if (max_inference_fps_ > 0.0) {
      minimum_inference_interval_ = std::chrono::nanoseconds(
        static_cast<std::int64_t>(1000000000.0 / max_inference_fps_));
    }

    engine_ = std::make_unique<YoloTrtEngine>(
      YoloTrtEngine::Options{
        engine_path_, device_id_, confidence_threshold_, nms_threshold_,
        max_detections_, class_names_});

    detections_publisher_ =
      create_publisher<vision_msgs::msg::Detection2DArray>(
      detections_topic_, rclcpp::SensorDataQoS());
    health_publisher_ =
      create_publisher<std_msgs::msg::String>(
      health_topic_, rclcpp::QoS(1).reliable().transient_local());

    image_subscription_ =
      create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&YoloTensorRtNode::image_callback, this, std::placeholders::_1));

    health_timer_ = create_wall_timer(
      std::chrono::seconds(2), std::bind(&YoloTensorRtNode::publish_health, this));

    RCLCPP_INFO(
      get_logger(),
      "READY engine=%s input=%dx%d classes=%d "
      "image=%s detections=%s max_fps=%.1f",
      engine_path_.c_str(), engine_->input_width(), engine_->input_height(),
      engine_->class_count(), image_topic_.c_str(),
      detections_topic_.c_str(), max_inference_fps_);
  }


private:

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    const auto started = std::chrono::steady_clock::now();
    received_frames_.fetch_add(1);
    if (minimum_inference_interval_.count() > 0) {
      if (next_inference_due_.time_since_epoch().count() == 0) {
        next_inference_due_ = started + minimum_inference_interval_;
      } else if (started < next_inference_due_) {
        rate_limited_frames_.fetch_add(1);
        return;
      } else if (started - next_inference_due_ >= minimum_inference_interval_) {
        // Rebase after a long input gap so recovery does not cause a burst.
        next_inference_due_ = started + minimum_inference_interval_;
      } else {
        // Preserve the fractional cadence against a 30 FPS camera clock. This
        // alternates accepted input gaps instead of collapsing 20 FPS to 15.
        next_inference_due_ += minimum_inference_interval_;
      }
    }

    try {
      const cv_bridge::CvImageConstPtr image =
        cv_bridge::toCvShare(message, "bgr8");
      if (!image || image->image.empty()) {
        throw std::runtime_error("Received an empty image");
      }

      const std::vector<Detection> detections = engine_->infer(image->image);
      vision_msgs::msg::Detection2DArray output;
      output.header = message->header;
      output.detections.reserve(detections.size());
      for (const auto & detection : detections) {
        vision_msgs::msg::Detection2D item;
        item.bbox.center.position.x =
          detection.box.x + detection.box.width * 0.5F;
        item.bbox.center.position.y =
          detection.box.y + detection.box.height * 0.5F;
        item.bbox.size_x = detection.box.width;
        item.bbox.size_y = detection.box.height;
        item.results.resize(1);
        item.results[0].hypothesis.class_id =
          std::to_string(detection.class_id);
        item.results[0].hypothesis.score = detection.score;
        output.detections.push_back(std::move(item));
      }
      detections_publisher_->publish(output);

      const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      last_inference_ms_.store(elapsed);
      last_detection_count_.store(detections.size());
      processed_frames_.fetch_add(1);

      const auto frame_count = processed_frames_.load();
      if (frame_count % 100 == 0) {
        RCLCPP_INFO(
          get_logger(), "AI running: frame=%lu inference=%.2fms detections=%zu",
          static_cast<unsigned long>(frame_count), elapsed, detections.size());
      }
    } catch (const CudaRuntimeError & error) {
      inference_errors_.fetch_add(1);
      fatal_backend_error_.store(true);
      RCLCPP_FATAL(
        get_logger(),
        "Fatal CUDA/TensorRT context error: %s; exiting for ROS launch respawn",
        error.what());
      // A launch timeout poisons this process' CUDA context. Retrying the
      // callback can never recover it and wastes CPU/GPU resources needed by
      // the Qt scene graph. End this process; both production YOLO actions are
      // configured with respawn=True.
      rclcpp::shutdown();
    } catch (const std::exception & error) {
      inference_errors_.fetch_add(1);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Inference failed: %s", error.what());
    }
  }

  void publish_health()
  {
    std_msgs::msg::String message;
    std::ostringstream status;
    status << (fatal_backend_error_.load() ? "FATAL" : "READY")
           << " backend=TensorRT engine=" << engine_path_
           << " max_fps=" << max_inference_fps_
           << " received=" << received_frames_.load()
           << " frames=" << processed_frames_.load()
           << " rate_limited=" << rate_limited_frames_.load()
           << " errors=" << inference_errors_.load()
           << " inference_ms=" << last_inference_ms_.load()
           << " detections=" << last_detection_count_.load();
    message.data = status.str();
    health_publisher_->publish(message);
  }

  std::unique_ptr<YoloTrtEngine> engine_;

  std::string engine_path_;
  std::string image_topic_;
  std::string detections_topic_;
  std::string health_topic_;
  std::vector<std::string> class_names_;
  float confidence_threshold_{0.30F};
  float nms_threshold_{0.45F};
  int max_detections_{300};
  int device_id_{0};
  double max_inference_fps_{0.0};
  std::chrono::nanoseconds minimum_inference_interval_{0};
  std::chrono::steady_clock::time_point next_inference_due_{};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr
    detections_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_publisher_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  std::atomic<std::uint64_t> processed_frames_{0};
  std::atomic<std::uint64_t> received_frames_{0};
  std::atomic<std::uint64_t> rate_limited_frames_{0};
  std::atomic<std::uint64_t> inference_errors_{0};
  std::atomic<bool> fatal_backend_error_{false};
  std::atomic<std::size_t> last_detection_count_{0};
  std::atomic<double> last_inference_ms_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<YoloTensorRtNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "[yolo_tensorrt_node] FATAL: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
