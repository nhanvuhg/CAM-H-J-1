// ROS2 publisher backed by the validated V3Link CUDA debayer/tone pipeline.
// The implementation is compiled at the ROS production output size. The
// standalone Desktop viewer remains 800x450 and uses the same source pipeline.
#define V3LINK_CUDA_NO_MAIN
#define V3LINK_OUTPUT_WIDTH 640
#define V3LINK_OUTPUT_HEIGHT 360
#include "/home/nhan/v3link_cuda_view.cu"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

class V4L2DualCameraCudaNode : public rclcpp::Node {
public:
    V4L2DualCameraCudaNode()
        : Node("v4l2_dual_camera"),
          cam0_topic_(declare_parameter<std::string>(
              "cam0_topic", "/cam0HP/image_raw")),
          cam1_topic_(declare_parameter<std::string>(
              "cam1_topic", "/cam1HP/image_raw")),
          cam0_health_topic_(declare_parameter<std::string>(
              "cam0_health_topic", "/camera/cam0/health")),
          cam1_health_topic_(declare_parameter<std::string>(
              "cam1_health_topic", "/camera/cam1/health")),
          cam0_clarity_(declare_parameter<double>("cam0_clarity", 0.45)),
          cam1_clarity_(declare_parameter<double>("cam1_clarity", 0.60)),
          capture_fps_(declare_parameter<int>("capture_fps", 30)),
          exposure_(declare_parameter<int>("exposure", 32000)) {
        auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
        image_publishers_[0] =
            create_publisher<sensor_msgs::msg::Image>(cam0_topic_, image_qos);
        image_publishers_[1] =
            create_publisher<sensor_msgs::msg::Image>(cam1_topic_, image_qos);
        health_publishers_[0] = create_publisher<std_msgs::msg::String>(
            cam0_health_topic_, rclcpp::QoS(10).reliable());
        health_publishers_[1] = create_publisher<std_msgs::msg::String>(
            cam1_health_topic_, rclcpp::QoS(10).reliable());

        if (capture_fps_ < 1 || capture_fps_ > 60) {
            throw std::runtime_error("capture_fps must be in range 1..60");
        }
        if (exposure_ < 13 || exposure_ > 683710) {
            throw std::runtime_error("exposure is outside IMX477 range");
        }

        health_timer_ = create_wall_timer(
            1s, std::bind(&V4L2DualCameraCudaNode::publishHealth, this));
        capture_thread_ =
            std::thread(&V4L2DualCameraCudaNode::captureLoop, this);

        RCLCPP_INFO(
            get_logger(),
            "CUDA V4L2 starting: CAM0=/dev/video1 CAM1=/dev/video0, "
            "raw=1920x1080 RG10, publish=640x360 BGR8, request=%d fps, "
            "exposure=%d gain=66, clarity={%.2f,%.2f}",
            capture_fps_, exposure_, cam0_clarity_, cam1_clarity_);
    }

    ~V4L2DualCameraCudaNode() override {
        stopping_.store(true);
        g_stop.store(true);
        if (capture_thread_.joinable()) capture_thread_.join();
    }

private:
    void captureLoop() {
        try {
            CudaProcessor processor;
            const cv::Mat chroma_lut = makeChromaLut();
            RCLCPP_INFO(get_logger(), "CUDA device: %s SM %d.%d",
                        processor.gpuName().c_str(), processor.computeMajor(),
                        processor.computeMinor());

            while (rclcpp::ok() && !stopping_.load()) {
                try {
                    // Opening is intentionally serialized: both V3Link
                    // receivers share the camera control path.
                    V4L2Camera cam0(0, 1, exposure_, capture_fps_);
                    std::this_thread::sleep_for(1500ms);
                    V4L2Camera cam1(1, 0, exposure_, capture_fps_);

                    {
                        std::lock_guard<std::mutex> lock(status_mutex_);
                        statuses_[0] = "STREAMING";
                        statuses_[1] = "STREAMING";
                    }
                    RCLCPP_INFO(get_logger(),
                                "Both CUDA camera streams are active");

                    std::array<uint64_t, 2> window_frames{0, 0};
                    auto window_start = std::chrono::steady_clock::now();
                    while (rclcpp::ok() && !stopping_.load()) {
                        V4L2Camera* cameras[2] = {&cam0, &cam1};
                        const float clarity[2] = {
                            static_cast<float>(cam0_clarity_),
                            static_cast<float>(cam1_clarity_)};

                        for (int camera_id = 0;
                             camera_id < 2 && rclcpp::ok() &&
                             !stopping_.load();
                             ++camera_id) {
                            auto frame = cameras[camera_id]->readLatest();
                            if (!frame.data) break;
                            cv::Mat image;
                            try {
                                image = processor.process(
                                    frame.data, frame.stride_words,
                                    clarity[camera_id], chroma_lut);
                            } catch (...) {
                                cameras[camera_id]->release();
                                throw;
                            }
                            cameras[camera_id]->release();
                            publishImage(camera_id, image);
                            ++window_frames[camera_id];
                            published_[camera_id].fetch_add(1);
                            last_frame_ns_[camera_id].store(
                                std::chrono::duration_cast<
                                    std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now()
                                        .time_since_epoch())
                                    .count());
                        }

                        const auto now = std::chrono::steady_clock::now();
                        const double elapsed =
                            std::chrono::duration<double>(now - window_start)
                                .count();
                        if (elapsed >= 2.0) {
                            std::lock_guard<std::mutex> lock(status_mutex_);
                            for (int id = 0; id < 2; ++id) {
                                measured_fps_[id] =
                                    window_frames[id] / elapsed;
                                window_frames[id] = 0;
                            }
                            average_gpu_ms_ = processor.averageGpuMs();
                            window_start = now;
                        }
                    }
                } catch (const std::exception& error) {
                    if (stopping_.load() || !rclcpp::ok()) break;
                    const int reconnect = ++reconnects_;
                    {
                        std::lock_guard<std::mutex> lock(status_mutex_);
                        statuses_[0] = "RECONNECTING";
                        statuses_[1] = "RECONNECTING";
                    }
                    RCLCPP_ERROR(get_logger(),
                                 "CUDA camera error: %s; paired reconnect #%d "
                                 "in 5 seconds",
                                 error.what(), reconnect);
                    for (int i = 0; i < 50 && !stopping_.load() &&
                                        rclcpp::ok();
                         ++i) {
                        std::this_thread::sleep_for(100ms);
                    }
                }
            }
        } catch (const std::exception& error) {
            RCLCPP_FATAL(get_logger(), "CUDA pipeline initialization failed: %s",
                         error.what());
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                statuses_[0] = "FATAL";
                statuses_[1] = "FATAL";
            }
            rclcpp::shutdown();
        }
    }

    void publishImage(int camera_id, const cv::Mat& image) {
        auto message = std::make_unique<sensor_msgs::msg::Image>();
        message->header.stamp = now();
        message->header.frame_id =
            "cam" + std::to_string(camera_id) + "_optical_frame";
        message->height = kOutputHeight;
        message->width = kOutputWidth;
        message->encoding = "bgr8";
        message->is_bigendian = false;
        message->step = kOutputWidth * 3;
        message->data.assign(image.data,
                             image.data + kOutputWidth * kOutputHeight * 3);
        image_publishers_[camera_id]->publish(std::move(message));
    }

    void publishHealth() {
        std::array<double, 2> fps{};
        std::array<std::string, 2> status{};
        double gpu_ms = 0.0;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            fps = measured_fps_;
            status = statuses_;
            gpu_ms = average_gpu_ms_;
        }
        const int64_t now_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        for (int id = 0; id < 2; ++id) {
            const int64_t last_ns = last_frame_ns_[id].load();
            const double age = last_ns > 0 ? (now_ns - last_ns) / 1e9 : 999.0;
            std_msgs::msg::String message;
            std::ostringstream text;
            text << status[id] << " device=/dev/video" << (id == 0 ? 1 : 0)
                 << " backend=CUDA size=" << kOutputWidth << "x"
                 << kOutputHeight << " fps=" << std::fixed
                 << std::setprecision(1) << fps[id] << " age="
                 << std::setprecision(2) << age << "s gpu_ms=" << gpu_ms
                 << " reconnects=" << reconnects_.load()
                 << " published=" << published_[id].load();
            message.data = text.str();
            health_publishers_[id]->publish(message);
        }
    }

    std::string cam0_topic_;
    std::string cam1_topic_;
    std::string cam0_health_topic_;
    std::string cam1_health_topic_;
    double cam0_clarity_;
    double cam1_clarity_;
    int capture_fps_;
    int exposure_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 2>
        image_publishers_;
    std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr, 2>
        health_publishers_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    std::thread capture_thread_;
    std::atomic<bool> stopping_{false};
    std::atomic<int> reconnects_{0};
    std::array<std::atomic<uint64_t>, 2> published_{{0, 0}};
    std::array<std::atomic<int64_t>, 2> last_frame_ns_{{0, 0}};
    std::mutex status_mutex_;
    std::array<double, 2> measured_fps_{{0.0, 0.0}};
    std::array<std::string, 2> statuses_{{"OPENING", "OPENING"}};
    double average_gpu_ms_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<V4L2DualCameraCudaNode>();
        rclcpp::spin(node);
        node.reset();
    } catch (const std::exception& error) {
        std::cerr << "V4L2 CUDA ROS node failed: " << error.what() << "\n";
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
