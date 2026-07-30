#include "yolo_ros_hailort_cpp/yolo_ros_hailort_cpp.hpp"
#include <algorithm>  // [CONF-GUARD] for std::remove_if
#include <cmath>      // std::round for letterbox sizing

namespace yolo_ros_hailort_cpp
{
    YoloNode::YoloNode(const rclcpp::NodeOptions &options)
        : Node("yolo_ros_hailort_cpp", options)
    {
        using namespace std::chrono_literals; // NOLINT
        this->init_timer_ = this->create_wall_timer(
            0s, std::bind(&YoloNode::onInit, this));
    }

    YoloNode::~YoloNode()
    {
        // Safely join the inference thread before destruction
        if (inference_thread_.joinable()) {
            inference_thread_.join();
        }
    }

    void YoloNode::onInit()
    {
        this->init_timer_->cancel();
        this->param_listener_ = std::make_shared<yolo_parameters::ParamListener>(
            this->get_node_parameters_interface());

        this->params_ = this->param_listener_->get_params();

        if (this->params_.imshow_isshow)
        {
            cv::namedWindow("yolo", cv::WINDOW_AUTOSIZE);
        }

        if (this->params_.class_labels_path != "")
        {
            RCLCPP_INFO(this->get_logger(), "read class labels from '%s'", this->params_.class_labels_path.c_str());
            this->class_names_ = yolo_cpp::utils::read_class_labels_file(this->params_.class_labels_path);
        }
        else
        {
            this->class_names_ = yolo_cpp::COCO_CLASSES;
        }

        // Retry init mỗi 3s thay vì throw chết node.
        // Lý do: Hailo PCIe có thể chưa sẵn sàng lúc boot, hoặc HEF lock bị process khác giữ.
        // Tự retry hồi phục được không cần systemd kick lại.
        for (int attempt = 1; rclcpp::ok(); ++attempt) {
            try {
                this->yolo_ = std::make_unique<yolo_cpp::YoloHailoRT>(
                    this->params_.model_path,
                    this->params_.conf,
                    this->params_.nms);
                break;
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                    "[INIT] YoloHailoRT failed (attempt %d): %s — retry in 3s",
                    attempt, e.what());
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
        }
        if (!this->yolo_) return;  // shutdown trong lúc retry

        RCLCPP_INFO(this->get_logger(), "model loaded");

        // [FIX-2] Use best_effort QoS with depth=1 to drop frames instead of queuing
        // When YOLO inference stalls (detecting objects), new frames are dropped instead
        // of accumulating in buffer → prevents executor starvation and pipe backpressure
        auto qos = rclcpp::QoS(1).best_effort();
        this->sub_image_ = image_transport::create_subscription(
            this, this->params_.src_image_topic_name,
            std::bind(&YoloNode::colorImageCallback, this, std::placeholders::_1),
            "raw",
            qos.get_rmw_qos_profile());


        this->pub_detection2d_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            this->params_.publish_boundingbox_topic_name,
            10);

        if (this->params_.publish_resized_image) {
            this->pub_image_ = image_transport::create_publisher(this, this->params_.publish_image_topic_name);
        }
    }

    void YoloNode::colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &ptr)
    {
        // Guard: nếu init retry chưa xong (yolo_ = nullptr) → bỏ frame.
        if (!this->yolo_) return;

        // Drop frame if inference thread is busy — prevents latency accumulation.
        if (inference_busy_.exchange(true)) {
            return;
        }

        // Guard cv_bridge: toCvCopy có thể throw nếu encoding mismatch.
        cv_bridge::CvImagePtr img;
        try {
            img = cv_bridge::toCvCopy(ptr, "bgr8");
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "[CVB] toCvCopy failed: %s", e.what());
            inference_busy_.store(false);
            return;
        }
        if (!img || img->image.empty()) {
            inference_busy_.store(false);
            return;
        }

        if (inference_thread_.joinable()) {
            inference_thread_.detach();
        }

        // Run inference in managed thread. Bọc try/catch toàn bộ để exception KHÔNG
        // kill thread mà không reset inference_busy_ → kẹt true → mọi frame sau bị drop.
        inference_thread_ = std::thread([this, img]() {
            try {
                cv::Mat frame = img->image.clone();

                const int model_w = static_cast<int>(this->yolo_->get_input_width());
                const int model_h = static_cast<int>(this->yolo_->get_input_height());

                // Mapping from model space back to source pixels. Stretch keeps the
                // per-axis ratios; letterbox uses one ratio plus the padding offset.
                float scale_x, scale_y, pad_x = 0.0f, pad_y = 0.0f;
                cv::Mat resized_frame;

                if (this->params_.letterbox) {
                    // A plain resize distorts objects when the camera aspect ratio does not
                    // match the HEF input (640x480 into 640x640 stretches by 1.33 vertically),
                    // which costs far more confidence than it looks: measured 0.25 vs 0.82.
                    const float r = std::min(static_cast<float>(model_w) / frame.cols,
                                             static_cast<float>(model_h) / frame.rows);
                    const int new_w = static_cast<int>(std::round(frame.cols * r));
                    const int new_h = static_cast<int>(std::round(frame.rows * r));
                    pad_x = (model_w - new_w) / 2.0f;
                    pad_y = (model_h - new_h) / 2.0f;

                    cv::Mat scaled;
                    cv::resize(frame, scaled, cv::Size(new_w, new_h));
                    resized_frame = cv::Mat(model_h, model_w, frame.type(), cv::Scalar(114, 114, 114));
                    scaled.copyTo(resized_frame(cv::Rect(static_cast<int>(pad_x), static_cast<int>(pad_y), new_w, new_h)));

                    scale_x = scale_y = 1.0f / r;
                } else {
                    cv::resize(frame, resized_frame, cv::Size(model_w, model_h));
                    scale_x = static_cast<float>(frame.cols) / static_cast<float>(model_w);
                    scale_y = static_cast<float>(frame.rows) / static_cast<float>(model_h);
                }

                auto now = std::chrono::system_clock::now();
                auto objects = this->yolo_->inference(resized_frame);
                auto end = std::chrono::system_clock::now();

                const float conf_threshold = static_cast<float>(this->params_.conf);
                objects.erase(
                    std::remove_if(objects.begin(), objects.end(),
                        [conf_threshold](const yolo_cpp::Object &o) {
                            return o.prob < conf_threshold;
                        }),
                    objects.end());

                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
                RCLCPP_INFO(this->get_logger(), "Inference time: %5ld ms", elapsed.count());

                if (this->params_.imshow_isshow || this->params_.publish_resized_image)
                {
                    yolo_cpp::utils::draw_objects(frame, objects, this->class_names_);
                    if (this->params_.imshow_isshow)
                    {
                        cv::imshow("yolo", frame);
                        auto key = cv::waitKey(1);
                        if (key == 27) {
                            rclcpp::shutdown();
                        }
                    }
                    if (this->params_.publish_resized_image) {
                        sensor_msgs::msg::Image::SharedPtr pub_img =
                            cv_bridge::CvImage(img->header, "bgr8", frame).toImageMsg();
                        this->pub_image_.publish(pub_img);
                    }
                }

                vision_msgs::msg::Detection2DArray detections = objects_to_detection2d(objects, img->header, scale_x, scale_y, pad_x, pad_y);
                this->pub_detection2d_->publish(detections);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "[INFER] exception: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "[INFER] unknown exception");
            }
            inference_busy_.store(false);  // PHẢI luôn reset, kể cả khi exception.
        });
    }

    vision_msgs::msg::Detection2DArray YoloNode::objects_to_detection2d(
        const std::vector<yolo_cpp::Object> &objects,
        const std_msgs::msg::Header &header,
        float scale_x, float scale_y, float pad_x, float pad_y)
    {
        vision_msgs::msg::Detection2DArray detection2d;
        detection2d.header = header;
        for (const auto &obj : objects)
        {
            vision_msgs::msg::Detection2D det;
            det.bbox.center.position.x = (obj.rect.x + obj.rect.width / 2.0f - pad_x) * scale_x;
            det.bbox.center.position.y = (obj.rect.y + obj.rect.height / 2.0f - pad_y) * scale_y;
            det.bbox.size_x = obj.rect.width * scale_x;
            det.bbox.size_y = obj.rect.height * scale_y;

            det.results.resize(1);
            det.results[0].hypothesis.class_id = std::to_string(obj.label);
            det.results[0].hypothesis.score = obj.prob;
            detection2d.detections.emplace_back(det);
        }
        return detection2d;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(yolo_ros_hailort_cpp::YoloNode)