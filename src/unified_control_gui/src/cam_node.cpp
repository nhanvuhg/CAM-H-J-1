#include "unified_control_gui/cam_node.hpp"
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <sys/stat.h>

namespace
{
QString cameraDisplayName(const QString &topic, int fallbackIndex)
{
    // Camera roles are tied to their ROS namespaces, not to the order in which
    // topics happen to be discovered or selected in the settings page.
    if (topic.contains(QStringLiteral("/cam0HP/")))
        return QStringLiteral("CAM0 - INPUT");
    if (topic.contains(QStringLiteral("/cam1HP/")))
        return QStringLiteral("CAM1 - OUTPUT");

    return QString("Camera %1").arg(fallbackIndex + 1);
}
}  // namespace

CamNode::CamNode(QQmlApplicationEngine &engine)
    : QObject(), rclcpp::Node("qml_cam_node_hp"), engine_(&engine) {
    displayFps_ = declare_parameter<int>("display_fps", 10);
    if (displayFps_ < 1 || displayFps_ > 30) {
        throw std::invalid_argument("display_fps must be in range 1..30");
    }

    // Set config file path
    const char* home = std::getenv("HOME");
    if (home) {
        std::string config_dir = std::string(home) + "/.config/ros2_gui";
        // Create directory if it doesn't exist
        mkdir(config_dir.c_str(), 0755);
        configFilePath_ = config_dir + "/camera_topics.conf";
        RCLCPP_INFO(this->get_logger(), "Config file: %s", configFilePath_.c_str());
    }

    // Pre-allocate providers 1 lần. Engine owns chúng — không delete thủ công bao giờ.
    // Bug history: setup() cũ delete provider mỗi lần đổi topic → callback đang fire
    // vẫn giữ index cũ → use-after-free khi truy cập providers_[i].
    providers_.reserve(maxCameras_);
    for (int i = 0; i < maxCameras_; ++i) {
        auto *provider = new CamProvider();
        QString providerId = QString("cam_%1").arg(i);
        engine_->addImageProvider(providerId, provider);  // engine TAKES OWNERSHIP
        providers_.push_back(provider);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Camera display processing limited to %d FPS per feed",
        displayFps_);
}

CamNode::~CamNode()
{
    // Join discovery thread nếu đang chạy — KHÔNG detach để tránh thread sống quá engine.
    if (discoveryThread_.joinable()) {
        discoveryThread_.join();
    }
    // KHÔNG delete providers_ — QQmlEngine sở hữu, sẽ tự delete khi engine destruct.
}

void CamNode::setup(const std::vector<std::string> &topics)
{
    subs_.clear();

    std::vector<std::string> limitedTopics = topics;
    if (limitedTopics.size() > static_cast<size_t>(maxCameras_))
        limitedTopics.resize(maxCameras_);

    // KHÔNG còn dynamic alloc/delete providers — chúng pre-alloc ở constructor.

    cameraList_.clear();

    for (auto &timestamp : lastDisplayFrameNs_) {
        timestamp.store(0, std::memory_order_relaxed);
    }

    for (size_t i = 0; i < limitedTopics.size(); ++i)
    {
        QString providerId = QString("cam_%1").arg(i);
        CamProvider *provider = providers_[i];  // pointer ổn định, sống suốt đời CamNode

        // Capture provider POINTER trực tiếp (không capture i rồi tra vector) — tránh
        // race: nếu vector bị resize (về sau), index có thể trỏ sai. Pointer thì stable.
        auto sub = this->create_subscription<sensor_msgs::msg::Image>(
            limitedTopics[i], rclcpp::SensorDataQoS().keep_last(1),
            [this, provider, i](const std::shared_ptr<const sensor_msgs::msg::Image> &msg)
            {
                if (!shouldProcessFrame(i)) {
                    return;
                }
                try
                {
                    const auto cvimg = cv_bridge::toCvShare(msg, "bgr8")->image;
                    QImage qimg(cvimg.data, cvimg.cols, cvimg.rows, cvimg.step, QImage::Format_BGR888);
                    provider->setImage(qimg.copy());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Image error: %s", e.what());
                }
            });

        subs_.push_back(sub);

        const QString topic = QString::fromStdString(limitedTopics[i]);
        QVariantMap cam;
        cam["name"] = cameraDisplayName(topic, static_cast<int>(i));
        cam["topic"] = topic;
        cam["providerId"] = providerId;
        cameraList_.append(cam);
    }

    emit cameraListChanged();
}

bool CamNode::shouldProcessFrame(std::size_t cameraIndex)
{
    if (cameraIndex >= lastDisplayFrameNs_.size()) {
        return false;
    }

    const auto nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const std::int64_t minimumIntervalNs = 1000000000LL / displayFps_;
    auto previousNs = lastDisplayFrameNs_[cameraIndex].load(std::memory_order_relaxed);
    if (nowNs - previousNs < minimumIntervalNs) {
        return false;
    }

    lastDisplayFrameNs_[cameraIndex].store(nowNs, std::memory_order_relaxed);
    return true;
}

QStringList CamNode::getAvailableImageTopics()
{
    QStringList result;
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &pair : topics_and_types)
    {
        const auto &topic_name = pair.first;
        const auto &types = pair.second;
        for (const auto &type : types)
        {
            if (type == "sensor_msgs/msg/Image")
            {
                result << QString::fromStdString(topic_name);
            }
        }
    }
    return result;
}

void CamNode::fetchAvailableTopicsAsync()
{
    // Prevent concurrent discovery
    bool expected = false;
    if (!fetchingTopics_.compare_exchange_strong(expected, true))
        return;

    // Join thread cũ nếu có — đảm bảo chỉ 1 thread sống tại 1 thời điểm.
    if (discoveryThread_.joinable()) discoveryThread_.join();

    // Run discovery in background thread to avoid blocking UI.
    // Lưu handle vào member để dtor join được (KHÔNG detach).
    discoveryThread_ = std::thread([this]() {
        QStringList result;
        try {
            auto topics_and_types = this->get_topic_names_and_types();
            for (const auto &pair : topics_and_types) {
                for (const auto &type : pair.second) {
                    if (type == "sensor_msgs/msg/Image")
                        result << QString::fromStdString(pair.first);
                }
            }
        } catch (...) {}

        fetchingTopics_ = false;
        // Emit via queued connection — safe to cross thread boundary
        emit availableTopicsChanged(result);
    });
}


void CamNode::updateCameraTopic(int index, const QString &newTopic)
{
    if (index < 0 || index >= static_cast<int>(subs_.size()))
        return;

    // Guard: skip empty or invalid topic names
    if (newTopic.isEmpty()) {
        RCLCPP_WARN(this->get_logger(), "updateCameraTopic: empty topic for index %d, skipped", index);
        return;
    }

    subs_[index].reset();
    lastDisplayFrameNs_[static_cast<std::size_t>(index)].store(
        0, std::memory_order_relaxed);

    CamProvider *provider = providers_[index];  // pointer stable
    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        newTopic.toStdString(), rclcpp::SensorDataQoS().keep_last(1),
        [this, provider, index](const std::shared_ptr<const sensor_msgs::msg::Image> &msg)
        {
            if (!shouldProcessFrame(static_cast<std::size_t>(index))) {
                return;
            }
            try
            {
                const auto cvimg = cv_bridge::toCvShare(msg, "bgr8")->image;
                QImage qimg(cvimg.data, cvimg.cols, cvimg.rows, cvimg.step, QImage::Format_BGR888);
                provider->setImage(qimg.copy());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Image error: %s", e.what());
            }
        });

    subs_[index] = sub;

    QVariantMap cam = cameraList_[index].toMap();
    cam["name"] = cameraDisplayName(newTopic, index);
    cam["topic"] = newTopic;
    cameraList_[index] = cam;

    emit cameraListChanged();
    
    // Auto-save topic selections when changed
    saveTopicsToFile();
}

void CamNode::refreshTopics()
{
    QStringList qTopics = getAvailableImageTopics();

    if (qTopics.size() > maxCameras_)
        qTopics = qTopics.mid(0, maxCameras_);

    std::vector<std::string> topics;
    for (const QString &qstr : qTopics)
        topics.push_back(qstr.toStdString());

    setup(topics);
}

QVariantList CamNode::cameraList() const
{
    return cameraList_;
}

// Save current topic selections to file
void CamNode::saveTopicsToFile()
{
    if (configFilePath_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Config file path not set, cannot save");
        return;
    }

    std::ofstream file(configFilePath_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open config file for writing: %s", configFilePath_.c_str());
        return;
    }

    // Write each camera's topic
    for (const auto &cam : cameraList_) {
        QVariantMap camMap = cam.toMap();
        QString topic = camMap["topic"].toString();
        file << topic.toStdString() << std::endl;
    }

    file.close();
    RCLCPP_INFO(
        this->get_logger(), "Saved %d camera topics to %s",
        cameraList_.size(), configFilePath_.c_str());
}

// Load topic selections from file
std::vector<std::string> CamNode::loadTopicsFromFile()
{
    std::vector<std::string> topics;

    if (configFilePath_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Config file path not set, cannot load");
        return topics;
    }

    std::ifstream file(configFilePath_);
    if (!file.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Config file not found (first run?), will use defaults");
        return topics;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            topics.push_back(line);
        }
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu camera topics from %s", topics.size(), configFilePath_.c_str());
    return topics;
}

// Public method callable from QML
void CamNode::saveTopicSelections()
{
    saveTopicsToFile();
}

// Public method callable from QML - loads and applies saved topics
void CamNode::loadTopicSelections()
{
    auto topics = loadTopicsFromFile();
    if (!topics.empty()) {
        setup(topics);
        RCLCPP_INFO(this->get_logger(), "Restored camera topics from config");
    } else {
        // This production system always has two fixed Jetson CSI feeds.
        // Keep the two production topics fixed while DDS discovery settles;
        // auto-discovery used to bind only one camera intermittently.
        setup({"/cam0HP/image_overlay", "/cam1HP/image_overlay"});
        saveTopicsToFile();
        RCLCPP_INFO(
            this->get_logger(),
            "Using fixed Jetson AI topics: /cam0HP/image_overlay, /cam1HP/image_overlay");
    }
}
