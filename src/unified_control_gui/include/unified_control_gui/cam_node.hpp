#ifndef CAM_NODE_HPP
#define CAM_NODE_HPP

#include <QObject>
#include <QQmlApplicationEngine>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QVariantList>
#include <QStringList>
#include <thread>
#include <atomic>
#include "cam_provider.hpp"

class CamNode : public QObject, public rclcpp::Node {
    Q_OBJECT
    Q_PROPERTY(QVariantList cameraList READ cameraList NOTIFY cameraListChanged)

public:
    CamNode(QQmlApplicationEngine &engine);
    ~CamNode() override;
    void setup(const std::vector<std::string> &topics);

    Q_INVOKABLE QStringList getAvailableImageTopics();
    Q_INVOKABLE void fetchAvailableTopicsAsync();   // non-blocking version
    Q_INVOKABLE void updateCameraTopic(int index, const QString &newTopic);
    Q_INVOKABLE void refreshTopics();
    Q_INVOKABLE void saveTopicSelections();
    Q_INVOKABLE void loadTopicSelections();

    QVariantList cameraList() const;

signals:
    void cameraListChanged();
    void availableTopicsChanged(QStringList topics);  // emitted async from bg thread

private:
    void saveTopicsToFile();
    std::vector<std::string> loadTopicsFromFile();

    // Providers pre-allocated 1 lần ở constructor, sống suốt đời CamNode.
    // QQmlEngine TAKES OWNERSHIP qua addImageProvider() — KHÔNG được delete thủ công.
    // Vì thế setup() chỉ rebind subscription, không touch providers_.
    std::vector<CamProvider *> providers_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
    QVariantList cameraList_;
    QQmlApplicationEngine *engine_;
    int maxCameras_ = 4;
    std::string configFilePath_ = "";
    std::atomic<bool> fetchingTopics_{false};  // prevent concurrent fetches
    std::thread discoveryThread_;              // joined in dtor — không detach
};

#endif // CAM_NODE_HPP
