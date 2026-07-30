#ifndef SCALE_CONTROLLER_HPP
#define SCALE_CONTROLLER_HPP

#include <QObject>
#include <QVariantList>
#include <QString>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"

class ScaleController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(float currentWeight READ currentWeight NOTIFY currentWeightChanged)
    Q_PROPERTY(QString monitorStatus READ monitorStatus NOTIFY monitorStatusChanged)
    Q_PROPERTY(QString loadcellStatus READ loadcellStatus NOTIFY loadcellStatusChanged)
    Q_PROPERTY(QString calStatus READ calStatus NOTIFY calStatusChanged)
    Q_PROPERTY(float lastKnownCalWeight READ lastKnownCalWeight WRITE setLastKnownCalWeight NOTIFY lastKnownCalWeightChanged)
    Q_PROPERTY(float totalBatchWeight READ totalBatchWeight NOTIFY targetChanged)
    Q_PROPERTY(QString activeInkName READ activeInkName NOTIFY targetChanged)
    Q_PROPERTY(QString activeCartName READ activeCartName NOTIFY targetChanged)
    Q_PROPERTY(float minWeight READ minWeight NOTIFY targetChanged)
    Q_PROPERTY(float maxWeight READ maxWeight NOTIFY targetChanged)
    Q_PROPERTY(float inkCapacity READ inkCapacity WRITE setInkCapacity NOTIFY inkCapacityChanged)
    Q_PROPERTY(float currentMlFill READ currentMlFill NOTIFY currentMlFillChanged)
    Q_PROPERTY(int totalBatch READ totalBatch NOTIFY batchStatsChanged)
    Q_PROPERTY(int passBatch READ passBatch NOTIFY batchStatsChanged)
    Q_PROPERTY(int failBatch READ failBatch NOTIFY batchStatsChanged)
    Q_PROPERTY(int consecFails READ consecFails NOTIFY consecFailsChanged)
    Q_PROPERTY(bool scaleNodeConnected READ scaleNodeConnected NOTIFY scaleNodeConnectedChanged)
    Q_PROPERTY(bool zeroDriftPending READ zeroDriftPending NOTIFY zeroDriftPendingChanged)

public:
    explicit ScaleController(rclcpp::Node::SharedPtr node, QObject *parent = nullptr);

    bool scaleNodeConnected() const { return scale_node_connected_; }
    float currentWeight() const { return current_weight_; }
    QString monitorStatus() const { return monitor_status_; }
    QString loadcellStatus() const { return loadcell_status_; }
    QString calStatus() const { return cal_status_; }
    float lastKnownCalWeight() const { return last_known_cal_weight_; }

    float totalBatchWeight() const { return total_batch_weight_; }
    QString activeInkName() const { return active_ink_name_; }
    QString activeCartName() const { return active_cart_name_; }
    float minWeight() const { return min_weight_; }
    float maxWeight() const { return max_weight_; }
    float inkCapacity() const { return ink_capacity_; }
    float currentMlFill() const { return current_ml_fill_; }

    int totalBatch() const { return total_batch_; }
    int passBatch() const { return pass_batch_; }
    int failBatch() const { return fail_batch_; }
    int consecFails() const { return consec_fails_; }
    bool zeroDriftPending() const { return zero_drift_pending_; }

public slots:
    QVariantList getInkProfiles();
    QVariantList getCartProfiles();
    bool createInkProfile(const QString& name, float density);
    bool createInkProfileWithBatch(const QString& idInk, float density, const QString& lotPi, const QString& lotCi);
    bool createCartProfile(const QString& name, float density);
    bool deleteInkProfile(const QString& name);
    bool deleteCartProfile(const QString& name);
    
    void confirmTarget(const QString& inkName, float inkDensity, const QString& cartName, float cartDensity, float relativeError, float inkCapacity);
    void setInkCapacity(float capacity);
    
    void tare();
    void resetTare();
    void ackOverload();
    void dismissZeroDrift();   // operator chọn NO ở popup → giữ banner "TEMPO NOT YET TARED"
    void resetBatch();
    void startCalibration();
    void setKnownCalibration(float weight);
    void setLastKnownCalWeight(float weight);

signals:
    void currentWeightChanged();
    void monitorStatusChanged();
    void loadcellStatusChanged();
    void calStatusChanged();
    void lastKnownCalWeightChanged();
    
    void targetChanged();
    void inkCapacityChanged();
    void currentMlFillChanged();
    void profilesChanged();
    void batchStatsChanged();
    void consecFailsChanged();
    void scaleNodeConnectedChanged();
    void zeroDriftPendingChanged();
    
    // Alarms to trigger QML Popups
    void overloadAlarm();
    void zeroDriftAlarm();
    void calErrorAlarm();
    void calDoneAlarm();

private:
    rclcpp::Node::SharedPtr node_;

    // Values
    float current_weight_{0.0f};
    QString monitor_status_{"NO_SIGNAL"};
    QString loadcell_status_{"UNKNOWN"};
    QString cal_status_{"IDLE"};
    float last_known_cal_weight_{500.0f};
    QString active_profile_{""};
    QString active_ink_name_{"NONE"};
    QString active_cart_name_{"NONE"};
    float total_batch_weight_{0.0f};
    float min_weight_{0.0f};
    float max_weight_{0.0f};
    float ink_capacity_{0.0f};
    float current_ml_fill_{0.0f};
    int total_batch_{0};
    int pass_batch_{0};
    int fail_batch_{0};
    int consec_fails_{0};
    bool scale_node_connected_{false};
    qint64 last_weight_time_{0};
    QTimer* connection_timer_{nullptr};
    bool last_zero_drift_{false};
    bool zero_drift_pending_{false};   // true = operator đã chọn NO, chờ tare

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_active_profile_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_weight_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_min_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_max_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cal_weight_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tare_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tare_reset_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_overload_ack_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_batch_reset_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_ink_capacity_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_weight_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_monitor_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cal_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_batch_stats_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_consec_fails_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_overload_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_zero_drift_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ink_capacity_;

    // Services
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_cal_start_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_cal_set_known_;
};

#endif // SCALE_CONTROLLER_HPP
