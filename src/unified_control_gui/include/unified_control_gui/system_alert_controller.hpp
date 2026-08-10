#ifndef SYSTEM_ALERT_CONTROLLER_HPP
#define SYSTEM_ALERT_CONTROLLER_HPP

#include <QObject>
#include <QMap>
#include <QHash>
#include <QString>
#include <QTimer>
#include <QVariantList>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class SystemAlertController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList activeAlerts READ activeAlerts NOTIFY alertsChanged)
    Q_PROPERTY(int errorCount READ errorCount NOTIFY alertsChanged)
    Q_PROPERTY(int warningCount READ warningCount NOTIFY alertsChanged)
    Q_PROPERTY(int unacknowledgedWarningCount READ unacknowledgedWarningCount NOTIFY alertsChanged)
    Q_PROPERTY(bool canStart READ canStart NOTIFY alertsChanged)
    Q_PROPERTY(QString startBlockReason READ startBlockReason NOTIFY alertsChanged)
    Q_PROPERTY(QString operationMode READ operationMode NOTIFY operationModeChanged)
    Q_PROPERTY(bool scaleIgnored READ scaleIgnored NOTIFY scaleIgnoredChanged)

public:
    explicit SystemAlertController(rclcpp::Node::SharedPtr node, QObject *parent = nullptr);

    QVariantList activeAlerts() const;
    int errorCount() const;
    int warningCount() const;
    int unacknowledgedWarningCount() const;
    bool canStart() const;
    QString startBlockReason() const;
    QString operationMode() const { return operation_mode_; }
    bool scaleIgnored() const { return scale_ignored_; }

    Q_INVOKABLE void setOperationMode(const QString &mode);
    Q_INVOKABLE void setScaleIgnored(bool ignored);
    Q_INVOKABLE bool prepareStart(const QString &mode);
    Q_INVOKABLE bool acknowledgeWarning(const QString &id);
    Q_INVOKABLE int acknowledgeAllWarnings();
    Q_INVOKABLE void requestAttention();

signals:
    void alertsChanged();
    void operationModeChanged();
    void scaleIgnoredChanged();
    void attentionRequested();

private:
    struct Alert {
        QString id;
        QString source;
        QString level;
        QString area;
        QString title;
        QString message;
        QString action;
        QString time;
        bool acknowledged{false};
        bool connection_issue{false};
        qint64 sequence{0};
    };

    rclcpp::Node::SharedPtr node_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    QMap<QString, Alert> alerts_;
    QString operation_mode_{"manual"};
    qint64 next_sequence_{1};
    bool robot_fault_state_seen_{false};
    bool feeder_fault_state_seen_{false};
    bool scale_ignored_{false};
    QHash<QString, qint64> heartbeat_seen_ms_;
    QTimer *heartbeat_timer_{nullptr};
    qint64 heartbeat_watch_started_ms_{0};

    void observeString(const QString &source, const QString &value);
    void observeBool(const QString &source, bool value);
    void observeFeederNotification(const QString &value);
    void upsertAlert(const QString &id, const QString &source,
                     const QString &baseLevel, const QString &area,
                     const QString &title, const QString &message,
                     bool connectionIssue = false);
    void clearAlert(const QString &id);
    void clearAreaErrors(const QString &area);
    void clearArea(const QString &area);
    void clearPrefix(const QString &prefix);
    void reclassifyConnectionAlerts();
    void markHeartbeat(const QString &source);
    void checkHeartbeats();

    QString effectiveLevel(const QString &baseLevel, bool connectionIssue) const;
    static QString actionForArea(const QString &area);
    static QString stableCameraMessage(const QString &source, const QString &value);
    static QString normalized(const QString &value);
    static bool isHealthy(const QString &value);
    static bool hasCriticalToken(const QString &value);
    static bool isConnectionIssue(const QString &value);
    static bool isProductionMode(const QString &mode);
};

#endif // SYSTEM_ALERT_CONTROLLER_HPP
