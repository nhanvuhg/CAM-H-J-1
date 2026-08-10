#ifndef UNIFIED_CONTROL_GUI_AUTH_CONTROLLER_HPP
#define UNIFIED_CONTROL_GUI_AUTH_CONTROLLER_HPP

#include <QObject>
#include <QString>

class AuthController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool authenticated READ authenticated NOTIFY authenticatedChanged)
    Q_PROPERTY(QString username READ username NOTIFY authenticatedChanged)
    Q_PROPERTY(QString role READ role NOTIFY authenticatedChanged)
    Q_PROPERTY(QString lastError READ lastError NOTIFY lastErrorChanged)

public:
    explicit AuthController(QObject *parent = nullptr);

    bool authenticated() const { return authenticated_; }
    QString username() const { return username_; }
    QString role() const { return role_; }
    QString lastError() const;

    Q_INVOKABLE bool login(const QString &username, const QString &password);
    Q_INVOKABLE void logout();
    void refreshTranslations();

signals:
    void authenticatedChanged();
    void lastErrorChanged();

private:
    void ensureUsersFile();
    bool readUser(const QString &username, const QString &password, QString *role) const;
    void setError(const QString &error);

    QString users_file_;
    bool authenticated_{false};
    QString username_;
    QString role_;
    QString last_error_;
};

#endif // UNIFIED_CONTROL_GUI_AUTH_CONTROLLER_HPP
