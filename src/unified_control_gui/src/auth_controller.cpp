#include "unified_control_gui/auth_controller.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcessEnvironment>
#include <QStandardPaths>
#include <QDebug>

AuthController::AuthController(QObject *parent) : QObject(parent)
{
    const QString default_users_file =
        QDir::home().filePath("ros2_ws/src/unified_control_gui/fill_hp_users.json");
    users_file_ = QProcessEnvironment::systemEnvironment().value(
        "FILL_HP_USERS_FILE", default_users_file);
    ensureUsersFile();
}

void AuthController::ensureUsersFile()
{
    if (QFileInfo::exists(users_file_))
        return;

    // Never ship a usable fallback account in source code. The real users
    // file is machine-local and excluded from Git.
    const QJsonObject root{{"users", QJsonArray{}}};

    QFile file(users_file_);
    QDir().mkpath(QFileInfo(users_file_).absolutePath());
    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
        file.close();
        file.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner);
        qWarning() << "Created empty users file at" << users_file_
                   << "- add accounts before using GUI login";
    }
}

bool AuthController::readUser(const QString &username, const QString &password, QString *role) const
{
    QFile file(users_file_);
    if (!file.open(QIODevice::ReadOnly))
        return false;

    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    const QJsonArray users = document.isObject()
        ? document.object().value("users").toArray() : document.array();
    for (const QJsonValue &value : users) {
        const QJsonObject user = value.toObject();
        if (user.value("username").toString().trimmed() == username &&
            user.value("password").toString() == password) {
            *role = user.value("role").toString("viewer").trimmed().toLower();
            return true;
        }
    }
    return false;
}

void AuthController::setError(const QString &error)
{
    if (last_error_ == error)
        return;
    last_error_ = error;
    emit lastErrorChanged();
}

bool AuthController::login(const QString &username, const QString &password)
{
    const QString normalized = username.trimmed();
    QString matched_role;
    if (normalized.isEmpty() || password.isEmpty() || !readUser(normalized, password, &matched_role)) {
        setError(QStringLiteral("Sai tài khoản hoặc mật khẩu"));
        return false;
    }

    username_ = normalized;
    role_ = matched_role;
    authenticated_ = true;
    setError(QString());
    emit authenticatedChanged();
    return true;
}

void AuthController::logout()
{
    if (!authenticated_)
        return;
    authenticated_ = false;
    username_.clear();
    role_.clear();
    setError(QString());
    emit authenticatedChanged();
}
