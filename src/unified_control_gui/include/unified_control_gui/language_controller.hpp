#ifndef LANGUAGE_CONTROLLER_HPP
#define LANGUAGE_CONTROLLER_HPP

#include <QObject>
#include <QString>

#include <memory>
#include <vector>

class QQmlEngine;
class QTranslator;

class LanguageController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString language READ language NOTIFY languageChanged)

public:
    explicit LanguageController(QQmlEngine *engine, QObject *parent = nullptr);
    ~LanguageController() override;

    QString language() const { return language_; }
    Q_INVOKABLE bool setLanguage(const QString &language);

signals:
    void languageChanged();

private:
    QString catalogDirectory() const;
    bool installVietnameseCatalogs();
    void removeCatalogs();

    QQmlEngine *engine_{nullptr};
    QString language_{"en"};
    std::vector<std::unique_ptr<QTranslator>> translators_;
};

#endif // LANGUAGE_CONTROLLER_HPP
