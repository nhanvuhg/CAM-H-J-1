#include "unified_control_gui/language_controller.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QTranslator>
#include <QQmlEngine>
#include <QDebug>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <exception>

namespace {

constexpr std::array<const char *, 4> kVietnameseCatalogs = {
    "unified_control_gui_core_vi",
    "unified_control_gui_camera_vi",
    "unified_control_gui_cartridge_vi",
    "unified_control_gui_cpp_vi",
};

} // namespace

LanguageController::LanguageController(QQmlEngine *engine, QObject *parent)
    : QObject(parent), engine_(engine)
{
    QSettings settings;
    const QString saved = settings.value("ui/language", "en").toString();
    if (saved.trimmed().toLower() == "vi" && installVietnameseCatalogs())
        language_ = "vi";
}

LanguageController::~LanguageController()
{
    removeCatalogs();
}

bool LanguageController::setLanguage(const QString &language)
{
    const QString next = language.trimmed().toLower();
    if (next != "en" && next != "vi")
        return false;
    if (next == language_)
        return true;

    removeCatalogs();
    if (next == "vi" && !installVietnameseCatalogs()) {
        qWarning() << "Vietnamese UI catalogs are unavailable; keeping English";
        language_ = "en";
    } else {
        language_ = next;
    }

    QSettings settings;
    settings.setValue("ui/language", language_);
    settings.sync();

    if (engine_)
        engine_->retranslate();
    emit languageChanged();
    return language_ == next;
}

QString LanguageController::catalogDirectory() const
{
    const QString overrideDir = qEnvironmentVariable("UNIFIED_GUI_I18N_DIR");
    if (!overrideDir.isEmpty() && QDir(overrideDir).exists())
        return overrideDir;

    try {
        const QString share = QString::fromStdString(
            ament_index_cpp::get_package_share_directory("unified_control_gui"));
        const QString installed = QDir(share).filePath("i18n");
        if (QDir(installed).exists())
            return installed;
    } catch (const std::exception &error) {
        qWarning() << "Cannot resolve unified_control_gui share directory:"
                   << error.what();
    }

    return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(
        "../../share/unified_control_gui/i18n");
}

bool LanguageController::installVietnameseCatalogs()
{
    const QString directory = catalogDirectory();
    std::vector<std::unique_ptr<QTranslator>> loaded;
    loaded.reserve(kVietnameseCatalogs.size());

    for (const char *catalog : kVietnameseCatalogs) {
        auto translator = std::make_unique<QTranslator>();
        const QString catalogPath = QDir(directory).filePath(
            QString::fromLatin1(catalog) + QStringLiteral(".qm"));
        if (!translator->load(catalogPath) || translator->isEmpty()) {
            qWarning() << "Cannot load translation catalog" << catalog
                       << "or catalog is empty in" << directory;
            return false;
        }
        loaded.push_back(std::move(translator));
    }

    // Install as one atomic language pack. A partially loaded pack would mix
    // English and Vietnamese across pages, which is worse than a clean
    // fallback to English.
    std::vector<QTranslator *> installed;
    installed.reserve(loaded.size());
    for (const auto &translator : loaded) {
        if (!QCoreApplication::installTranslator(translator.get())) {
            for (auto it = installed.rbegin(); it != installed.rend(); ++it)
                QCoreApplication::removeTranslator(*it);
            qWarning() << "Cannot install the complete Vietnamese language pack;"
                          " keeping English";
            return false;
        }
        installed.push_back(translator.get());
    }
    translators_ = std::move(loaded);
    return true;
}

void LanguageController::removeCatalogs()
{
    for (const auto &translator : translators_)
        QCoreApplication::removeTranslator(translator.get());
    translators_.clear();
}
