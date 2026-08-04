#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDir>
#include <QFileInfo>
#include <QTimer>
#include "unified_control_gui/cam_node.hpp"
#include "unified_control_gui/robot_controller.hpp"
#include "unified_control_gui/cartridge_controller.hpp"
#include "unified_control_gui/scale_controller.hpp"
#include "unified_control_gui/hp_controller.hpp"
#include "unified_control_gui/auth_controller.hpp"
#include <thread>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);

    rclcpp::init(argc, argv);
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    // Add qrc:/icons path for icons
    engine.addImportPath("qrc:/");

    auto camNode = std::make_shared<CamNode>(engine);
    camNode->loadTopicSelections();
    
    engine.rootContext()->setContextProperty("camNode", camNode.get());

    auto robotController = new RobotController(camNode);
    engine.rootContext()->setContextProperty("robotController", robotController);

    auto cartridgeController = new CartridgeController(camNode);
    engine.rootContext()->setContextProperty("cartridgeController", cartridgeController);

    auto scaleController = new ScaleController(camNode);
    engine.rootContext()->setContextProperty("scaleController", scaleController);

    auto hpNode = std::make_shared<rclcpp::Node>("hp_controller_node");
    auto hpController = new HpController(hpNode);
    engine.rootContext()->setContextProperty("hpController", hpController);

    auto authController = new AuthController(&app);
    engine.rootContext()->setContextProperty("authController", authController);

    // Load QML from the current user's workspace so Pi and Jetson render the
    // same live source tree. Fall back to qrc for installed-only deployments.
    const QString qmlPath =
        QDir::home().filePath("ros2_ws/src/unified_control_gui/qml/Main.qml");
    if (QFileInfo::exists(qmlPath)) {
        qDebug() << "Loading QML from filesystem:" << qmlPath;
        engine.load(QUrl::fromLocalFile(qmlPath));
    } else {
        qDebug() << "Loading QML from qrc (fallback)";
        engine.load(QUrl(QStringLiteral("qrc:/qml/Main.qml")));
    }

    if (engine.rootObjects().isEmpty())
        return -1;

    std::thread rosThread([=]() {
        rclcpp::spin(camNode);
    });

    std::thread hpRosThread([=]() {
        rclcpp::spin(hpNode);
    });

    // rclcpp catches SIGINT/SIGTERM and marks its context as stopped, but that
    // alone does not end Qt's event loop. Poll from the GUI thread so service
    // managers and the launcher can stop the GUI gracefully without SIGKILL.
    QTimer rosShutdownTimer;
    QObject::connect(&rosShutdownTimer, &QTimer::timeout, &app, [&app]() {
        if (!rclcpp::ok()) {
            app.quit();
        }
    });
    rosShutdownTimer.start(200);

    // Shutdown ordering:
    //   1. app.exec() returns khi QML window đóng.
    //   2. rclcpp::shutdown() đánh dấu shutdown — spin() trong 2 thread sẽ thoát.
    //   3. join() đảm bảo cả 2 thread thực sự kết thúc trước khi engine/node destruct.
    //  KHÔNG detach: nếu thread vẫn đang spin lúc node bị destruct sẽ crash.
    int rc = app.exec();
    rclcpp::shutdown();
    if (rosThread.joinable())   rosThread.join();
    if (hpRosThread.joinable()) hpRosThread.join();
    return rc;
}
