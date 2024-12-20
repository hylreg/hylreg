#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include "vision.h"
#include <QQmlContext>
#include "imageprovider.h"
#include "threadmanager.h"

Vision* createVision() {
    Vision *vision = Vision::create(nullptr, nullptr);
    ModelManager *modelManager = new ModelManager();
    vision->setModelManger(modelManager);
    return vision;
}


int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
        QQmlApplicationEngine engine;

    // // 创建 ImageProvider 实例并注册到 QML
    ImageProvider *provider = new ImageProvider();
    engine.addImageProvider("camera", provider);




    // 创建 Vision 单例
    Vision *vision = Vision::create(nullptr, nullptr);

    // 创建 ModelManager、ThreadManager 和 QSettings 对象
    ModelManager *modelManager = new ModelManager();
    ThreadManager *threadManager = new ThreadManager(provider);
    // QSettings *settingsManager = new QSettings("settings.ini", QSettings::IniFormat);
    SettingsManager *settingsManager = new SettingsManager("settings.ini");

    // 设置 Vision 的各个属性
    vision->setModelManger(modelManager);
    vision->setThreadManager(threadManager);
    vision->setSettingsManager(settingsManager);

    // 注册 Vision 单例类型
    qmlRegisterSingletonType<Vision>("MyApp", 1, 0, "MyApp", [](QQmlEngine*, QJSEngine*) -> QObject* {
        return Vision::create(nullptr, nullptr);
    });

    // // 通过静态工厂函数注册 QML 单例
    // qmlRegisterSingletonType<Vision>("MyApp", 1, 0, "Vision", &Vision::create);
    // // 设置 Vision 的 ModelManager
    // Vision *vision = Vision::create(nullptr, nullptr);
    // // 创建并设置 ModelManager 对象
    // ModelManager *modelManager = new ModelManager();
    // vision->setModelManger(modelManager);

    // // 创建 ImageProvider 实例并注册到 QML
    // ImageProvider *provider = new ImageProvider();
    // engine.addImageProvider("camera", provider);

    // // 创建 ThreadManager 实例并注册到 QML
    // ThreadManager *threadManager = new ThreadManager(provider);
    // vision->setThreadManager(threadManager);

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("MyVision", "Main");

    return app.exec();
}
