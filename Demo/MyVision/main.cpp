#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include "vision.h"
#include <QQmlContext>
#include "ImageProvider.h"
#include "ThreadManager.h"


int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
        QQmlApplicationEngine engine;


    // QQuickStyle::setStyle("Material");
    // 通过静态工厂函数注册 QML 单例
    qmlRegisterSingletonType<Vision>("com.example.vision", 1, 0, "Vision", &Vision::create);

    // 创建 ImageProvider 实例并注册到 QML
    ImageProvider *provider = new ImageProvider();
    // engine.rootContext()->setContextProperty("imageProvider", provider);
    engine.addImageProvider("camera", provider);

    // 创建 ThreadManager 实例并注册到 QML
    ThreadManager *threadManager = new ThreadManager(provider);
    engine.rootContext()->setContextProperty("threadManager", threadManager);



    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("MyVision", "Main");

    return app.exec();
}
