#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include "vision.h"
#include "VideoFrameProvider.h"


int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
        QQmlApplicationEngine engine;


    // QQuickStyle::setStyle("Material");
    // 通过静态工厂函数注册 QML 单例
    qmlRegisterSingletonType<Vision>("com.example.vision", 1, 0, "Vision", &Vision::create);

    // 注册 VideoFrameProvider 给 QML 使用
    VideoFrameProvider *videoFrameProvider = new VideoFrameProvider();
    engine.addImageProvider(QLatin1String("videoframe"), videoFrameProvider);



    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("MyVision", "Main");

    return app.exec();
}
