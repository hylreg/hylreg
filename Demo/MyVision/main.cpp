#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include "vision.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    // QQuickStyle::setStyle("Material");
    // 通过静态工厂函数注册 QML 单例
    qmlRegisterSingletonType<Vision>("com.example.vision", 1, 0, "Vision", &Vision::create);

    QQmlApplicationEngine engine;
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("MyVision", "Main");

    return app.exec();
}
