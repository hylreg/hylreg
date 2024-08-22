#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickImageProvider>
#include "imageprovider.h"


int main(int argc, char *argv[])
{

    qmlRegisterType<ImageProvider>("ImageProvider",1,0,"ImageProvider");




    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("Test", "Main");

    return app.exec();
}
