#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickImageProvider>
#include "imageprovider.h"
#include "model.h"
#include <QQmlContext>



int main(int argc, char *argv[])
{



    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    // 注册新类型到 QML
    qmlRegisterType<ImageProvider>("ImageProvider",1,0,"ImageProvider");
    qmlRegisterType<Model>("Model",1,0,"Model");

    // 以类的实例形式引入
    ImageProvider *myImageProvider = new ImageProvider;
    engine.rootContext()->setContextProperty("myImageProviderQML", myImageProvider);


    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("Test", "Main");

    return app.exec();
}
