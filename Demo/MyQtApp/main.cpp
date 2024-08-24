#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickImageProvider>
#include "imageprovider.h"
#include "model.h"
#include <QQmlContext>
#include <mythread.h>
#include <QThread>
#include <threadcontroller.h>



int main(int argc, char *argv[])
{
    qDebug() << "主线程对象的地址: " << QThread::currentThread();

    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    // 注册新类型到 QML
    qmlRegisterType<ImageProvider>("ImageProvider",1,0,"ImageProvider");
    qmlRegisterType<Model>("Model",1,0,"Model");

    // 以类的实例形式引入
    ImageProvider *myImageProvider = new ImageProvider;
    engine.rootContext()->setContextProperty("myImageProviderQML", myImageProvider);
    // engine.addImageProvider("myImageProvider1", myImageProvider);
    engine.addImageProvider(QLatin1String("myImage"), myImageProvider);

    ThreadController controller;
    engine.rootContext()->setContextProperty("threadController", &controller);


    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("Test", "Main");

    return app.exec();
}
