#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include "vision.h"
#include <QQmlContext>
#include "ImageProvider.h"
#include "ThreadManager.h"

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


    // QQuickStyle::setStyle("Material");
        // 注册 ModelManager 类
    // qmlRegisterType<ModelManager>("com.example", 1, 0, "ModelManager");

    // // 创建 ModelManager 对象
    // ModelManager *modelManager = new ModelManager();

    // // 获取 Vision 单例对象并设置 ModelManager
    // Vision *vision = Vision::create(&engine, nullptr);
    // vision->setModelManger(modelManager);

        // // 创建 Vision 对象作为顶级对象
        // Vision *vision;

        // // 创建 ModelManager 对象并设置到 Vision 中
        // ModelManager *modelManager = new ModelManager();
        // vision->setModelManger(modelManager);

        // // 将 Vision 作为 QML 的根对象传递
        // engine.rootContext()->setContextProperty("Vision", vision);

        // 创建 Vision 对象并设置 ModelManager

    Vision *vision = createVision();

    // 将 Vision 作为 QML 的根对象传递
    engine.rootContext()->setContextProperty("Vision", vision);

    // 通过静态工厂函数注册 QML 单例
    qmlRegisterSingletonType<Vision>("MyApp", 1, 0, "Vision", &Vision::create);


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
