#ifndef VISION_H
#define VISION_H

#include <QObject>
#include <QQmlEngine>
#include <modelmanager.h>
#include <threadmanager.h>
// #include <QSettings>
#include <settingsmanager.h>

class Vision : public QObject
{
    Q_OBJECT
    QML_ELEMENT       // 将 Foo 暴露给 QML
    QML_SINGLETON     // 将 Foo 注册为单例
public:
    // explicit Vision(QObject *parent = nullptr);

    // 静态方法，返回单例实例
    static Vision* create(QQmlEngine*, QJSEngine*) {
        static Vision s_instance;  // 创建并持有一个单例实例
        return &s_instance;
    }


    ModelManager *modelManger() const;
    void setModelManger(ModelManager *newModelManger);

    ThreadManager *threadManager() const;
    void setThreadManager(ThreadManager *newThreadManager);



    SettingsManager *getSettingsManager() const;
    void setSettingsManager(SettingsManager *newSettingsManager);

signals:

    void modelMangerChanged();
    void threadManagerChanged();
    void settingsManagerChanged();

private:
    Vision() {}  // 私有构造函数，防止外部实例化

    ModelManager *m_modelManger;
    ThreadManager *m_threadManager;
    SettingsManager *settingsManager;


    Q_PROPERTY(ModelManager *modelManger READ modelManger WRITE setModelManger NOTIFY modelMangerChanged FINAL)
    Q_PROPERTY(ThreadManager *threadManager READ threadManager WRITE setThreadManager NOTIFY threadManagerChanged FINAL)
    Q_PROPERTY(SettingsManager *settingsManager READ getSettingsManager WRITE setSettingsManager NOTIFY settingsManagerChanged FINAL)
};

#endif // VISION_H
