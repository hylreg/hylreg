#ifndef VISION_H
#define VISION_H

#include <QObject>
#include <QQmlEngine>
#include <modelmanager.h>

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



    QString *status() const;
    void setStatus(QString *newStatus);

signals:

    void modelMangerChanged();

    void statusChanged();

private:
    Vision() {}  // 私有构造函数，防止外部实例化

    ModelManager *m_modelManger;
    QString *m_status;
    Q_PROPERTY(ModelManager *modelManger READ modelManger WRITE setModelManger NOTIFY modelMangerChanged FINAL)
    Q_PROPERTY(QString *status READ status WRITE setStatus NOTIFY statusChanged FINAL)
};

#endif // VISION_H
