#ifndef VISION_H
#define VISION_H

#include <QObject>
#include <QQmlEngine>

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

signals:


private:
    Vision() {}  // 私有构造函数，防止外部实例化
};

#endif // VISION_H
