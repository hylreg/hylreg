#ifndef THREADCONTROLLER_H
#define THREADCONTROLLER_H

#include <QObject>
#include <QThread>
#include "MyThread.h"

class ThreadController : public QObject
{
    Q_OBJECT
public:
    explicit ThreadController(QObject *parent = nullptr);
    // Q_INVOKABLE void startThread(); // 公开槽

signals:
    void startThread();

private:
    // QThread* thread;
    // MyThread* worker;
};

#endif // THREADCONTROLLER_H
