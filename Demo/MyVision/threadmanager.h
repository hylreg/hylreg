// ThreadManager.h
#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include <QObject>
#include "MyThread.h"

class ThreadManager : public QObject {
    Q_OBJECT
public:
    explicit ThreadManager(QObject *parent = nullptr);
    ~ThreadManager();

    Q_INVOKABLE void startThreadWork();

private:
    MyThread *thread;
};

#endif // THREADMANAGER_H
