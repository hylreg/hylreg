// ThreadManager.h
#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include <QObject>
#include <QThread>
#include "Worker.h"

class ThreadManager : public QObject {
    Q_OBJECT
public:
    explicit ThreadManager(QObject *parent = nullptr);
    ~ThreadManager();

    Q_INVOKABLE void startThreadWork();

private:
    QThread *thread;
    Worker *worker;
};

#endif // THREADMANAGER_H
