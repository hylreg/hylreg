// ThreadManager.h
#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include <QObject>
#include <QThreadPool>

class ThreadManager : public QObject {
    Q_OBJECT
public:
    explicit ThreadManager(QObject *parent = nullptr);
    ~ThreadManager();

    Q_INVOKABLE void startThreadWork();

private:
    QThreadPool *threadPool;
};

#endif // THREADMANAGER_H
