#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include <QObject>
#include <QThreadPool>

class Worker; // Forward declaration

class ThreadManager : public QObject {
    Q_OBJECT
public:
    explicit ThreadManager(QObject *parent = nullptr);

public slots:
    void startTask();

private:
    QThreadPool threadPool;
};

#endif // THREADMANAGER_H
