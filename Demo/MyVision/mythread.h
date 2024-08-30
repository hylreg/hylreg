// MyThread.h
#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QThread>
#include <QObject>

class Worker : public QObject {
    Q_OBJECT
public slots:
    void doWork();
};

class MyThread : public QThread {
    Q_OBJECT
public:
    explicit MyThread(QObject *parent = nullptr);
    ~MyThread();
protected:
    void run() override;
signals:
    void startWork();
};

#endif // MYTHREAD_H
