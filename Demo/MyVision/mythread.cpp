// MyThread.cpp
#include "MyThread.h"
#include <QDebug>

void Worker::doWork() {
    qDebug() << "Thread started";
    for (int i = 0; i < 5; ++i) {
        QThread::sleep(1);
        qDebug() << "Working..." << i;
    }
    qDebug() << "Thread finished";
}

MyThread::MyThread(QObject *parent) : QThread(parent) {}

MyThread::~MyThread() {
    quit();
    wait();
}

void MyThread::run() {
    Worker worker;
    connect(this, &MyThread::startWork, &worker, &Worker::doWork);
    emit startWork();
    exec();  // 开启事件循环
}
