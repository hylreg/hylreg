// Worker.cpp
#include "Worker.h"
#include <QThread>
#include <QDebug>

Worker::Worker(QObject *parent) : QObject(parent) {}

Worker::~Worker() {
    qDebug() << "Worker destroyed";
}

void Worker::doWork() {
    qDebug() << "Thread started in:" << QThread::currentThread();
    for (int i = 0; i < 5; ++i) {
        QThread::sleep(1);
        qDebug() << "Working..." << i;
    }
    emit workFinished();
    qDebug() << "Thread finished";
}
