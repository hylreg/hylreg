// Worker.cpp
#include "Worker.h"
#include <QDebug>
#include <QThread>

Worker::Worker() {}

void Worker::run() {
    qDebug() << "Thread started in:" << QThread::currentThread();
    for (int i = 0; i < 5; ++i) {
        QThread::sleep(1);
        qDebug() << "Working..." << i;
    }
    qDebug() << "Thread finished";
}
