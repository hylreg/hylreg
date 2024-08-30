#include "Worker.h"
#include <QDebug>
#include <QThread>

void Worker::run() {
    qDebug() << "Task running in thread:" << QThread::currentThreadId();
}
