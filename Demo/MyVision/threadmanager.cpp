#include "ThreadManager.h"
#include "Worker.h"
#include <QDebug>

ThreadManager::ThreadManager(QObject *parent) : QObject(parent) {
    threadPool.setMaxThreadCount(5);
}

void ThreadManager::startTask() {
    qDebug() << "Starting task...";
    Worker *worker = new Worker();
    threadPool.start(worker);
}
