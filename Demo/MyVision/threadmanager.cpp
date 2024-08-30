// ThreadManager.cpp
#include "ThreadManager.h"

ThreadManager::ThreadManager(QObject *parent) : QObject(parent) {
    thread = new QThread(this);
    worker = new Worker();

    // 将 worker 对象移到新的线程中
    worker->moveToThread(thread);

    // 连接信号槽
    connect(thread, &QThread::started, worker, &Worker::doWork);
    connect(worker, &Worker::workFinished, thread, &QThread::quit);
    connect(worker, &Worker::workFinished, worker, &Worker::deleteLater);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);
}

ThreadManager::~ThreadManager() {
    if (thread->isRunning()) {
        thread->quit();
        thread->wait();
    }
}

void ThreadManager::startThreadWork() {
    if (!thread->isRunning()) {
        thread->start();
    }
}
