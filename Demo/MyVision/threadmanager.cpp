// ThreadManager.cpp
#include "ThreadManager.h"

ThreadManager::ThreadManager(QObject *parent) : QObject(parent) {
    thread = new MyThread(this);
}

ThreadManager::~ThreadManager() {
    if (thread->isRunning()) {
        thread->quit();
        thread->wait();
    }
}

void ThreadManager::startThreadWork() {
    if (!thread->isRunning()) {
        thread->start();  // 在按钮点击后才启动线程
    }
}
