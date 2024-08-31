// ThreadManager.cpp
#include "ThreadManager.h"
#include "Worker.h"

ThreadManager::ThreadManager(QObject *parent) : QObject(parent) {
    threadPool = new QThreadPool(this);
    threadPool->setMaxThreadCount(4);  // 设置线程池的最大线程数
}

ThreadManager::~ThreadManager() {
    threadPool->waitForDone();  // 等待线程池中的所有任务完成
    delete threadPool;
}

void ThreadManager::startTask() {
    Worker *worker = new Worker();
    // 使用 lambda 确保 worker 在任务完成后被正确删除
    threadPool->start(worker);
}
