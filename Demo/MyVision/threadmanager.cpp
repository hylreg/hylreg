#include "ThreadManager.h"

ThreadManager::ThreadManager(ImageProvider *provider, QObject *parent)
    : QObject(parent), imageProvider(provider), cameraWorker(nullptr) {}

ThreadManager::~ThreadManager() {
    stopCamera();
}

void ThreadManager::startCamera() {
    QMutexLocker locker(&mutex);

    if (!cameraWorker) {
        cameraWorker = new CameraWorker(imageProvider);
        threadPool.start(cameraWorker);
    }
}

void ThreadManager::stopCamera() {
    QMutexLocker locker(&mutex);

    if (cameraWorker) {
        threadPool.clear(); // 清除线程池中的任务
        delete cameraWorker;
        cameraWorker = nullptr;
    }
}
