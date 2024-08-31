#include "ThreadManager.h"

ThreadManager::ThreadManager(ImageProvider *provider, QObject *parent)
    : QObject(parent), imageProvider(provider), cameraWorker(nullptr) {}

ThreadManager::~ThreadManager() {
    stopCamera();
}

void ThreadManager::startCamera() {
    if (!cameraWorker) {

        cameraWorker = new CameraWorker(imageProvider);
        threadPool.start(cameraWorker);
    }
}

void ThreadManager::stopCamera() {
    if (cameraWorker) {
        threadPool.clear(); // 清除线程池中的任务
        delete cameraWorker;
        cameraWorker = nullptr;
    }
}
