#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include <QObject>
#include <QThreadPool>
#include "CameraWorker.h"
#include "ImageProvider.h"
#include <QMutex>

class ThreadManager : public QObject {
    Q_OBJECT
public:
    explicit ThreadManager(ImageProvider *provider, QObject *parent = nullptr);
    ~ThreadManager();

    Q_INVOKABLE void startCamera();
    Q_INVOKABLE void stopCamera();

private:
    QThreadPool threadPool;
    CameraWorker *cameraWorker;
    ImageProvider *imageProvider;

    mutable QMutex mutex;
};

#endif // THREADMANAGER_H
