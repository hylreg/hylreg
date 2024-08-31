#ifndef CAMERAWORKER_H
#define CAMERAWORKER_H

#include <QThread>
#include <QMutex>
#include <QImage>
#include "ImageProvider.h"
#include <QRunnable>

class CameraWorker : public QObject, public QRunnable {
    Q_OBJECT
public:
    explicit CameraWorker(ImageProvider *provider);
    void run() override;

private:
    ImageProvider *provider;
    QMutex mutex; // 用于保护对 ImageProvider 的访问
};

#endif // CAMERAWORKER_H
