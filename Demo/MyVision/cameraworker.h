#ifndef CAMERAWORKER_H
#define CAMERAWORKER_H

#include <QThread>
#include <QMutex>
#include <QImage>
#include "imageprovider.h"
#include <QRunnable>
#include <modelmanager.h>
#include <opencv2/opencv.hpp>

class Vision;


class CameraWorker : public QObject, public QRunnable {
    Q_OBJECT
public:
    explicit CameraWorker(ImageProvider *provider);
    void run() override;

private:
    ImageProvider *provider;
    QMutex mutex; // 用于保护对 ImageProvider 的访问
    ModelManager *modelManager;
};

#endif // CAMERAWORKER_H
