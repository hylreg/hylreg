#ifndef CAMERAWORKER_H
#define CAMERAWORKER_H

#include <QRunnable>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <QThread>

class ImageProvider;

class CameraWorker : public QObject, public QRunnable {
    Q_OBJECT
public:
    CameraWorker(ImageProvider *provider);
    void run() override;

// signals:
//     void finished();

private:
    ImageProvider *provider;
};

#endif // CAMERAWORKER_H
