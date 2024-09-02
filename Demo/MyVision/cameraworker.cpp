#include "CameraWorker.h"
#include <vision.h>

CameraWorker::CameraWorker(ImageProvider *provider) : provider(provider) {}

void CameraWorker::run() {

    cv::VideoCapture cap(0); // 打开默认摄像头
    if (!cap.isOpened()) {
        return; // 摄像头打开失败
    }

    while (!QThread::currentThread()->isInterruptionRequested()) {
        cv::Mat frame;
        cap >> frame; // 捕获帧
        if (frame.empty()) {
            continue;
        }

        // 转换到 QImage
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        Vision* visionInstance = Vision::create(nullptr,nullptr);
        visionInstance->modelManger()->model.yolov8ORT(frame, visionInstance->modelManger()->modelPath().toStdString(), visionInstance->modelManger()->classPath().toStdString());

        QImage image =QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888).rgbSwapped();

        {
            QMutexLocker locker(&mutex); // 自动加锁

        //     modelManager->modelManagerTest();
        modelManager->model.modelTest(visionInstance->modelManger()->modelPath());
        modelManager->model.modelTest(visionInstance->modelManger()->classPath());


            provider->setImage(image);
        }

        // 添加适当的延迟来控制帧率
        QThread::msleep(30); // 30 ms 延迟
    }
}

