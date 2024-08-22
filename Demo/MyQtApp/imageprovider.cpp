#include "imageprovider.h"

ImageProvider::ImageProvider(QObject *parent)
    : QObject{parent}
{}

// 打开摄像头并显示视频流的函数
void ImageProvider::openCamera(int cameraIndex) {
    // 打开指定索引的摄像头
    cv::VideoCapture cap(cameraIndex);

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return;
    }

    cv::Mat frame;
    while (true) {
        // 捕获视频帧
        cap >> frame;

        // 检查帧是否有效
        if (frame.empty()) {
            std::cerr << "Error: Could not grab frame." << std::endl;
            break;
        }

        // 显示视频帧
        cv::imshow("Camera", frame);

        // 按 'q' 键退出
        if (cv::waitKey(30) == 'q') {
            break;
        }
    }

    // 释放摄像头和销毁所有窗口
    cap.release();
    cv::destroyAllWindows();
}
