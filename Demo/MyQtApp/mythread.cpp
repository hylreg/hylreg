#include "mythread.h"

MyThread::MyThread(QObject *parent)
    : QObject{parent}
{}

void MyThread::openCamera()
{
    qDebug() << "当前线程对象的地址: " << QThread::currentThread();

    //opencv打开摄像头
    cv::VideoCapture cap(0);
    if(!cap.isOpened()){
        qDebug()<<"open camera failed";
        return;
    }
    while(1){
        cv::Mat frame;
        cap>>frame;
        if(frame.empty()){
            qDebug()<<"read frame failed";
            break;
        }
        cv::imshow("camera",frame);
        if(cv::waitKey(30)==27){
            qDebug()<<"esc";
            break;
        }
    }

}
