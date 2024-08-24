#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <QThread>


class MyThread : public QObject
{
    Q_OBJECT
public:
    explicit MyThread(QObject *parent = nullptr);

    //打开摄像头
    void openCamera();

signals:


};

#endif // MYTHREAD_H
