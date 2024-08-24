#include "threadcontroller.h"

ThreadController::ThreadController(QObject *parent)
    : QObject{parent}
{

    QThread* subThread = new QThread;
    MyThread* worker = new MyThread;
    worker->moveToThread(subThread);

    connect(this, &ThreadController::startThread, worker, &MyThread::openCamera);
    // connect(this, &ThreadController::startThread, this, [worker](){
    //     worker->openCamera();
    // });

    subThread->start();
}
