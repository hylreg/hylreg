#include "threadcontroller.h"

ThreadController::ThreadController(QObject *parent)
    : QObject{parent}
{

    // QThread* subThread = new QThread;
    // MyThread* worker = new MyThread;
    // worker->moveToThread(subThread);

    // connect(this, &ThreadController::startThread, worker, &MyThread::openCamera);
    // // connect(this, &ThreadController::startThread, this, [worker](){
    // //     worker->openCamera();
    // // });

    // subThread->start();


    // 线程池初始化，设置最大线程池数
    QThreadPool::globalInstance()->setMaxThreadCount(4);
    // 添加任务
    MyThreadPool* task = new MyThreadPool();
    // connect(this, &ThreadController::startThread, task, QThreadPool::globalInstance()->start(task));
    connect(this, &ThreadController::startThread, this, [task](){
        QThreadPool::globalInstance()->start(task);
    });
    // QThreadPool::globalInstance()->start(task);

}
