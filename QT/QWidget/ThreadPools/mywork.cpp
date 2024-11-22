#include "mywork.h"



MyWork::MyWork(QObject *parent)
{
    // 任务执行完毕,该对象自动销毁
    setAutoDelete(true);
}

void MyWork::run(){
    while (!stopMyWork) {
        qDebug("MyWork");
        QThread::sleep(1);
    }
}

void MyWork::stop()
{
    stopMyWork = true;
}
