#ifndef MYTHREADPOOL_H
#define MYTHREADPOOL_H

#include <QObject>
#include <QRunnable>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <QThread>
#include <QThreadPool>

// class MyThreadPool : public QObject, QRunnable
class MyThreadPool : public QRunnable
{
    // Q_OBJECT
public:
    // explicit MyThreadPool(QObject *parent = nullptr);
    // explicit MyThreadPool();
    // ~MyThreadPool();
    void run() override;
signals:
};

#endif // MYTHREADPOOL_H
