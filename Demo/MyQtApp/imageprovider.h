#ifndef IMAGEPROVIDER_H
#define IMAGEPROVIDER_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <iostream>

class ImageProvider : public QObject
{
    Q_OBJECT
public:
    explicit ImageProvider(QObject *parent = nullptr);

    Q_INVOKABLE void openCamera(int cameraIndex = 0);

signals:
};

#endif // IMAGEPROVIDER_H
