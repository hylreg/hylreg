#ifndef MODEL_H
#define MODEL_H

#include <QObject>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <QDebug>
#include "yolo_v8.h"




class Model : public QObject
{
    Q_OBJECT
public:
    explicit Model(QObject *parent = nullptr);

    void yolov8ORT(cv::Mat &frame, std::string onnxpath, std::string labels_txt_file);
    std::vector<std::string> readClassNames(std::string labels_txt_file);
    void modelTest();
    void modelTest(QString name);

    // MyYOLO_V8::DetectTest();
    MyYOLO_V8 *myyolov8;

signals:

private:


};

#endif // MODEL_H
