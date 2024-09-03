#include <iostream>
#include <iomanip>
#include "YOLOv8-ONNXRuntime-CPP/inference.h"
#include <filesystem>
#include <fstream>
#include <random>
#include <QString>

class MyYOLO_V8
{
public:
    MyYOLO_V8() {}
    void Detector(YOLO_V8 *&p,cv::Mat &img);
    void Classifier(YOLO_V8 *&p);
    int ReadCocoYaml(YOLO_V8 *&p,QString classPath);
    void CreateDetectSession(YOLO_V8 *&p,QString modelPath,QString classPath);
    void ClsTest();



};
