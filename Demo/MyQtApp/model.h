#ifndef MODEL_H
#define MODEL_H

#include <QObject>
#include <qstring.h>
#include <QDebug>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <fstream>



class Model : public QObject
{
    Q_OBJECT
public:
    explicit Model(QObject *parent = nullptr);

    std::vector<std::string> readClassNames(std::string labels_txt_file);

    void yolov8ORT(cv::Mat &frame ,std::string onnxpath,std::string labels_txt_file);

    int start;
    int getStart() const;
    void setStart(int newStart);

    QString modelPath;
    QString getModelPath() const;
    void setModelPath(const QString &newModelPath);

    QString classNamePath;
    QString getClassNamePath() const;
    void setClassNamePath(const QString &newClassNamePath);

    void onModelPathChanged();

    // ImageProvider imageProvider;

    // std::string labels_txt_file;



signals:
    void startChanged();
    void modelPathChanged();
    void classNamePathChanged();

private:
    Q_PROPERTY(int start READ getStart WRITE setStart NOTIFY startChanged FINAL)
    Q_PROPERTY(QString modelPath READ getModelPath WRITE setModelPath NOTIFY modelPathChanged FINAL)
    Q_PROPERTY(QString classNamePath READ getClassNamePath WRITE setClassNamePath NOTIFY classNamePathChanged FINAL)

};

#endif // MODEL_H
