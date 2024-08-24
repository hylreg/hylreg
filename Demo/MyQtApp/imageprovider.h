#ifndef IMAGEPROVIDER_H
#define IMAGEPROVIDER_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <QTimer>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QQuickImageProvider>
#include "model.h"

#include <onnxruntime_cxx_api.h>

class ImageProvider : public QQuickImageProvider
{
    Q_OBJECT
public:
    explicit ImageProvider();

    QImage requestImage(const QString &id, QSize *size, const QSize& requestedSize) override;
    Q_INVOKABLE void openCamera(int cameraIndex = 0);


    QTimer timer1;
    QTimer timer2;
    // 定义超时时间
    // int timeout = 10;

    QImage cvMatToQImage(const cv::Mat& mat);


    Q_INVOKABLE QImage getUpdataImg() const;
    void setUpdataImg(const QImage &newUpdataImg);


    void updateFrame();


    int start;

    Q_INVOKABLE int getStart() const;
    void setStart(int newStart);

    Q_INVOKABLE Model *model() const;
    void setModel(Model *newModel);

    Q_INVOKABLE QString getOnnxpath() const;
    Q_INVOKABLE void setOnnxpath(const QString &newOnnxpath);

    Q_INVOKABLE QString getLabels_txt_file() const;
    Q_INVOKABLE void setLabels_txt_file(const QString &newLabels_txt_file);

    // Ort::Session session;

    QString onnxpath;
    QString labels_txt_file;

signals:
    void updataImgChanged();
    void startChanged();
    void modelChanged();

    void onnxpathChanged();

    void labels_txt_fileChanged();

private:
    QImage updataImg;
    Q_PROPERTY(QImage updataImg READ getUpdataImg WRITE setUpdataImg NOTIFY updataImgChanged FINAL)
    Q_PROPERTY(int start READ getStart WRITE setStart NOTIFY startChanged FINAL)

    Model *m_model;
    Q_PROPERTY(Model *model READ model WRITE setModel NOTIFY modelChanged FINAL)

    cv::Mat frame;



    Q_PROPERTY(QString onnxpath READ getOnnxpath WRITE setOnnxpath NOTIFY onnxpathChanged FINAL)
    Q_PROPERTY(QString labels_txt_file READ getLabels_txt_file WRITE setLabels_txt_file NOTIFY labels_txt_fileChanged FINAL)
};

#endif // IMAGEPROVIDER_H
