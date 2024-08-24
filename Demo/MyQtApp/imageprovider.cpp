#include "imageprovider.h"

ImageProvider::ImageProvider()
    : QQuickImageProvider(QQuickImageProvider::Image)
{
    m_model = new Model(this);
    connect(&timer1, &QTimer::timeout, this, [this](){

        QImage tempImg(frame.data, frame.cols, frame.rows, QImage::Format_BGR888);
        QImage dispImg = tempImg.copy(0,0, tempImg.width(),tempImg.height());

        setUpdataImg(dispImg);

        qDebug("setUpdataImg");
        qDebug() << "onnxpath value:" << getOnnxpath();


    });


    connect(&timer1, &QTimer::timeout, this, [this](){
        // disconnect(&timer1, &QTimer::timeout, this, nullptr);

        m_model->yolov8ORT(frame,getOnnxpath().toStdString(),getLabels_txt_file().toStdString());
        QImage tempImg(frame.data, frame.cols, frame.rows, QImage::Format_BGR888);
        QImage dispImg = tempImg.copy(0,0, tempImg.width(),tempImg.height());
        setUpdataImg(dispImg);
        qDebug("yolov8ORT");
    });

}

QImage ImageProvider::requestImage(const QString &id, QSize *size, const QSize& requestedSize){
    return updataImg;
}

// 打开摄像头并显示视频流的函数
void ImageProvider::openCamera(int cameraIndex) {
    // 打开指定索引的摄像头
    cv::VideoCapture cap(cameraIndex);
    // cv::VideoCapture cap("D:\\Users\\Admin\\Videos\\Captures\\zidane.jpg 2024-08-24 02-34-28.mp4");

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return;
    }

    timer1.start(100); // Update interval
    timer2.start(100);



    while (true) {
        // 捕获视频帧
        cap >> frame;

        // cv::resize(frame,frame,cv::Size(1280,720));

        // 检查帧是否有效
        if (frame.empty()) {
            std::cerr << "Error: Could not grab frame." << std::endl;
            break;
        }


        // 显示视频帧
        // cv::imshow("Camera", frame);

        // 按 'q' 键退出
        if (cv::waitKey(30) == 'q') {
            break;
        }
    }

    // 释放摄像头和销毁所有窗口
    cap.release();
    cv::destroyAllWindows();
}

QImage ImageProvider::cvMatToQImage(const cv::Mat &mat)
{
    if (mat.type() == CV_8UC1) {
        // 8-bit, single-channel (grayscale) image
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
    } else if (mat.type() == CV_8UC3) {
        // 8-bit, 3-channel (BGR) image
        cv::Mat rgbMat;
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
        return QImage(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);
    } else if (mat.type() == CV_8UC4) {
        // 8-bit, 4-channel (BGRA) image
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGBA8888);
    } else {
        // Unsupported format
        return QImage();
    }
}

QImage ImageProvider::getUpdataImg() const
{
    return updataImg;
}

void ImageProvider::setUpdataImg(const QImage &newUpdataImg)
{
    if (updataImg == newUpdataImg)
        return;
    updataImg = newUpdataImg;
    emit updataImgChanged();
}

int ImageProvider::getStart() const
{
    return start;
}

void ImageProvider::setStart(int newStart)
{
    if (start == newStart)
        return;
    start = newStart;
    emit startChanged();
}

Model *ImageProvider::model() const
{
    return m_model;
}

void ImageProvider::setModel(Model *newModel)
{
    if (m_model == newModel)
        return;
    m_model = newModel;
    emit modelChanged();
}

QString ImageProvider::getOnnxpath() const
{
    qDebug() << "onnxpath value:" << onnxpath;

    return onnxpath;
}

void ImageProvider::setOnnxpath(const QString &newOnnxpath)
{
    if (onnxpath == newOnnxpath)
        return;
    onnxpath = newOnnxpath;

    qDebug() << "onnxPath value:" << newOnnxpath;

    emit onnxpathChanged();
}

QString ImageProvider::getLabels_txt_file() const
{
    return labels_txt_file;
}

void ImageProvider::setLabels_txt_file(const QString &newLabels_txt_file)
{
    if (labels_txt_file == newLabels_txt_file)
        return;
    labels_txt_file = newLabels_txt_file;
    emit labels_txt_fileChanged();
}

