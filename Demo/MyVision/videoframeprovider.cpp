#include "videoframeprovider.h"

VideoFrameProvider::VideoFrameProvider(): QQuickImageProvider(QQuickImageProvider::Image)
{
    cap.open(0); // 打开默认摄像头
}

VideoFrameProvider::~VideoFrameProvider()
{
    cap.release();
}

QImage VideoFrameProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    Q_UNUSED(id);
    Q_UNUSED(requestedSize);

    updateFrame();

    if (size)
        *size = m_frame.size();

    return m_frame;
}

void VideoFrameProvider::updateFrame()
{
    cv::Mat mat;
    cap >> mat;

    if (!mat.empty()) {
        m_frame = cvMatToQImage(mat);
    }
}

QImage VideoFrameProvider::cvMatToQImage(const cv::Mat &mat)
{
    switch (mat.type()) {
    case CV_8UC1:
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
    case CV_8UC3:
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
    case CV_8UC4:
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
    default:
        return QImage();
    }
}
