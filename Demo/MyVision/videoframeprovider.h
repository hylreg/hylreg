#ifndef VIDEOFRAMEPROVIDER_H
#define VIDEOFRAMEPROVIDER_H

#include <QQuickImageProvider>
#include <opencv2/opencv.hpp>

class VideoFrameProvider : public QQuickImageProvider
{
public:
    VideoFrameProvider();

    ~VideoFrameProvider();

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;

    void updateFrame();

private:
    cv::VideoCapture cap;
    QImage m_frame;
    QImage cvMatToQImage(const cv::Mat &mat);
};

#endif // VIDEOFRAMEPROVIDER_H
