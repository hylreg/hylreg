#include "imageprovider.h"

ImageProvider::ImageProvider() : QQuickImageProvider(QQuickImageProvider::Image) {}

QImage ImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize) {
    Q_UNUSED(id);
    Q_UNUSED(requestedSize);

    QMutexLocker locker(&m_mutex); // 自动加锁


    if (size)
        *size = m_image.size();
    return m_image;
}

void ImageProvider::setImage(const QImage &newImage) {
    {
        QMutexLocker locker(&m_mutex); // 自动加锁
        m_image = newImage;
    }
    // emit imageChanged(); // 确保有信号发出
}
