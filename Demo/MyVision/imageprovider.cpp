#include "ImageProvider.h"

ImageProvider::ImageProvider() : QQuickImageProvider(QQuickImageProvider::Image) {}

QImage ImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize) {
    Q_UNUSED(id);
    Q_UNUSED(requestedSize);

    if (size)
        *size = m_image.size();
    return m_image;
}

void ImageProvider::setImage(const QImage &newImage) {
    m_image = newImage;
    // emit imageChanged(); // 确保有信号发出
}
