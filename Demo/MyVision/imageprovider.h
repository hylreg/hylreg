#ifndef IMAGEPROVIDER_H
#define IMAGEPROVIDER_H

#include <QQuickImageProvider>
#include <QImage>
#include <QMutex>

class ImageProvider : public QQuickImageProvider {
    // Q_OBJECT
public:
    ImageProvider();

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
    void setImage(const QImage &newImage);

private:
    QImage m_image;
    QMutex m_mutex; // 用于保护 m_image 的访问
};

#endif // IMAGEPROVIDER_H
