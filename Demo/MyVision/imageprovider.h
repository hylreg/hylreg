#ifndef IMAGEPROVIDER_H
#define IMAGEPROVIDER_H

#include <QObject>
#include <QImage>
#include <QQuickImageProvider>

class ImageProvider : public QQuickImageProvider {
    // Q_OBJECT
public:
    ImageProvider();

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;

    void setImage(const QImage &image);

// signals:
//     void imageChanged(); // 添加信号声明
private:
    QImage m_image;
};

#endif // IMAGEPROVIDER_H
