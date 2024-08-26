#ifndef MESSAGE_H
#define MESSAGE_H

#include <QObject>
#include <QQmlEngine>
#include <QtQml/qqmlregistration.h>

class Message : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString author READ author WRITE setAuthor NOTIFY authorChanged)
    Q_PROPERTY(QDateTime creationDate READ creationDate WRITE setCreationDate NOTIFY creationDateChanged)
    QML_ELEMENT
public:
    explicit Message(QObject *parent = nullptr);

    QString author() const;
    void setAuthor(const QString &author);

    QDateTime creationDate() const;
    void setCreationDate(const QDateTime &date);

signals:
    void authorChanged();
    void creationDateChanged();

private:
    QString m_author;
    QDateTime m_creationDate;
};

#endif // MESSAGE_H
