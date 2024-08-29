#include "vision.h"

// Vision::Vision(QObject *parent)
//     : QObject{parent}
// {}

QString Vision::status() const
{
    return m_status;
}

void Vision::setStatus(const QString &newStatus)
{
    if (m_status == newStatus)
        return;
    m_status = newStatus;
    emit statusChanged();
}

QString Vision::getStatusInfo() const {
    return QString("Current status is: %1").arg(m_status);
}
