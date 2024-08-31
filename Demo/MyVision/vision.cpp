#include "vision.h"

// Vision::Vision(QObject *parent)
//     : QObject{parent}
// {}


ModelManager *Vision::modelManger() const
{
    return m_modelManger;
}

void Vision::setModelManger(ModelManager *newModelManger)
{
    if (m_modelManger == newModelManger)
        return;
    m_modelManger = newModelManger;
    emit modelMangerChanged();
}

QString *Vision::status() const
{
    return m_status;
}

void Vision::setStatus(QString *newStatus)
{
    if (m_status == newStatus)
        return;
    m_status = newStatus;
    emit statusChanged();
}

