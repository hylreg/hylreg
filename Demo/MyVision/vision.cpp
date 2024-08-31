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


ThreadManager *Vision::threadManager() const
{
    return m_threadManager;
}

void Vision::setThreadManager(ThreadManager *newThreadManager)
{
    if (m_threadManager == newThreadManager)
        return;
    m_threadManager = newThreadManager;
    emit threadManagerChanged();
}

SettingsManager *Vision::getSettingsManager() const
{
    return settingsManager;
}

void Vision::setSettingsManager(SettingsManager *newSettingsManager)
{
    if (settingsManager == newSettingsManager)
        return;
    settingsManager = newSettingsManager;
    emit settingsManagerChanged();
}
