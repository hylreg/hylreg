#include "modelmanager.h"

ModelManager::ModelManager(QObject *parent)
    : QObject{parent}
{}

QString ModelManager::modelPath() const
{
    return m_modelPath;
}

void ModelManager::setModelPath(const QString &newModelPath)
{
    if (m_modelPath == newModelPath)
        return;
    m_modelPath = newModelPath;
    emit modelPathChanged();
}

QString ModelManager::classPath() const
{
    return m_classPath;
}

void ModelManager::setClassPath(const QString &newClassPath)
{
    if (m_classPath == newClassPath)
        return;
    m_classPath = newClassPath;
    emit classPathChanged();
}

QString ModelManager::labelPath() const
{
    return m_labelPath;
}

void ModelManager::setLabelPath(const QString &newLabelPath)
{
    if (m_labelPath == newLabelPath)
        return;
    m_labelPath = newLabelPath;
    emit labelPathChanged();
}
