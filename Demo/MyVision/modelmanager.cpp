#include "modelmanager.h"

ModelManager::ModelManager(QObject *parent)
    : QObject{parent}
{}

QString ModelManager::AIAlgorithm() const
{
    return m_AIAlgorithm;
}

void ModelManager::setAIAlgorithm(const QString &newAIAlgorithm)
{
    if (m_AIAlgorithm == newAIAlgorithm)
        return;
    m_AIAlgorithm = newAIAlgorithm;
    emit AIAlgorithmChanged();
}




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

QString ModelManager::deployPlatform() const
{
    return m_deployPlatform;
}

void ModelManager::setDeployPlatform(const QString &newDeployPlatform)
{
    if (m_deployPlatform == newDeployPlatform)
        return;
    m_deployPlatform = newDeployPlatform;
    emit deployPlatformChanged();
}

int ModelManager::checkBoxState() const
{
    return m_checkBoxState;
}

void ModelManager::setCheckBoxState(int newCheckBoxState)
{
    if (m_checkBoxState == newCheckBoxState)
        return;
    m_checkBoxState = newCheckBoxState;
    emit checkBoxStateChanged();
}

void ModelManager::modelRun(QString AIAlgorithm, QString m_modelPath, QString m_classPath, QString m_labelPath, QString m_deployPlatform, int m_checkBoxState)
{
    // model.yolov8ORT();
}

void ModelManager::modelManagerTest()
{
    qDebug("modelManagerTest");
}


