#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include <QObject>
#include <QRunnable>
#include <QThread>
#include <model.h>


class ModelManager : public QObject
{
    Q_OBJECT
public:
    explicit ModelManager(QObject *parent = nullptr);

    Q_INVOKABLE QString AIAlgorithm() const;
    Q_INVOKABLE void setAIAlgorithm(const QString &newAIAlgorithm);

    Q_INVOKABLE QString modelPath() const;
    Q_INVOKABLE void setModelPath(const QString &newModelPath);

    Q_INVOKABLE QString classPath() const;
    Q_INVOKABLE void setClassPath(const QString &newClassPath);

    Q_INVOKABLE QString labelPath() const;
    Q_INVOKABLE void setLabelPath(const QString &newLabelPath);

    Q_INVOKABLE QString deployPlatform() const;
    Q_INVOKABLE void setDeployPlatform(const QString &newDeployPlatform);


    Q_INVOKABLE int checkBoxState() const;
    Q_INVOKABLE void setCheckBoxState(int newCheckBoxState);


    void modelRun(QString AIAlgorithm,
               QString m_modelPath,
               QString m_classPath,
               QString m_labelPath,
               QString m_deployPlatform,
               int m_checkBoxState);

    void modelManagerTest();

    Model model;


signals:
    void AIAlgorithmChanged();

    void modelPathChanged();

    void classPathChanged();

    void labelPathChanged();

    void deployPlatformChanged();

    void displaySettingsChanged();

    void checkBoxStateChanged();



private:
    QString m_AIAlgorithm;
    QString m_modelPath;
    QString m_classPath;
    QString m_labelPath;

    QString m_deployPlatform;
    int m_checkBoxState;


    Q_PROPERTY(QString AIAlgorithm READ AIAlgorithm WRITE setAIAlgorithm NOTIFY AIAlgorithmChanged FINAL)
    Q_PROPERTY(QString modelPath READ modelPath WRITE setModelPath NOTIFY modelPathChanged FINAL)
    Q_PROPERTY(QString classPath READ classPath WRITE setClassPath NOTIFY classPathChanged FINAL)
    Q_PROPERTY(QString labelPath READ labelPath WRITE setLabelPath NOTIFY labelPathChanged FINAL)
    Q_PROPERTY(QString deployPlatform READ deployPlatform WRITE setDeployPlatform NOTIFY deployPlatformChanged FINAL)
    Q_PROPERTY(int checkBoxState READ checkBoxState WRITE setCheckBoxState NOTIFY checkBoxStateChanged FINAL)
};

#endif // MODELMANAGER_H
