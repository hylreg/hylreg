#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include <QObject>

class ModelManager : public QObject
{
    Q_OBJECT
public:
    explicit ModelManager(QObject *parent = nullptr);

    Q_INVOKABLE QString modelPath() const;
    Q_INVOKABLE void setModelPath(const QString &newModelPath);

    Q_INVOKABLE QString classPath() const;
    Q_INVOKABLE void setClassPath(const QString &newClassPath);

    Q_INVOKABLE QString labelPath() const;
    Q_INVOKABLE void setLabelPath(const QString &newLabelPath);

signals:

    void modelPathChanged();

    void classPathChanged();

    void labelPathChanged();

private:
    QString m_modelPath;
    QString m_classPath;
    QString m_labelPath;


    Q_PROPERTY(QString modelPath READ modelPath WRITE setModelPath NOTIFY modelPathChanged FINAL)
    Q_PROPERTY(QString classPath READ classPath WRITE setClassPath NOTIFY classPathChanged FINAL)
    Q_PROPERTY(QString labelPath READ labelPath WRITE setLabelPath NOTIFY labelPathChanged FINAL)
};

#endif // MODELMANAGER_H
