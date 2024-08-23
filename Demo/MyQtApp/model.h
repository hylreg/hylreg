#ifndef MODEL_H
#define MODEL_H

#include <QObject>
#include <qstring.h>
#include <QDebug>

class Model : public QObject
{
    Q_OBJECT
public:
    explicit Model(QObject *parent = nullptr);

    Q_PROPERTY(QString modelPath READ modelPath WRITE setModelPath NOTIFY modelPathChanged FINAL)


    QString modelPath() const { return m_modelPath; }
    void setModelPath(QString newValue) {
        if (m_modelPath != newValue) {
            m_modelPath = newValue;
            emit modelPathChanged();
        }
    }

    void onModelPathChanged();
signals:
    void modelPathChanged();

private:
    QString m_modelPath;
};

#endif // MODEL_H
