#ifndef MYWORK_H
#define MYWORK_H

#include <QObject>
#include <QRunnable>
#include <QThread>

class MyWork : public QObject, public QRunnable
{
    Q_OBJECT
public:
    explicit MyWork(QObject *parent = nullptr);

    void run() override;

    void stop();

private:
    bool stopMyWork = false;

signals:
};

#endif // MYWORK_H
