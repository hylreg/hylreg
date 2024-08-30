// Worker.h
#ifndef WORKER_H
#define WORKER_H

#include <QRunnable>

class Worker : public QRunnable {
public:
    Worker();
    void run() override;
};

#endif // WORKER_H
