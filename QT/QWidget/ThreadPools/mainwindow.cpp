#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    QThreadPool::globalInstance()->setMaxThreadCount(4);

}

MainWindow::~MainWindow() {
    delete ui;
}


void MainWindow::on_pushButton_start_clicked() {
    qDebug() << "start thread";

    // if (task && task->isRunning()) {
    //     task->stop();  // 停止当前任务
    //     task = nullptr;  // 置空 task
    // }// 添加任务

    task = new MyWork;
    QThreadPool::globalInstance()->start(task);

}


void MainWindow::on_pushButton_stop_clicked() {
    // 检查 task 是否已经被创建并启动
    if (task) {
        task->stop();
        qDebug() << "close thread";

    }
}

