#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
}

MainWindow::~MainWindow() {
    delete ui;
}


void MainWindow::on_pushButton_start_clicked() {
    qDebug() << "start thread";

    // 线程池初始化，设置最大线程池数
    QThreadPool::globalInstance()->setMaxThreadCount(4);
    // 添加任务
    QThreadPool::globalInstance()->start(task);

}


void MainWindow::on_pushButton_stop_clicked() {
    // 检查 task 是否已经被创建并启动
    if (task) {
        task->stop();  // 调用 stop 方法来停止任务
        qDebug() << "close thread";
    }
}

