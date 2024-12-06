#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    class1 = new Class1(this);
    connect(this, &MainWindow::lineEditTextChanged, class1, &Class1::onLineEditTextChanged);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    QString str = ui->lineEdit->text();
    std::cout<<"Main:"<<str.toStdString()<<std::endl;

    emit lineEditTextChanged(str);
}

