#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include "class1.h"


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void lineEditTextChanged(const QString &text);  // Signal to send text to Class1


private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    Class1 *class1;
};
#endif // MAINWINDOW_H
