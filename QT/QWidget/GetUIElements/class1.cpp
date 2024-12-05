#include "class1.h"

Class1::Class1(QObject *parent)
    : QObject{parent}
{}

void Class1::Class1Func()
{
    QString str = "class1";
    std::cout<<str.toStdString()<<std::endl;
}

void Class1::onLineEditTextChanged(const QString &text)
{
    std::cout << "Class1:"<<text.toStdString() << std::endl;

    Class2 *class2 = new Class2(text);


}
