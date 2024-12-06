#include "class2.h"

Class2::Class2(const QString& text,QObject *parent)
    : QObject{parent},m_text(text)
{
    Class2Func();
}

void Class2::Class2Func()
{
    QString str = m_text;
    std::cout<<"class2:"<<str.toStdString()<<std::endl;
}
