#ifndef CLASS2_H
#define CLASS2_H

#include <QObject>
#include <iostream>


class Class2 : public QObject
{
    Q_OBJECT
public:
    explicit Class2(const QString& text);

    void Class2Func();


signals:

private:
    QString m_text;  // 存储传入的文本
};

#endif // CLASS2_H
