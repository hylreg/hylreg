#ifndef CLASS1_H
#define CLASS1_H

#include <QObject>
#include <iostream>
#include "class2.h"


class Class1 : public QObject
{
    Q_OBJECT
public:
    explicit Class1(QObject *parent = nullptr);

    void Class1Func();

signals:

public slots:
    void onLineEditTextChanged(const QString &text);  // Slot to receive text from MainWindow

};

#endif // CLASS1_H
