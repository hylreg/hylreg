#include "model.h"

Model::Model(QObject *parent)
    : QObject{parent}
{
    // 连接信号和槽
    connect(this, &Model::modelPathChanged, this, &Model::onModelPathChanged);
}

// 槽函数定义
void Model::onModelPathChanged() {
    // 执行你想做的事情，比如打印日志
    qDebug() << "Model path changed to:" << m_modelPath;
    // 你可以在这里添加更多的处理逻辑
}
