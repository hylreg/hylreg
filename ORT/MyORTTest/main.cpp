//
// Created by Admin on 2024/11/12.
//
#include "onnxruntime_cxx_api.h"

int main(){

    // 创建 ONNX Runtime 环境
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
    // 创建 SessionOptions
    Ort::SessionOptions session_options;

    // 加载 ONNX 模型
    const std::string model_path = "../data/mnist.onnx";
    Ort::Session session = std::make_shared<Ort::Session>(env, model_path, session_options);

    return 0;
}