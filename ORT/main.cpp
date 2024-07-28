#include <onnxruntime_cxx_api.h>
#include <iostream>

int main() {
    // 创建 ONNX Runtime 环境
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");

    // 创建 SessionOptions 并启用 CUDA
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // 配置 CUDA Provider 选项
    // OrtCUDAProviderOptions cuda_options;
    // cuda_options.device_id = 0; // 使用第一个 GPU 设备
    // cuda_options.gpu_mem_limit = std::numeric_limits<size_t>::max(); // 不限制 GPU 内存
    // cuda_options.arena_extend_strategy = 0;
    // // cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearch::EXHAUSTIVE;
    // cuda_options.do_copy_in_default_stream = 1;

    // 使用 Ort::SessionOptions 的 AppendExecutionProvider_CUDA 方法
    // session_options.AppendExecutionProvider_CUDA(cuda_options);


    // 加载 ONNX 模型
    const wchar_t* model_path = L"../data/mnist.onnx"; // 将路径改为宽字符字符串
    Ort::Session session(env, model_path, session_options);

    // 获取模型输入输出名称
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::AllocatedStringPtr input_name = session.GetInputNameAllocated(0, allocator);
    Ort::AllocatedStringPtr output_name = session.GetOutputNameAllocated(0, allocator);

    std::cout << "Input Name: " << input_name.get() << std::endl;
    std::cout << "Output Name: " << output_name.get() << std::endl;

    return 0;
}
