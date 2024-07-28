#include <torch/torch.h>
#include <iostream>

int main() {
    // 检查CUDA是否可用
    if (!torch::cuda::is_available()) {
        std::cerr << "CUDA is not available. Exiting..." << std::endl;
        return -1;
    }

    // 设置默认的CUDA设备
    torch::Device device(torch::kCUDA);
    std::cout << "CUDA Device Count: " << torch::cuda::device_count() << std::endl;

    // 创建一个随机张量并将其移动到CUDA设备上
    auto tensor = torch::rand({3, 3}).to(device);
    std::cout << "Tensor on CUDA:\n" << tensor << std::endl;

    return 0;
}
