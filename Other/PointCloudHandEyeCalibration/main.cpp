
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>

// 定义齐次变换矩阵类型
#define M_PI 3.14159265358979323846
using Transform = Eigen::Matrix4d;

// 工具函数：生成随机齐次变换矩阵
Transform generateRandomTransform() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> angle_dist(-M_PI, M_PI);
    std::uniform_real_distribution<> translation_dist(-1.0, 1.0);

    // 随机旋转
    double angle = angle_dist(gen);
    Eigen::Matrix3d R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // 随机平移
    Eigen::Vector3d t(translation_dist(gen), translation_dist(gen), translation_dist(gen));

    // 构造齐次变换矩阵
    Transform T = Transform::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

// 工具函数：添加噪声
Transform addNoiseToTransform(const Transform& T, double noise_level) {
    Transform noisy_T = T;

    // 添加旋转噪声
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::AngleAxisd angleAxis(noise_level * Eigen::Vector3d::Random().normalized());
    R = angleAxis.toRotationMatrix() * R;

    // 添加平移噪声
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    t += noise_level * Eigen::Vector3d::Random();

    // 构造齐次变换矩阵
    noisy_T.block<3, 3>(0, 0) = R;
    noisy_T.block<3, 1>(0, 3) = t;
    return noisy_T;
}

// 定义 Ceres 误差项
struct HandEyeError {
    HandEyeError(const Transform& A, const Transform& B) : A_(A), B_(B) {}

    template <typename T>
    bool operator()(const T* const X_raw, T* residuals) const {
        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor>> X(X_raw);

        // 计算误差：A * X - X * B
        Eigen::Matrix<T, 4, 4> error = A_.cast<T>() * X - X * B_.cast<T>();

        // 展平误差矩阵为一维数组
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                residuals[i * 4 + j] = error(i, j);
            }
        }
        return true;
    }

    const Transform A_, B_;
};

// 优化求解 AX = XB
Transform solveHandEyeAXEqualsXB(const std::vector<Transform>& A, const std::vector<Transform>& B) {
    ceres::Problem problem;
    Transform X = Transform::Identity(); // 初始猜测
    double* X_data = X.data();

    for (size_t i = 0; i < A.size(); ++i) {
        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<HandEyeError, 16, 16>(
                        new HandEyeError(A[i], B[i])),
                nullptr, X_data);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;
    return X;
}

// 剔除异常值
void filterOutliers(std::vector<Transform>& A, std::vector<Transform>& B, double threshold) {
    std::vector<Transform> filteredA, filteredB;
    for (size_t i = 0; i < A.size(); ++i) {
        double translationError = (A[i].block<3, 1>(0, 3) - B[i].block<3, 1>(0, 3)).norm();
        if (translationError < threshold) {
            filteredA.push_back(A[i]);
            filteredB.push_back(B[i]);
        }
    }
    A = filteredA;
    B = filteredB;
}

int main() {
    // 生成示例数据
    Transform X_gt = generateRandomTransform(); // Ground truth X
    std::vector<Transform> A, B;

    size_t num_samples = 20;
    double noise_level = 0.05;

    for (size_t i = 0; i < num_samples; ++i) {
        Transform A_i = generateRandomTransform();
        Transform B_i = X_gt.inverse() * A_i * X_gt; // 生成 B_i = X^-1 * A_i * X
        A.push_back(addNoiseToTransform(A_i, noise_level));
        B.push_back(addNoiseToTransform(B_i, noise_level));
    }

    // 滤波：剔除异常值
    filterOutliers(A, B, 0.5);

    // 求解 AX = XB
    Transform X = solveHandEyeAXEqualsXB(A, B);

    // 输出结果
    std::cout << "Ground truth X:\n" << X_gt << "\n\n";
    std::cout << "Estimated X:\n" << X << "\n\n";

    // 验证误差
    double error = (X_gt - X).norm();
    std::cout << "Estimation Error: " << error << std::endl;

    return 0;
}
