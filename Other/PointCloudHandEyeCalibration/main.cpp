#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include <ceres/ceres.h>
using namespace pcl;
using namespace Eigen;


// 验证 X 是否满足 AX = XB
void verifyHandEyeCalibration(const std::vector<Matrix4f> &A_list, const std::vector<Matrix4f> &B_list, const Matrix4f &X) {
    float total_error = 0.0f;

    for (size_t i = 0; i < A_list.size(); ++i) {
        // 计算 AX 和 XB
        Matrix4f AX = A_list[i] * X;
        Matrix4f XB = X * B_list[i];

        // 计算误差矩阵
        Matrix4f error = AX - XB;

        // 使用 Frobenius 范数衡量误差
        float frobenius_norm = error.norm();
        total_error += frobenius_norm;

        std::cout << "第 " << i + 1 << " 对变换的误差 (Frobenius 范数): " << frobenius_norm << std::endl;
    }

    std::cout << "所有变换对的总误差: " << total_error << std::endl;
}
void verifyHandEyeCalibration(const std::vector<Matrix4d> &A_list, const std::vector<Matrix4d> &B_list, const Matrix4d& X) {
    float total_error = 0.0f;

    for (size_t i = 0; i < A_list.size(); ++i) {
        // 计算 AX 和 XB
        Matrix4d AX = A_list[i] * X;
        Matrix4d XB = X * B_list[i];

        // 计算误差矩阵
        Matrix4d error = AX - XB;

        // 使用 Frobenius 范数衡量误差
        float frobenius_norm = error.norm();
        total_error += frobenius_norm;

        std::cout << "第 " << i + 1 << " 对变换的误差 (Frobenius 范数): " << frobenius_norm << std::endl;
    }

    std::cout << "所有变换对的总误差: " << total_error << std::endl;
}
void validateHandEyeCalibration(const Eigen::Matrix4d& X,
                                const std::vector<Eigen::Matrix4d>& A,
                                const std::vector<Eigen::Matrix4d>& B) {
    double totalError = 0.0;

    for (size_t i = 0; i < A.size(); ++i) {
        Eigen::Matrix4d left = A[i] * X;
        Eigen::Matrix4d right = X * B[i];
        Eigen::Matrix4d diff = left - right;

        // 计算 Frobenius 范数
        double error = diff.norm();
        totalError += error;

        std::cout << "Pair " << i + 1 << " Error: " << error << std::endl;
    }

    std::cout << "Average Error: " << totalError / A.size() << std::endl;
}

// 随机生成变换矩阵（用于模拟测试数据）
Matrix4f generateRandomTransform() {
    Matrix4f T = Matrix4f::Identity();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    // 随机旋转矩阵（保证正交性）
    Vector3f axis = Vector3f(dist(gen), dist(gen), dist(gen)).normalized();
    float angle = dist(gen) * M_PI;
    AngleAxisf rotation(angle, axis);
    T.block<3, 3>(0, 0) = rotation.matrix();

    // 随机平移向量
    T(0, 3) = dist(gen) * 2.0f; // 平移范围 [-2, 2]
    T(1, 3) = dist(gen) * 2.0f;
    T(2, 3) = dist(gen) * 2.0f;

    return T;
}


// 手眼标定的代价函数，基于AX = XB模型
struct HandEyeCost {
    Eigen::Matrix4d A; // 手臂末端位姿
    Eigen::Matrix4d B; // 摄像头位姿

    HandEyeCost(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b) : A(a), B(b) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t, T* residuals) const {
        // 将四元数转换为旋转矩阵
        Eigen::Quaternion<T> quat(q[0], q[1], q[2], q[3]);
        Eigen::Matrix<T, 3, 3> R = quat.toRotationMatrix();

        // 构造手眼变换矩阵 X
        Eigen::Matrix<T, 4, 4> X = Eigen::Matrix<T, 4, 4>::Identity();
        X.template block<3, 3>(0, 0) = R;                     // 旋转部分
        X.template block<3, 1>(0, 3) = Eigen::Matrix<T, 3, 1>(t[0], t[1], t[2]); // 平移部分

        // 计算 AX - XB
        Eigen::Matrix<T, 4, 4> AX = A.cast<T>() * X;
        Eigen::Matrix<T, 4, 4> XB = X * B.cast<T>();
        Eigen::Matrix<T, 4, 4> diff = AX - XB;

        // 将残差定义为矩阵元素的差值
        for (int i = 0; i < 12; ++i) {
            residuals[i] = diff(i / 4, i % 4); // 仅计算前三行（3x4矩阵的残差）
        }

        return true;
    }

    // 创建代价函数的静态方法
    static ceres::CostFunction* Create(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B) {
        return new ceres::AutoDiffCostFunction<HandEyeCost, 12, 4, 3>(
                new HandEyeCost(A, B));
    }
};

// 使用Ceres优化器进行手眼标定
Eigen::Matrix4d optimizeHandEye(const std::vector<Eigen::Matrix4d>& A,
                                const std::vector<Eigen::Matrix4d>& B) {
    // 初始化四元数和平移向量
    double q[4] = {1.0, 0.0, 0.0, 0.0}; // 初始四元数（单位四元数，表示无旋转）
    double t[3] = {0.0, 0.0, 0.0};     // 初始平移向量

    // 构造Ceres优化问题
    ceres::Problem problem;
    for (size_t i = 0; i < A.size(); ++i) {
        // 为每组 A 和 B 添加一个残差块
        ceres::CostFunction* cost_function = HandEyeCost::Create(A[i], B[i]);
        problem.AddResidualBlock(cost_function, nullptr, q, t);
    }

    // 设置四元数的归一化约束
    problem.SetParameterization(q, new ceres::QuaternionParameterization());

    // 设置优化器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;          // 使用稠密QR分解
    options.minimizer_progress_to_stdout = true;           // 输出优化进度
    options.max_num_iterations = 100;                     // 最大迭代次数
    options.function_tolerance = 1e-10;                   // 函数容忍误差，控制优化精度

    // 运行优化
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 打印优化结果
    std::cout << summary.FullReport() << std::endl;

    // 将优化结果转换为4x4的变换矩阵
    Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    X.block<3, 3>(0, 0) = quat.toRotationMatrix(); // 设置旋转部分
    X.block<3, 1>(0, 3) = Eigen::Vector3d(t[0], t[1], t[2]); // 设置平移部分

    return X; // 返回标定结果
}


int main() {
    std::vector<Matrix4f> A_list;
    std::vector<Matrix4f> B_list;

    // 生成多组测试数据
    int num_pairs = 50;
    Matrix4f X_true = generateRandomTransform(); // 真实的手眼标定变换矩阵
    std::cout << "真实的手眼标定变换矩阵 X_true:\n" << X_true << std::endl;

    for (int i = 0; i < num_pairs; ++i) {
        Matrix4f A = generateRandomTransform(); // 随机生成 A
        Matrix4f B = X_true.inverse() * A * X_true; // 根据 A 和 X_true 生成对应的 B
        A_list.push_back(A);
        B_list.push_back(B);
    }


    std::cout << "------------------------------------------" << std::endl;

    std::vector<Eigen::Matrix4d> A_list_double;
    for (const auto& mat : A_list) {
        A_list_double.push_back(mat.cast<double>());
    }
    std::vector<Eigen::Matrix4d> B_list_double;
    for (const auto& mat : B_list) {
        B_list_double.push_back(mat.cast<double>());
    }

    // 调用优化函数计算手眼变换
    Eigen::Matrix4d X = optimizeHandEye(A_list_double, B_list_double);
//    validateHandEyeCalibration(X, A_list_double, B_list_double);
    verifyHandEyeCalibration(A_list_double, B_list_double, X);
    // 输出最终的手眼标定结果
    std::cout << "Hand-Eye Calibration Result (X1):\n" << X << std::endl;
    return 0;
}


