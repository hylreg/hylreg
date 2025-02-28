
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
    std::cout << "所有变换对的平均误差: " << total_error / A_list.size() << std::endl;

}
void validateHandEyeCalibration(const Eigen::Matrix4d& X,const std::vector<Eigen::Matrix4d>& A,const std::vector<Eigen::Matrix4d>& B) {
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
    T(0, 3) = dist(gen) * 200.0f; // 平移范围 [-2, 2]
    T(1, 3) = dist(gen) * 200.0f;
    T(2, 3) = dist(gen) * 200.0f;

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
//    problem.SetParameterization(q, new ceres::QuaternionParameterization());
    problem.SetManifold(q, new ceres::QuaternionManifold());


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

// 用于生成加噪声的矩阵
Matrix4f addNoiseToMatrix(const Matrix4f& mat, float noise_level) {
    // 生成均值为0，标准差为 noise_level 的高斯噪声
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> disR(-noise_level*0.01, noise_level*0.01);
    std::normal_distribution<float> disT(-noise_level, noise_level);

    Matrix4f noisy_matrix = mat;

    // // 给每个元素加噪声
    // for (int i = 0; i < 4; ++i) {
    //     for (int j = 0; j < 4; ++j) {
    //         noisy_matrix(i, j) += dis(gen);
    //     }
    // }

    // 给每个元素加噪声
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            noisy_matrix(i, j) += disR(gen);
        }
    }

    // 只对平移部分（即最后一列的前三个元素）加噪声
    for (int i = 0; i < 3; ++i) {
        noisy_matrix(i, 3) += disT(gen); // 只对平移向量的元素加噪声
    }

    return noisy_matrix;
}

// -------------------------------------
// 手眼标定求解函数
Eigen::Matrix4d solveHandEye(const std::vector<Eigen::Matrix4d>& A_list,
                             const std::vector<Eigen::Matrix4d>& B_list) {
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();

    // 求解旋转部分
    for (size_t i = 0; i < A_list.size(); ++i) {
        Eigen::Matrix3d Ra = A_list[i].block<3, 3>(0, 0);
        Eigen::Matrix3d Rb = B_list[i].block<3, 3>(0, 0);
        M += Ra - Rb.transpose() * Ra * Rb;
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

    // 修正旋转矩阵符号以确保正交性
    if (R.determinant() < 0) {
        R = -R;
    }

    // 求解平移部分
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < A_list.size(); ++i) {
        t += A_list[i].block<3, 3>(0, 0) * B_list[i].block<3, 1>(0, 3) -
             R * B_list[i].block<3, 3>(0, 0) * B_list[i].block<3, 1>(0, 3);
    }
    t /= A_list.size();

    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    X.block<3, 3>(0, 0) = R;
    X.block<3, 1>(0, 3) = t;
    return X;
}



int main() {
    std::vector<Matrix4f> A_list;
    std::vector<Matrix4f> B_list;


    // 生成多组测试数据
    int num_pairs = 15;
    float noise_level = 0.01f; // 设定噪声的标准差
    Matrix4f X_true = generateRandomTransform(); // 真实的手眼标定变换矩阵
    std::cout << "真实的手眼标定变换矩阵 X_true:\n" << X_true << std::endl;

    for (int i = 0; i < num_pairs; ++i) {
        Matrix4f A = generateRandomTransform(); // 随机生成 A
        Matrix4f B = X_true.inverse() * A * X_true; // 根据 A 和 X_true 生成对应的 B

        // 给 A 和 B 加入噪声
        A = addNoiseToMatrix(A, noise_level);
        B = addNoiseToMatrix(B, noise_level);

        A_list.push_back(A);
        B_list.push_back(B);
    }



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
    std::cout << "真实的手眼标定变换矩阵 X_true:\n" << X_true << std::endl;

    // 输出最终的手眼标定结果
    std::cout << "ceres 手眼标定结果 X:\n" << X << std::endl;

    Eigen::Vector4d P_camera;
    Eigen::Vector4d P_robot;

    // 机器人坐标系中的点 (x, y, z, 1)
    P_robot<<0, 0, 0, 1;
    // 将机器人坐标系的点转换到相机坐标系
    P_camera = X.inverse() * P_robot;
    std::cout << "机器人坐标中的原点在相机坐标中的位置: " << P_camera.transpose() << std::endl;


    // 相机坐标系中的点 (x, y, z, 1)
    P_camera<<0, 0, 0, 1; // 示例点
    // 将相机坐标系的点转换到机器人坐标系
    P_robot = X * P_camera;
    std::cout << "相机坐标系中的原点在机器人坐标中的位置: " << P_robot.transpose() << std::endl;

//    // 线性求解
//    std::cout << "------------------------------------------" << std::endl;
//    // 使用手眼标定算法求解
//    Eigen::Matrix4d X_estimated = solveHandEye(A_list_double, B_list_double);
//    // 输出结果
//    std::cout << "真实手眼标定矩阵 X_true:\n" << X_true << "\n" << std::endl;
//    std::cout << "估计手眼标定矩阵 (X_estimated):\n" << X_estimated << "\n" << std::endl;
//    verifyHandEyeCalibration(A_list_double, B_list_double, X_estimated);
//
//    // 机器人坐标系中的点 (x, y, z, 1)
//    P_robot<<0, 0, 0, 1;
//    // 将机器人坐标系的点转换到相机坐标系
//    P_camera = X_estimated.inverse() * P_robot;
//    std::cout << "机器人坐标中的原点在相机坐标中的位置: " << P_camera.transpose() << std::endl;
//
//    // 相机坐标系中的点 (x, y, z, 1)
//    P_camera<<0, 0, 0, 1; // 示例点
//    // 将相机坐标系的点转换到机器人坐标系
//    P_robot = X_estimated * P_camera;
//    std::cout << "相机坐标系中的原点在机器人坐标中的位置: " << P_robot.transpose() << std::endl;
//    std::cout << "------------------------------------------" << std::endl;

    return 0;
}