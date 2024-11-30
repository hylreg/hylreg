#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <pcl/pcl_macros.h>

// 随机生成正交旋转矩阵
Eigen::Matrix3d generateRandomRotation() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);

    double angle_x = dist(gen);
    double angle_y = dist(gen);
    double angle_z = dist(gen);

    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
            0, cos(angle_x), -sin(angle_x),
            0, sin(angle_x), cos(angle_x);

    Eigen::Matrix3d Ry;
    Ry << cos(angle_y), 0, sin(angle_y),
            0, 1, 0,
            -sin(angle_y), 0, cos(angle_y);

    Eigen::Matrix3d Rz;
    Rz << cos(angle_z), -sin(angle_z), 0,
            sin(angle_z), cos(angle_z), 0,
            0, 0, 1;

    Eigen::Matrix3d R = Rz * Ry * Rx;

    // 校正旋转矩阵以确保正交性
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();

    return R;
}

// 随机生成平移向量
Eigen::Vector3d generateRandomTranslation(double range = 0.1) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-range, range);

    return Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
}

// 随机生成变换矩阵
Eigen::Matrix4d generateRandomTransform(double translation_range = 0.1) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = generateRandomRotation();
    T.block<3, 1>(0, 3) = generateRandomTranslation(translation_range);
    return T;
}

// 模拟数据生成函数
void generateSimulationData(int num_samples, std::vector<Eigen::Matrix4d>& A_list,
                            std::vector<Eigen::Matrix4d>& B_list, Eigen::Matrix4d& X) {
    // 随机生成真实的手眼标定矩阵 X
    X = generateRandomTransform();

    for (int i = 0; i < num_samples; ++i) {
        // 随机生成机械臂变换矩阵 A_i
        Eigen::Matrix4d A = generateRandomTransform();
        A_list.push_back(A);

        // 根据 X 和 A_i 生成对应的点云变换矩阵 B_i
        Eigen::Matrix4d B = X.inverse() * A * X;
        B_list.push_back(B);
    }
}

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
    // 模拟生成数据
    std::vector<Eigen::Matrix4d> A_list;
    std::vector<Eigen::Matrix4d> B_list;
    Eigen::Matrix4d X_true;

    int num_samples = 20; // 生成 20 组样本数据
    generateSimulationData(num_samples, A_list, B_list, X_true);

    // 使用手眼标定算法求解
    Eigen::Matrix4d X_estimated = solveHandEye(A_list, B_list);

    // 输出结果
    std::cout << "真实手眼标定矩阵 (X_true):\n" << X_true << "\n" << std::endl;
    std::cout << "估计手眼标定矩阵 (X_estimated):\n" << X_estimated << "\n" << std::endl;

    // 计算误差
    double rotation_error = 0.0;
    double translation_error = 0.0;

    for (size_t i = 0; i < A_list.size(); ++i) {
        Eigen::Matrix3d Ra_est = A_list[i].block<3, 3>(0, 0) * X_estimated.block<3, 3>(0, 0);
        Eigen::Matrix3d Rb_est = X_estimated.block<3, 3>(0, 0) * B_list[i].block<3, 3>(0, 0);

        rotation_error += (Ra_est - Rb_est).norm(); // 旋转部分误差
        translation_error += (A_list[i].block<3, 1>(0, 3) -
                              X_estimated.block<3, 3>(0, 0) * B_list[i].block<3, 1>(0, 3) -
                              X_estimated.block<3, 1>(0, 3))
                .norm(); // 平移部分误差
    }

    rotation_error /= A_list.size();
    translation_error /= A_list.size();

    // 打印误差
    std::cout << "旋转部分平均误差: " << rotation_error << std::endl;
    std::cout << "平移部分平均误差: " << translation_error << std::endl;

    return 0;
}
