//
// Created by Admin on 2025/2/28.
//
#include <iostream>
#include <pcl/common/transforms.h>
#include <ceres/ceres.h>
#include <Eigen/Geometry>

// 定义位姿数据结构
struct PosePair {
    Eigen::Matrix4d robot_pose;  // 机器人末端位姿 (4x4)
    Eigen::Matrix4d target_pose; // 标定板在相机坐标系中的位姿 (4x4)
};

// Ceres误差函数类
class HandEyeCostFunctor {
public:
    HandEyeCostFunctor(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B)
            : A_(A), B_(B) {}

    template <typename T>
    bool operator()(const T* const x_rotation, const T* const x_translation, T* residual) const {
        // 将优化变量转换为变换矩阵
        Eigen::Quaternion<T> q(x_rotation[0], x_rotation[1], x_rotation[2], x_rotation[3]);
        Eigen::Matrix<T, 3, 1> t(x_translation[0], x_translation[1], x_translation[2]);

        Eigen::Transform<T, 3, Eigen::Affine> X;
        X.linear() = q.toRotationMatrix();
        X.translation() = t;

        // 计算 AX - XB 的残差
        Eigen::Transform<T, 3, Eigen::Affine> A_transform(A_.cast<T>());
        Eigen::Transform<T, 3, Eigen::Affine> B_transform(B_.cast<T>());

        Eigen::Transform<T, 3, Eigen::Affine> AX = A_transform * X;
        Eigen::Transform<T, 3, Eigen::Affine> XB = X * B_transform;

        Eigen::Matrix<T, 4, 4> error = AX.matrix() - XB.matrix();

        // 将残差平铺为数组
        for (int i = 0; i < 16; ++i) {
            residual[i] = error(i / 4, i % 4);
        }
        return true;
    }

private:
    const Eigen::Matrix4d A_;
    const Eigen::Matrix4d B_;
};

// 主函数
int main() {
    // 1. 生成或加载标定数据（示例数据）
    std::vector<PosePair> calibration_data;
    for (int i = 0; i < 50; ++i) {
        PosePair pair;
        // 生成模拟机器人末端位姿和标定板位姿
        Eigen::AngleAxisd rot_vec(0.1*i, Eigen::Vector3d::UnitZ());
        pair.robot_pose.setIdentity();
        pair.robot_pose.block<3,3>(0,0) = rot_vec.toRotationMatrix();
        pair.robot_pose(0,3) = 0.1 * i;

        pair.target_pose.setIdentity(); // 实际应从点云或图像检测标定板位姿
        calibration_data.push_back(pair);
    }

    // 2. 初始化待优化变量（X: 相机到末端的变换）
    Eigen::Quaterniond x_rot(1, 0, 0, 0); // 初始化为单位四元数
    Eigen::Vector3d x_trans(0, 0, 0);     // 初始化为零平移

    // 3. 构建Ceres优化问题
    ceres::Problem problem;
    for (const auto& pair : calibration_data) {
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<HandEyeCostFunctor, 16, 4, 3>(
                        new HandEyeCostFunctor(pair.robot_pose, pair.target_pose));

        problem.AddResidualBlock(
                cost_function,
                nullptr, // 不使用鲁棒核函数
                x_rot.coeffs().data(),
                x_trans.data());
    }

    // 4. 配置四元数参数化
    problem.SetParameterization(x_rot.coeffs().data(),
                                new ceres::EigenQuaternionParameterization());

    // 5. 运行优化
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 6. 输出结果
    std::cout << summary.BriefReport() << "\n";
    Eigen::Matrix3d R = x_rot.toRotationMatrix();
    std::cout << "Optimized X (Camera to End-Effector):\n"
              << "Rotation:\n" << R << "\n"
              << "Translation:\n" << x_trans.transpose() << "\n";

    return 0;
}