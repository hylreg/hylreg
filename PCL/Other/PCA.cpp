//
// Created by Admin on 2024/7/10.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 从文件中加载点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/table_scene_lms400.pcd", *cloud) == -1) // 更换为你的PCD文件路径
    {
        PCL_ERROR("Couldn't read the file\n");
        return -1;
    }

    // 创建PCA对象并设置输入点云
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);

    // 获取点云的均值
    Eigen::Vector4f mean = pca.getMean();
    std::cout << "Mean: " << mean.transpose() << std::endl;

    // 获取主成分（特征向量）
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    std::cout << "Eigenvectors:\n" << eigenvectors << std::endl;

    // 获取特征值
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    std::cout << "Eigenvalues:\n" << eigenvalues << std::endl;

    // // 使用PCA进行点云变换
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pca.project(*cloud, *transformed_cloud);

    // 使用PCA进行点云变换，将其降维到2D
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud->points)
    {
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f pt_transformed = eigenvectors.transpose() * pt;
        transformed_cloud->push_back(pcl::PointXYZ(pt_transformed[0], pt_transformed[1], 0)); // 只保留前两个主成分
    }

    // 可视化原始点云和变换后的点云
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCA Example"));
    viewer->setBackgroundColor(0, 0, 0);

    // 添加原始点云（红色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> original_color(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, original_color, "original cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original cloud");

    // 添加变换后的点云（绿色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_color(transformed_cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, transformed_color, "transformed cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}
