//
// Created by Admin on 2024/6/27.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>

int main() {
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/table_scene_lms400_plane_0.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file 'your_point_cloud.pcd'\n");
        return -1;
    }

    // 创建法线估计对象
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 创建一个KdTree对象用于法线估计
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // 设置法线估计的搜索半径
    ne.setRadiusSearch(0.05); // 单位：米

    // 计算法线
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    // 可视化点云和法线
    pcl::visualization::PCLVisualizer viewer("Point Cloud with Normals");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "point cloud");

    // 可视化法线
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 10, 0.1, "normals");

    // 循环显示，直到窗口关闭
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}
