//
// Created by hylre on 24-7-12.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>

int main(int, char **argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/table_scene_lms400.pcd", *cloud) == -1) {
        PCL_ERROR("加载点云失败\n");
    }


    pcl::VoxelGrid<pcl::PointXYZ> sor;
    float voxel_size = 0.01;
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.setInputCloud(cloud);
    sor.filter(*cloud);

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    iss.setInputCloud(cloud);
    iss.setSearchMethod(tree);
    iss.setNumberOfThreads(10); // 初始化调度器并设置要使用的线程数
    iss.setSalientRadius(0.05); // 设置用于计算协方差矩阵的球邻域半径
    iss.setNonMaxRadius(0.05); // 设置非极大值抑制应用算法的半径
    iss.setThreshold21(0.95); // 设定第二个和第一个特征值之比的上限
    iss.setThreshold32(0.95); // 设定第三个和第二个特征值之比的上限
    iss.setMinNeighbors(6); // 在应用非极大值抑制算法时，设置必须找到的最小邻居数
    iss.compute(*keypoints);


    std::cout << "original cloud points size = " << cloud->points.size() << std::endl; // 112099
    std::cout << "ISS_3D points size = " << keypoints->points.size() << std::endl; // 4760
    // 关键点显示
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D ISS"));
    viewer1->setBackgroundColor(255, 255, 255);
    viewer1->setWindowName("ISS");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0.0, 255, 0.0);
    viewer1->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
    viewer1->addPointCloud<pcl::PointXYZ>(keypoints, "key cloud");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "key cloud");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "key cloud");
    while (!viewer1->wasStopped()) {
        viewer1->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }

    return 0;
}
