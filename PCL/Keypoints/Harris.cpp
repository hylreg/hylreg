#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/table_scene_lms400.pcd", *cloud) == -1) {
        pcl::console::print_error("Couldn't read file %s!\n");
        return (-1);
    }

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    float voxel_size = 0.01;
    sor.setLeafSize (voxel_size,voxel_size,voxel_size);
    sor.setInputCloud (cloud);
    sor.filter (*cloud);


    //-------------------------- Harris 关键点提取 --------------------------------
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    pcl::PointCloud<pcl::PointXYZI>::Ptr harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>);

    harris.setInputCloud(cloud);
    harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::LOWE);

    harris.setRadius(0.05);
    // harris.setRadiusSearch(0.06);
    harris.setNonMaxSupression(true);
    harris.setThreshold(1e-4);
    harris.setRefine(true);
    harris.setNumberOfThreads(10);
    harris.compute(*harris_keypoints);

    //-------------------------- 可视化 --------------------------------------------
    // 创建 PCL 可视化对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);  // 设置背景颜色为黑色
    // 可视化原始点云
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "original_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "original_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");


    // 可视化 Harris 关键点
    viewer->addPointCloud<pcl::PointXYZI>(harris_keypoints, "harris_keypoints");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "harris_keypoints");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "harris_keypoints");

    // // 设置相机参数和视角
    // viewer->initCameraParameters();
    // viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    //-------------------------- 等待窗口关闭 --------------------------------------
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
