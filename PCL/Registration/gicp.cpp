//
// Created by Admin on 2024/11/18.
//
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // 创建源点云和目标点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取源点云和目标点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("source.pcd", *cloud_source) == -1)
    {
        PCL_ERROR("Couldn't read source file\n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("target.pcd", *cloud_target) == -1)
    {
        PCL_ERROR("Couldn't read target file\n");
        return (-1);
    }

    // --------------------------------GICP配准---------------------------------
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;				// gicp对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);	// 建树
    tree1->setInputCloud(source);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud(target);
    gicp.setSearchMethodSource(tree1);
    gicp.setSearchMethodTarget(tree2);
    gicp.setInputSource(cloud_source);						// 源点云
    gicp.setInputTarget(cloud_target);						// 目标点云
    gicp.setMaxCorrespondenceDistance(1.5);				// 设置对应点对之间的最大距离
    gicp.setTransformationEpsilon(1e-10);				// 为终止条件设置最小转换差异
    gicp.setEuclideanFitnessEpsilon(0.01);				// 设置收敛条件是均方误差和小于阈值， 停止迭代
    gicp.setMaximumIterations(50);						// 最大迭代次数
    gicp.setUseReciprocalCorrespondences(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*result);

    std::cout << "点对匹配分数：" << gicp.getFitnessScore() << std::endl;
    std::cout << "刚体变换矩阵：" << gicp.getFinalTransformation() << std::endl;

    // 可视化
    pcl::visualization::PCLVisualizer viewer("GICP Registration");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(cloud_output, 0, 0, 255);

    viewer.addPointCloud(cloud_source, source_color, "source_cloud");
    viewer.addPointCloud(cloud_target, target_color, "target_cloud");
    viewer.addPointCloud(cloud_output, output_color, "output_cloud");

    viewer.spin();

    return 0;
}
