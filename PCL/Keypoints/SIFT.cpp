#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/cloud_viewer.h>

namespace pcl {
    struct PointXYZ;

    // 自定义 SIFT 关键点字段选择器
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ> {
        // 选择 Z 坐标作为特征
        inline float operator () (const PointXYZ& p) const {
            return p.z;
        }
    };
}

int main(int argc, char** argv) {
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/table_scene_lms400.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file.\n");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    float voxel_size = 0.01;
    sor.setLeafSize (voxel_size,voxel_size,voxel_size);
    sor.setInputCloud (cloud);
    sor.filter (*cloud_filtered);

    // SIFT 关键点检测
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;

    // 设置输入点云
    sift.setInputCloud(cloud_filtered);

    // 设置 SIFT 参数
    sift.setScales(0.001, 3, 5);  // 设置尺度空间
    sift.setMinimumContrast(0.001);  // 设置最小对比度
    // narf_keypoint_detector.getParameters ().support_size = support_size;
    // sift.setKSearch(3);
    sift.setRadiusSearch(0.05);

    // 执行 SIFT 关键点检测
    sift.compute(result);

    // 输出检测到的关键点数量
    std::cout << "Detected " << result.points.size() << " SIFT keypoints." << std::endl;

    // 创建 PCL 可视化窗口
    pcl::visualization::PCLVisualizer viewer("SIFT Keypoints");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    // 将输入点云添加到可视化窗口中（显示为白色）
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "input_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");

    // 将检测到的 SIFT 关键点添加到可视化窗口中（显示为红色）
    for (size_t i = 0; i < result.points.size(); ++i) {
        pcl::PointWithScale point = result.points[i];
        viewer.addSphere<pcl::PointWithScale>(point, point.scale*0.5, 1.0, 0.0, 0.0, "keypoint_" + std::to_string(i));
    }

    // 显示窗口，直到用户关闭
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}
