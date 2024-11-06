#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

int main ()
{


    //读取点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/face_to_face.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    //提取前三个平面
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::ModelCoefficients::Ptr>> cloud_plane;
    while ( cloud_plane.size() < 3 ) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.5);


        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            PCL_ERROR ("无法估计给定数据集的平面模型.\n");
            return (-1);
        }

        //提取平面点云
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_plane_ptr);
        cloud_plane.push_back(std::make_pair(cloud_plane_ptr,coefficients));

        std::cerr << "模型系数: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;

        std::cerr << "模型内部值: " << inliers->indices.size() << std::endl;

        extract.setNegative(true);
        extract.filter(*cloud);

    }

    std::sort(cloud_plane.begin(),cloud_plane.end(),[](std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::ModelCoefficients::Ptr> a,std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::ModelCoefficients::Ptr> b){
        return a.second->values[3] < b.second->values[3];
    });

    //计算面到面的距离
    pcl::PointXYZ min_point,max_point;
    pcl::getMinMax3D(*cloud_plane[0].first,min_point,max_point);
    double min_distance = std::sqrt(std::pow(min_point.x,2)+std::pow(min_point.y,2)+std::pow(min_point.z,2));
    double max_distance = std::sqrt(std::pow(max_point.x,2)+std::pow(max_point.y,2)+std::pow(max_point.z,2));
    for (int i = 1; i < cloud_plane.size(); i++) {
        pcl::PointXYZ min_point,max_point;
        pcl::getMinMax3D(*cloud_plane[i].first,min_point,max_point);
        double distance = std::sqrt(std::pow(min_point.x,2)+std::pow(min_point.y,2)+std::pow(min_point.z,2));
        if (distance < min_distance) {
            min_distance = distance;
        }
        if (distance > max_distance) {
            max_distance = distance;
        }
    }
    std::cout << "min_distance: " << min_distance << std::endl;
    std::cout << "max_distance: " << max_distance << std::endl;


    return (0);
}