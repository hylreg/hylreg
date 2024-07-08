//
// Created by Admin on 2024/7/8.
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main ()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile ("../data/bun0.pcd", *cloud);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    // pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);


    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (*mls_points);

    // Save output
    pcl::io::savePCDFile ("../data/bun0-mls.pcd", *mls_points);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::PointNormal>(cloud, mls_points);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}
