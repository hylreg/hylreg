cmake_minimum_required(VERSION 3.28)
project(PCL)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)

##读取PCD文件
#add_executable(PCDRead IO/pcd_read.cpp)
#target_link_libraries(PCDRead ${PCL_LIBRARIES})
#
##写入PCD文件
#add_executable(PCDWrite IO/pcd_write.cpp)
#target_link_libraries(PCDWrite ${PCL_LIBRARIES})
#
##连接两个点云的点
#add_executable(ConcatenatePointClouds IO/concatenate_point_clouds.cpp)
#target_link_libraries(ConcatenatePointClouds ${PCL_LIBRARIES})
#
#
##使用直通过滤器过滤点云
#add_executable(PassThroughFilter Filtering/pass_through_filter.cpp)
#target_link_libraries(PassThroughFilter ${PCL_LIBRARIES})
#
##使用体素网格滤波器过滤点云
#add_executable(VoxelGridFilter Filtering/voxel_grid_filter.cpp)
#target_link_libraries(VoxelGridFilter ${PCL_LIBRARIES})
#
##使用统计滤波器过滤点云
#add_executable(StatisticalOutlierRemoval Filtering/statistical_outlier_removal.cpp)
#target_link_libraries(StatisticalOutlierRemoval ${PCL_LIBRARIES})
#
##使用条件滤波器过滤点云
#add_executable(ConditionFilter Filtering/condition_filter.cpp)
#target_link_libraries(ConditionFilter ${PCL_LIBRARIES})
#
##使用参数化模型投影点
#add_executable(ProjectInliers Filtering/project_inliers.cpp)
#target_link_libraries(ProjectInliers ${PCL_LIBRARIES})
#
##从点云中提取索引
#add_executable(ExtractIndices Filtering/extract_indices.cpp)
#target_link_libraries(ExtractIndices ${PCL_LIBRARIES})
#
##估计点云中的表面法线
#add_executable(EstimateNormals Features/estimate_normals.cpp)
#target_link_libraries(EstimateNormals ${PCL_LIBRARIES})
#
###使用积分图像的正态估计
##add_executable(NormalEstimationUsingIntegralImages Features/normal_estimation_using_integral_images.cpp)
##target_link_libraries(NormalEstimationUsingIntegralImages ${PCL_LIBRARIES})
#
##icp
#add_executable(ICP Registration/icp.cpp)
#target_link_libraries(ICP ${PCL_LIBRARIES})
#
##KdTree
#add_executable(KdTree KdTree/kd_tree.cpp)
#target_link_libraries(KdTree ${PCL_LIBRARIES})
#
##Interactive Iterative Closest Point
#add_executable(InteractiveICP Registration/interactive_icp.cpp)
#target_link_libraries(InteractiveICP ${PCL_LIBRARIES})
#
##Normal Distributions Transform
#add_executable(NDT Registration/ndt.cpp)
#target_link_libraries(NDT ${PCL_LIBRARIES})
#
#add_executable(AlignmentPrerejective Registration/alignment_prerejective.cpp)
#target_link_libraries(AlignmentPrerejective ${PCL_LIBRARIES})
#
##point_cloud_compression
#add_executable(PointCloudCompression Octree/point_cloud_compression.cpp)
#target_link_libraries(PointCloudCompression ${PCL_LIBRARIES})
#
##narf_keypoint_extraction
#add_executable(NARFKeypointExtraction Keypoints/narf_keypoint_extraction.cpp)
#target_link_libraries(NARFKeypointExtraction ${PCL_LIBRARIES})
#
##use Random Sample Consensus model
#add_executable(RandomSampleConsensus SampleConsensus/random_sample_consensus.cpp)
#target_link_libraries(RandomSampleConsensus ${PCL_LIBRARIES})
#
##基于多项式重构的平滑和正态估计
#add_executable(Resampling Surface/resampling.cpp)
#target_link_libraries(Resampling ${PCL_LIBRARIES})
#
##基于多项式重构的平滑和正态估计
#add_executable (ConcaveHull2D Surface/concave_hull_2d.cpp)
#target_link_libraries(ConcaveHull2D ${PCL_LIBRARIES})

#PCA
add_executable(PCA Other/pca.cpp)
target_link_libraries(PCA ${PCL_LIBRARIES})

add_executable(SIFT Keypoints/SIFT.cpp)
target_link_libraries(SIFT ${PCL_LIBRARIES})

add_executable(Harris Keypoints/Harris.cpp)
target_link_libraries(Harris ${PCL_LIBRARIES})
