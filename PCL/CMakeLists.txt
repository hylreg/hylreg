cmake_minimum_required(VERSION 3.28)
project(PCL)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)

#读取PCD文件
add_executable(PCDRead IO/pcd_read.cpp)
target_link_libraries(PCDRead ${PCL_LIBRARIES})

#写入PCD文件
add_executable(PCDWrite IO/pcd_write.cpp)
target_link_libraries(PCDWrite ${PCL_LIBRARIES})

#使用直通过滤器过滤点云
add_executable(PassThroughFilter Filtering/pass_through_filter.cpp)
target_link_libraries(PassThroughFilter ${PCL_LIBRARIES})

#使用体素网格滤波器过滤点云
add_executable(VoxelGridFilter Filtering/voxel_grid_filter.cpp)
target_link_libraries(VoxelGridFilter ${PCL_LIBRARIES})

#使用统计滤波器过滤点云
add_executable(StatisticalOutlierRemoval Filtering/statistical_outlier_removal.cpp)
target_link_libraries(StatisticalOutlierRemoval ${PCL_LIBRARIES})

#使用条件滤波器过滤点云
add_executable(ConditionFilter Filtering/condition_filter.cpp)
target_link_libraries(ConditionFilter ${PCL_LIBRARIES})

#使用参数化模型投影点
add_executable(ProjectInliers Filtering/project_inliers.cpp)
target_link_libraries(ProjectInliers ${PCL_LIBRARIES})

#从点云中提取索引
add_executable(ExtractIndices Filtering/extract_indices.cpp)
target_link_libraries(ExtractIndices ${PCL_LIBRARIES})

