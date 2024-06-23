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
