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
