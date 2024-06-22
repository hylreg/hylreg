cmake_minimum_required(VERSION 3.28)
project(PCL)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)

add_executable(PCDRead IO/pcd_read.cpp)
target_link_libraries(PCDRead ${PCL_LIBRARIES})