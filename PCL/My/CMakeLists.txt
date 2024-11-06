cmake_minimum_required(VERSION 3.28)
project(My)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)

add_executable(FaceToFace PointCloudMeasurements/FaceToFace.cpp)
target_link_libraries(FaceToFace ${PCL_LIBRARIES})
























