cmake_minimum_required(VERSION 3.28)
project(PointCloudHandEyeCalibration)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(PCL  REQUIRED)

#读取PCD文件
add_executable(HandEyeCalibration main.cpp
        main.cpp)
target_link_libraries(HandEyeCalibration ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${EIGEN3_LIBRARIES})






























