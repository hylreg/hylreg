cmake_minimum_required(VERSION 3.10)
project(hand_eye_calibration)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(hand_eye_calibration hand_eye_calibration.cpp)
target_link_libraries(hand_eye_calibration
        ${PCL_LIBRARIES}
        ceres
        Eigen3::Eigen
)