cmake_minimum_required(VERSION 3.28)
project(Other)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 CONFIG REQUIRED)
find_package(PCL  REQUIRED)

#读取PCD文件
add_executable(Center Center.cpp)
target_link_libraries(Center ${PCL_LIBRARIES})






























