cmake_minimum_required(VERSION 3.28)
project(OpenCV)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)


#Imgproc
add_executable(FindingContours FindingContours.cpp)
target_link_libraries(FindingContours ${OpenCV_LIBS})
