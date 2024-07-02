cmake_minimum_required(VERSION 3.28)
project(OpenCV)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

add_executable(OpenCV main.cpp)
target_link_libraries(OpenCV ${OpenCV_LIBS})


