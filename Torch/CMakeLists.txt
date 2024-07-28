cmake_minimum_required(VERSION 3.28)
project(OpenCV)

set(CMAKE_CXX_STANDARD 17)

find_package(torch REQUIRED)

add_executable(Torch main.cpp)
target_link_libraries(Torch ${TORCH_LIBRARIES})


