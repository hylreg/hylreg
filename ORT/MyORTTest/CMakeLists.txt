cmake_minimum_required(VERSION 3.28)
project(ORT)

set(CMAKE_CXX_STANDARD 17)

#find_package(onnxruntime REQUIRED)
#find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

set(ONNXRUNTIME_ROOT "D:/Libraries/onnxruntime-win-x64-gpu-1.16.3")
include_directories(ORT ${ONNXRUNTIME_ROOT}/include)

add_executable(ORT main.cpp)
target_link_libraries(ORT ${ONNXRUNTIME_ROOT}/lib/onnxruntime.lib )




#if (WIN32)
#    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#            COMMAND ${CMAKE_COMMAND} -E copy_if_different
#            "${ONNXRUNTIME_ROOT}/lib/onnxruntime.dll"
#            $<TARGET_FILE_DIR:${PROJECT_NAME}>)
#endif ()


