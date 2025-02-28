cmake_minimum_required(VERSION 3.10)
project(MyDynamicLib)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)

# 生成动态库
add_library(MyDynamicLib SHARED
        src/mylib.cpp
        include/mylib.h
)

# 包含头文件目录
target_include_directories(MyDynamicLib
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 跨平台符号导出处理
if(WIN32)
    target_compile_definitions(MyDynamicLib PRIVATE MYLIB_EXPORTS)
    target_compile_definitions(MyDynamicLib INTERFACE MYLIB_IMPORTS)
endif()

\# 创建可执行文件
add_executable(MyApp main.cpp)

# 链接动态库
target_link_libraries(MyApp PRIVATE MyDynamicLib)
