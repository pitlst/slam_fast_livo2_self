#--------------------------------------------------------------------------------------#
#   项目总体设置
#--------------------------------------------------------------------------------------#
cmake_minimum_required(VERSION 3.12)
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)  # 允许使用旧的 FindCUDA
endif()
# 设置项目名称
set(PROJECT_NAME slam_fast_livo2_self)
# 在这里直接指定项目为c++项目，节约查找tool chain的时间
project(${PROJECT_NAME})
# 设置c++标准等级和编译后输出路径
set(CMAKE_CXX_STANDARD 23)
# 导出执行命令到json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 强制化c++标准，当标准不符合要求时强制终止编译而不是采用更旧的标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# 设置编译优化的等级
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

#--------------------------------------------------------------------------------------#
#   项目依赖设置
#--------------------------------------------------------------------------------------#
# 找到海康动态库，保存到变量里
file(GLOB HIK_LIBS "/opt/MVS/lib/64/*.so")
# 找到海康头文件，保存到变量里
file(GLOB HIK_INCLUDE "/opt/MVS/include/*.h")
include_directories(/opt/MVS/include)
# 找到opencv
set(OpenCV_DIR /media/cheakf/新加卷/workspace/opencv/build)
find_package(OpenCV REQUIRED)
# 找到fmt
find_package(fmt REQUIRED)

# 设置头文件查找路径
include_directories(
    /opt/MVS/include
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/libs/toml++
    ${PROJECT_SOURCE_DIR}/libs/parallel_hashmap
)

#--------------------------------------------------------------------------------------#
#   项目编译设置
#--------------------------------------------------------------------------------------#
# 把项目目录定义成宏传递给程序
add_compile_definitions(PROJECT_PATH="${PROJECT_SOURCE_DIR}/")
add_executable(
    ${PROJECT_NAME}
    src/main.cpp
    src/general.hpp
    src/sensor/hk_camera.hpp
    src/sensor/hk_camera.cpp
    src/sensor/mid360_lidar.hpp
    src/sensor/mid360_lidar.cpp
    src/config/config.hpp
    src/config/config.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE pthread ${OpenCV_LIBS})
