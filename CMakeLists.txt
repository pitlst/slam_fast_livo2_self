#--------------------------------------------------------------------------------------#
#   项目总体设置
#--------------------------------------------------------------------------------------#
cmake_minimum_required(VERSION 3.12)
# cmake_policy(SET CMP0144 NEW)
# if(POLICY CMP0146)
#     cmake_policy(SET CMP0146 OLD) # 允许使用旧的 FindCUDA
# endif()
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
# 找到livox sdk
set(LIVOX_SDK /mnt/data/Livox-SDK2)
# 找到opencv
set(OpenCV_DIR /mnt/data/opencv/build)
find_package(OpenCV REQUIRED)
# 找到rerun，用于点云可视化
set(rerun_sdk_DIR /mnt/data/rerun_sdk_install/lib/cmake/rerun_sdk)
find_package(rerun_sdk REQUIRED)
# 其他的库
find_package(fmt REQUIRED)
find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)

#--------------------------------------------------------------------------------------#
#   项目编译设置
#--------------------------------------------------------------------------------------#
# 把项目目录定义成宏传递给程序
add_compile_definitions(PROJECT_PATH="${PROJECT_SOURCE_DIR}/")
add_executable(
    ${PROJECT_NAME}
    src/main.cpp
    src/common/common.hpp
    src/common/enhanced_exception.hpp
    src/common/struct.hpp
    src/time_sync/time_sync.hpp
    src/time_sync/time_sync.cpp
    src/sensor/hk_camera.hpp
    src/sensor/hk_camera.cpp
    src/sensor/mid360_lidar.hpp
    src/sensor/mid360_lidar.cpp
    src/core/livo_mapper.hpp
    src/core/livo_mapper.cpp
    src/core/point_prerpocess.hpp
    src/core/point_prerpocess.cpp
    src/core/imu_process.hpp
    src/core/imu_process.cpp
    src/core/vio_process.hpp
    src/core/vio_process.cpp
)
target_include_directories(
    ${PROJECT_NAME} PRIVATE
    /opt/MVS/include
    ${OpenCV_INCLUDE_DIRS}
    ${LIVOX_SDK}/include
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/libs/toml++
    ${PROJECT_SOURCE_DIR}/libs/readerwriterqueue
)
target_link_libraries(
    ${PROJECT_NAME} PRIVATE
    pthread
    absl::strings
    absl::str_format
    absl::numeric
    absl::int128
    absl::base
    absl::utility
    fmt::fmt
    Eigen3::Eigen
    rerun_sdk
    pcl_common
    pcl_io
    pcl_filters
    pcl_visualization
    ${LIVOX_SDK}/build/sdk_core/liblivox_lidar_sdk_shared.so
    ${OpenCV_LIBS}
    ${HIK_LIBS})
