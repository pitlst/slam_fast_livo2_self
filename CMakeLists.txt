#--------------------------------------------------------------------------------------#
#   项目总体设置
#--------------------------------------------------------------------------------------#
cmake_minimum_required(VERSION 3.12)
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

#--------------------------------------------------------------------------------------#
#   项目依赖设置
#--------------------------------------------------------------------------------------#
# 找到海康动态库，保存到变量里
file(GLOB HIK_LIBS "/opt/MVS/lib/64/*.so")
# 找到海康头文件，保存到变量里
file(GLOB HIK_INCLUDE "/opt/MVS/include/*.h")
include_directories(/opt/MVS/include)
set(LIVOX_SDK ${PROJECT_SOURCE_DIR}/../Livox-SDK2)
set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/../opencv/build)
find_package(OpenCV REQUIRED)
set(rerun_sdk_DIR ${PROJECT_SOURCE_DIR}/../rerun_sdk_install/lib/cmake/rerun_sdk)
find_package(rerun_sdk REQUIRED)
set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/../cppzmq/build)
find_package(cppzmq REQUIRED)
find_package(zstd REQUIRED CONFIG)
find_package(fmt REQUIRED)
find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common filters)

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
    src/common/so3_math.hpp
    src/time_sync/time_sync.hpp
    src/time_sync/time_sync.cpp
    src/sensor/hk_camera.hpp
    src/sensor/hk_camera.cpp
    src/sensor/mid360_lidar.hpp
    src/sensor/mid360_lidar.cpp
    src/sensor/webot_camera.hpp
    src/sensor/webot_camera.cpp
    src/sensor/webot_lidar.hpp
    src/sensor/webot_lidar.cpp
    src/core/livo_mapper.hpp
    src/core/livo_mapper.cpp
    src/core/point_preprocess.hpp
    src/core/point_preprocess.cpp
    src/core/imu_process.hpp
    src/core/imu_process.cpp
    src/core/vio_process.hpp
    src/core/vio_process.cpp
    src/core/voxel_mapper.hpp
    src/core/voxel_mapper.cpp
)
target_include_directories(
    ${PROJECT_NAME} PRIVATE
    /opt/MVS/include
    ${OpenCV_INCLUDE_DIRS}
    ${LIVOX_SDK}/include
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/libs/toml++
    ${PROJECT_SOURCE_DIR}/libs/nlohmann
    ${PROJECT_SOURCE_DIR}/libs/readerwriterqueue
    ${PROJECT_SOURCE_DIR}/libs/parallel_hashmap
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
    pcl_filters
    cppzmq
    ${LIVOX_SDK}/build/sdk_core/liblivox_lidar_sdk_shared.so
    ${OpenCV_LIBS}
    ${HIK_LIBS}
)