#ifndef HSM_CORE_LIVMAPPER_H
#define HSM_CORE_LIVMAPPER_H

#include <deque>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "common/struct.hpp"
#include "sensor/hk_camera.hpp"
#include "sensor/mid360_lidar.hpp"
#include "time_sync/time_sync.hpp"
#include "core/imu_process.hpp"
#include "core/point_preprocess.hpp"
#include "core/vio_process.hpp"
namespace hsm
{
    struct livo_mapper
    {
    public:
        // 相机和激光雷达的初始化外置
        explicit livo_mapper(std::unique_ptr<hk_camera> camera, std::unique_ptr<livox_lidar> lidar);

        // 循环入口
        void run();

        // 同步驱动数据
        bool sync_packages();
        // 从 SPSC 队列拉数据到内部 deque 缓冲区
        void drain_sensor_queues();

    public:
        // 缓冲队列
        std::deque<process_timestamped<imu_data>>                              lidar_imu_buffer;
        std::deque<process_timestamped<pcl::PointCloud<pcl::PointXYZINormal>>> lidar_point_buffer;
        std::deque<process_timestamped<cv::Mat>>                               frame_buffer;

        bool lidar_pushed = false;

        // 核心状态

        std::shared_ptr<lidar_measure_group> lidar_measures;
        std::shared_ptr<states_group>        states;

        // 相关的处理类

        std::unique_ptr<imu_process>      p_imu;
        std::unique_ptr<point_preprocess> p_point;
        std::unique_ptr<vio_process> p_vio;

        std::unique_ptr<time_sync> image_sync;
        std::unique_ptr<time_sync> point_sync;
        std::unique_ptr<time_sync> imu_sync;

        std::unique_ptr<hk_camera>   camera;
        std::unique_ptr<livox_lidar> lidar;
    };
} // namespace hsm

#endif