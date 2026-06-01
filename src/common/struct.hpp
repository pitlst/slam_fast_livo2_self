#ifndef SLAM_STRUCT_H
#define SLAM_STRUCT_H

#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include "readerwriterqueue.h"
#include "livox_lidar_def.h"

namespace hsm
{
    // 携带有时间戳的传感器数据
    template<typename T>
    struct timestamped
    {
        uint64_t device_timestamp;
        uint64_t host_timestamp;
        T        payload;
    };

    // 所有传感器缓冲区的大小限制
    inline constexpr size_t K_BUFFER_CAPACITY = 64;

    // 驱动的数据别名
    using point_data = std::vector<LivoxLidarCartesianHighRawPoint>;
    using imu_data   = LivoxLidarImuRawPoint;

    // 缓冲区的别名
    using point_queue = moodycamel::ReaderWriterQueue<timestamped<point_data>>;
    using imu_queue   = moodycamel::ReaderWriterQueue<timestamped<imu_data>>;
    using frame_queue = moodycamel::ReaderWriterQueue<timestamped<cv::Mat>>;

    // 拟合时间偏移后的imu数据
    struct raw_imu_data
    {
        double          timestamp;
        Eigen::Vector3d gyro;  // 角速度 (rad/s)
        Eigen::Vector3d accel; // 加速度 (m/s^2)
    };

    // 拟合时间偏移后的激光雷达单线点云格式
    struct raw_point_data
    {
        double   x;
        double   y;
        double   z;
        double   intensity;
        double   timestamp;
        uint16_t ring;
    };

    // 拟合时间偏移之后的图像输出
    struct frame_data
    {
        cv::Mat image;
        double  timestamp;
    };

    // 位资输出
    struct pose_6d
    {
        double          timestamp;
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        Eigen::Vector3d velocity;
    };
} // namespace hsm

#endif