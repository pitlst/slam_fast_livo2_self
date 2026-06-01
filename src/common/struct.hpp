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

    struct point_data
    {
        std::vector<LivoxLidarCartesianHighRawPoint> points;
        uint16_t                                     time_interval;
        uint16_t                                     dot_num;
    };

    // 驱动的数据别名
    using imu_data = LivoxLidarImuRawPoint;

    // 缓冲区的别名
    using point_queue = moodycamel::ReaderWriterQueue<timestamped<point_data>>;
    using imu_queue   = moodycamel::ReaderWriterQueue<timestamped<imu_data>>;
    using frame_queue = moodycamel::ReaderWriterQueue<timestamped<cv::Mat>>;

    // 拟合时间偏移后的imu数据
    struct raw_imu_data
    {
        double          timestamp = 0.0;
        Eigen::Vector3d gyro      = Eigen::Vector3d::Zero(); // 角速度 (rad/s)
        Eigen::Vector3d accel     = Eigen::Vector3d::Zero(); // 加速度 (m/s^2)
    };

    // 拟合时间偏移后的激光雷达单线点云格式
    struct raw_point_data
    {
        double   x         = 0.0;
        double   y         = 0.0;
        double   z         = 0.0;
        double   intensity = 0.0;
        double   timestamp = 0.0;
        uint16_t ring;
    };

    // 拟合时间偏移之后的图像输出
    struct frame_data
    {
        cv::Mat image;
        double  timestamp = 0.0;
    };

    // 位资输出
    struct pose_6d
    {
        double          timestamp   = 0.0;
        Eigen::Matrix3d rotation    = Eigen::Matrix3d::Zero();
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity    = Eigen::Vector3d::Zero();
    };

    // 拓展卡尔曼的状态
    enum EKF_STATE {
        WAIT = 0,
        VIO  = 1,
        LIO  = 2,
        LO   = 3
    };

    //
    struct measure_group
    {
        double                   vio_time = 0.0;
        double                   lio_time = 0.0;
        std::deque<raw_imu_data> imu;
        cv::Mat                  img;
    };

    struct lidar_measure_group
    {
        double                                                 lidar_frame_beg_time;
        double                                                 lidar_frame_end_time;
        double                                                 last_lio_update_time;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> lidar;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> pcl_proc_cur;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> pcl_proc_next;
        std::deque<measure_group>                              measures;
        EKF_STATE                                              lio_vio_flg;
        int                                                    lidar_scan_index_now;

        lidar_measure_group()
        {
            lidar_frame_beg_time = -0.0;
            lidar_frame_end_time = 0.0;
            last_lio_update_time = -1.0;
            lio_vio_flg          = EKF_STATE::WAIT;
            this->lidar          = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
            this->pcl_proc_cur   = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
            this->pcl_proc_next  = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
            this->measures.clear();
            lidar_scan_index_now = 0;
            last_lio_update_time = -1.0;
        };
    };

} // namespace hsm

#endif