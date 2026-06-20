#ifndef HSM_STRUCT_H
#define HSM_STRUCT_H

#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "readerwriterqueue.h"
#include "livox_lidar_def.h"

#include "common/so3_math.hpp"

namespace hsm
{
    // 携带有时间戳的传感器数据
    template<typename T>
    struct timestamped
    {
        uint64_t device_timestamp;
        uint64_t host_timestamp;
        T        value;
    };

    // 携带有处理后的全局时间点的传感器数据
    template<typename T>
    struct process_timestamped
    {
        double timestamp;
        T      value;
    };

    // 所有传感器缓冲区的大小限制
    inline constexpr size_t K_BUFFER_CAPACITY = 64;

    // 激光雷达的单行数据
    struct raw_point
    {
        int32_t x;
        int32_t y;
        int32_t z;
        uint8_t reflectivity;
        uint8_t tag;
    };

    // 激光雷达的整体点云数据
    struct point_data
    {
        std::vector<raw_point> points;
        uint16_t               time_interval;
        uint16_t               dot_num;
    };

    // imu数据
    struct imu_data
    {
        // 角速度
        float gyro_x;
        float gyro_y;
        float gyro_z;
        // 加速度
        float acc_x;
        float acc_y;
        float acc_z;
    };

    // 缓冲区的别名
    using point_queue = moodycamel::ReaderWriterQueue<timestamped<point_data>>;
    using imu_queue   = moodycamel::ReaderWriterQueue<timestamped<imu_data>>;
    using frame_queue = moodycamel::ReaderWriterQueue<timestamped<cv::Mat>>;

    // 拟合时间偏移后的imu数据
    struct raw_imu_data
    {
        Eigen::Vector3d gyro  = Eigen::Vector3d::Zero(); // 角速度 (rad/s)
        Eigen::Vector3d accel = Eigen::Vector3d::Zero(); // 加速度 (m/s^2)
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

    // 每一轮迭代需要的传感器数据
    struct measure_group
    {
        double                   vio_time = 0.0;
        double                   lio_time = 0.0;
        std::deque<raw_imu_data> imu;
        cv::Mat                  img;
    };

    // 算法类的处理状态结构
    struct lidar_measure_group
    {
        double                    lidar_frame_beg_time;
        double                    lidar_frame_end_time;
        double                    last_lio_update_time;
        std::deque<measure_group> measures;
        EKF_STATE                 lio_vio_flg;
        // 当前待处理的点云
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> pcl_proc_cur;
        // 下一轮待处理的点云
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> pcl_proc_next;

        lidar_measure_group()
            : lidar_frame_beg_time(-0.0),
              lidar_frame_end_time(0.0),
              last_lio_update_time(-0.0),
              lio_vio_flg(EKF_STATE::WAIT),
              pcl_proc_cur(std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>()),
              pcl_proc_next(std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>())
        {
            this->measures.clear();
        };
    };

    struct states_group
    {
        // 类内默认初始化（C++11）。协方差用 IIFE 初始化，避免构造函数体重复逻辑
        Eigen::Matrix3d rot_end       = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos_end       = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel_end       = Eigen::Vector3d::Zero();
        Eigen::Vector3d bias_g        = Eigen::Vector3d::Zero();
        Eigen::Vector3d bias_a        = Eigen::Vector3d::Zero();
        Eigen::Vector3d gravity       = Eigen::Vector3d::Zero();
        double          inv_expo_time = 1.0;

        Eigen::Matrix<double, 19, 19> cov = []
        {
            Eigen::Matrix<double, 19, 19> c = Eigen::Matrix<double, 19, 19>::Identity() * 0.01;
            c(6, 6)                         = 0.00001;
            c.block<9, 9>(10, 10)           = Eigen::Matrix<double, 9, 9>::Identity() * 0.00001;
            return c;
        }();

        // 基于 += 实现 +，消除重复代码；并标记 const 与 nodiscard
        [[nodiscard]] states_group operator+(const Eigen::Matrix<double, 19, 1>& state_add) const
        {
            states_group result = *this;
            result += state_add;
            return result;
        }

        states_group& operator+=(const Eigen::Matrix<double, 19, 1>& state_add)
        {
            rot_end = rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
            pos_end += state_add.block<3, 1>(3, 0);
            inv_expo_time += state_add(6, 0);
            vel_end += state_add.block<3, 1>(7, 0);
            bias_g += state_add.block<3, 1>(10, 0);
            bias_a += state_add.block<3, 1>(13, 0);
            gravity += state_add.block<3, 1>(16, 0);
            return *this;
        }

        [[nodiscard]] Eigen::Matrix<double, 19, 1> operator-(const states_group& b) const
        {
            Eigen::Matrix<double, 19, 1> a;
            const Eigen::Matrix3d        rotd = b.rot_end.transpose() * this->rot_end;

            a.block<3, 1>(0, 0)  = Log(rotd);
            a.block<3, 1>(3, 0)  = this->pos_end - b.pos_end;
            a(6, 0)              = this->inv_expo_time - b.inv_expo_time;
            a.block<3, 1>(7, 0)  = this->vel_end - b.vel_end;
            a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
            a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
            a.block<3, 1>(16, 0) = this->gravity - b.gravity;
            return a;
        }

        void resetpose() noexcept
        {
            rot_end = Eigen::Matrix3d::Identity();
            pos_end = Eigen::Vector3d::Zero();
            vel_end = Eigen::Vector3d::Zero();
        }
    };

} // namespace hsm

#endif