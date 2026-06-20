#ifndef HSM_WEBOTLIDAR_H
#define HSM_WEBOTLIDAR_H

#include <string>
#include <memory>
#include <thread>
#include <atomic>

#include "Eigen/Core"

#include "common/common.hpp"
#include "common/struct.hpp"
#include "common/enhanced_exception.hpp"

namespace hsm
{
    struct webot_lidar
    {
        friend std::shared_ptr<webot_lidar> make_webot_lidar(const std::string& host, size_t port, const Eigen::Matrix3f& imu_rotation);

    public:
        ~webot_lidar();

        bool get_points(timestamped<point_data>& out);
        bool get_imu(timestamped<imu_data>&);

        std::jthread      back_thread;
        std::atomic<bool> running_label = false;
    };

    // 工厂函数
    std::shared_ptr<webot_lidar> make_webot_lidar(const std::string& host = "127.0.0.1", size_t port = 12345, const Eigen::Matrix3f& imu_rotation = Eigen::Matrix3f::Identity());
} // namespace hsm

#endif
