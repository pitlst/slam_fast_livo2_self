#ifndef HSM_WEBOTLIDAR_H
#define HSM_WEBOTLIDAR_H

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include "zmq.hpp"
#include "Eigen/Core"

#include "common/common.hpp"
#include "common/struct.hpp"

namespace hsm
{
    struct webot_lidar
    {
        friend std::shared_ptr<webot_lidar> make_webot_lidar(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url, Eigen::Matrix3f const& imu_rotation);

    public:
        ~webot_lidar();

        bool get_points(timestamped<point_data>& out);
        bool get_imu(timestamped<imu_data>&);

        std::shared_ptr<zmq::context_t> conetxt;
        std::unique_ptr<zmq::socket_t>  socket;
        std::jthread                    back_thread;
        std::atomic<bool>               running_label = false;
    };

    // 工厂函数
    std::shared_ptr<webot_lidar> make_webot_lidar(
        std::shared_ptr<zmq::context_t> conetxt,
        std::string const&              connect_url  = "tcp://localhost:5555",
        Eigen::Matrix3f const&          imu_rotation = Eigen::Matrix3f::Identity());
} // namespace hsm

#endif
