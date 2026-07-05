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

#pragma pack(push, 1)
struct CustomPoint {
    float  x, y, z;           // 12 bytes
    uint8_t reflectivity;     // 1 byte
    uint8_t tag;              // 1 byte
    uint8_t line;             // 1 byte
    uint8_t padding;          // 1 byte
};
#pragma pack(pop)

static_assert(sizeof(CustomPoint) == 16, "CustomPoint must be 16 bytes");

struct LidarHeader {
    uint64_t timebase;   // microseconds
    uint32_t point_num;
    uint8_t  lidar_id;
    uint8_t  rsvd[3];
};
static_assert(sizeof(LidarHeader) == 16, "LidarHeader must be 16 bytes");

namespace hsm
{
    struct webot_lidar
    {
        friend std::shared_ptr<webot_lidar> make_webot_lidar(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url, Eigen::Matrix3f const& imu_rotation);

    public:
        ~webot_lidar();

        bool get_points(timestamped<point_data>& out);
        bool get_imu(timestamped<imu_data>& out);

        std::jthread      back_thread;
        std::atomic<bool> running_label = false;
    };

    // 工厂函数
    std::shared_ptr<webot_lidar> make_webot_lidar(
        std::shared_ptr<zmq::context_t> conetxt,
        std::string const&              connect_url  = "tcp://localhost:5555",
        Eigen::Matrix3f const&          imu_rotation = Eigen::Matrix3f::Identity());

    // 基于 span 的小端序解包
    template<typename T>
    inline T unpack_le(std::span<uint8_t const> data, size_t offset = 0)
    {
        static_assert(std::is_arithmetic_v<T>, "T 必须是算术类型");
        throw_if(data.size() < offset + sizeof(T), "unpack_le: span too small");
        T val;
        std::memcpy(&val, data.data() + offset, sizeof(T));
        return val;
    }
} // namespace hsm

#endif
