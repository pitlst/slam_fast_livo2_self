#ifndef SLAM_MID360LIDAR_H
#define SLAM_MID360LIDAR_H

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <vector>
#include <mutex>
#include <memory>
#include <filesystem>

#include "general.hpp"
#include "config/config.hpp"

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

namespace hsm
{
    struct livox_lidar
    {
        friend std::unique_ptr<livox_lidar> make_livox_lidar(const std::filesystem::path& input_path);

    public:
        livox_lidar() = default;
        ~livox_lidar();
        /// 获取最新一帧点云数据
        std::vector<LivoxLidarCartesianHighRawPoint> get_points();
        // 获取最新一帧imu
        LivoxLidarImuRawPoint get_imu();
    };

    std::unique_ptr<livox_lidar> make_livox_lidar(const std::filesystem::path& input_path = std::filesystem::path(PROJECT_PATH) / "config" / "mid360.json");
} // namespace hsm

// 点云获取回调
extern void _livox_point_callback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
// imu获取回调
extern void _livox_imu_callback(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
// 设备在线
extern void _livox_device_online_callback(uint32_t handle, const LivoxLidarInfo* info, void* client_data);

extern std::vector<LivoxLidarCartesianHighRawPoint> _livox_points;
extern std::mutex                                   _livox_points_mutex;
extern LivoxLidarImuRawPoint                        _livox_imu;
extern std::mutex                                   _livox_imu_mutex;

#endif