#ifndef SLAM_MID360LIDAR_H
#define SLAM_MID360LIDAR_H

#include <vector>
#include <memory>
#include <filesystem>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include "readerwriterqueue.h"

#include "general.hpp"
#include "config/config.hpp"

namespace hsm
{
    using point_data = std::vector<LivoxLidarCartesianHighRawPoint>;
    using imu_data   = LivoxLidarImuRawPoint;

    using point_queue = moodycamel::ReaderWriterQueue<timestamped<point_data>>;
    using imu_queue   = moodycamel::ReaderWriterQueue<timestamped<imu_data>>;

    struct livox_lidar
    {
        friend std::unique_ptr<livox_lidar> make_livox_lidar(const std::filesystem::path& input_path);

    public:
        ~livox_lidar();

        timestamped<point_data> get_points(uint64_t& timestamp);
        timestamped<imu_data>   get_imu(uint64_t& timestamp);
    };

    std::unique_ptr<livox_lidar> make_livox_lidar(const std::filesystem::path& input_path = std::filesystem::path(PROJECT_PATH) / "config" / "mid360.json");
} // namespace hsm

#endif
