#ifndef HSM_MID360LIDAR_H
#define HSM_MID360LIDAR_H

#include <cstring>
#include <bit>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include "Eigen/Core"

#include "common/common.hpp"
#include "common/struct.hpp"
#include "common/enhanced_exception.hpp"

namespace hsm
{
    struct livox_lidar
    {
        friend std::unique_ptr<livox_lidar> make_livox_lidar(const std::filesystem::path& input_path, const Eigen::Matrix3f& imu_rotation);

    public:
        ~livox_lidar();

        bool get_points(timestamped<point_data>& out);
        bool get_imu(timestamped<imu_data>& out);

    public:
        ;
    };

    std::unique_ptr<livox_lidar> make_livox_lidar(
        const std::filesystem::path& input_path   = std::filesystem::path(PROJECT_PATH) / "config" / "mid360.json",
        const Eigen::Matrix3f&       imu_rotation = Eigen::Matrix3f::Identity());
} // namespace hsm

#endif
