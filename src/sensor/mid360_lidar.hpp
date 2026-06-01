#ifndef HSM_MID360LIDAR_H
#define HSM_MID360LIDAR_H

#include <cstring>
#include <bit>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include "common/common.hpp"
#include "common/struct.hpp"
#include "common/enhanced_exception.hpp"

namespace hsm
{
    struct livox_lidar
    {
        friend std::unique_ptr<livox_lidar> make_livox_lidar(const std::filesystem::path& input_path);

    public:
        ~livox_lidar();

        timestamped<point_data> get_points();
        timestamped<imu_data>   get_imu();
    };

    std::unique_ptr<livox_lidar> make_livox_lidar(const std::filesystem::path& input_path = std::filesystem::path(PROJECT_PATH) / "config" / "mid360.json");
} // namespace hsm

#endif
