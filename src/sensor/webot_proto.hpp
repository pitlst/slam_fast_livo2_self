#ifndef HSM_WEBOT_PROTO_H
#define HSM_WEBOT_PROTO_H

#include <cstdint>
#include <vector>
#include <string>
#include <string_view>
#include <span>
#include <stdexcept>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "json.hpp"
#include "common/struct.hpp"
#include "common/enhanced_exception.hpp"

namespace hsm::webot_proto
{
    [[nodiscard]] inline constexpr uint16_t read_be16(const uint8_t* p) noexcept
    {
        return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
    }

    [[nodiscard]] inline constexpr uint32_t read_be32(const uint8_t* p) noexcept
    {
        return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
    }

    [[nodiscard]] inline constexpr uint64_t read_be64(const uint8_t* p) noexcept
    {
        return (uint64_t(p[0]) << 56) | (uint64_t(p[1]) << 48) | (uint64_t(p[2]) << 40) | (uint64_t(p[3]) << 32) | (uint64_t(p[4]) << 24) | (uint64_t(p[5]) << 16) | (uint64_t(p[6]) << 8) | uint64_t(p[7]);
    }

    [[nodiscard]] inline float read_be_float(const uint8_t* p) noexcept
    {
        uint32_t bits = read_be32(p);
        float    result;
        std::memcpy(&result, &bits, sizeof(result));
        return result;
    }

    struct CameraFrame
    {
        uint64_t             timestamp_us;
        uint16_t             width;
        uint16_t             height;
        std::vector<uint8_t> image; // BGRA, size = width * height * 4
    };

    struct LidarFrame
    {
        uint64_t           timestamp_us;
        uint16_t           num_layers;
        uint16_t           horiz_res;
        std::vector<float> ranges; // size = num_layers * horiz_res
    };

    struct ImuFrame
    {
        uint64_t timestamp_us;
        float    accel[3]; // m/s²
        float    gyro[3];  // rad/s
    };

    struct SensorInfo
    {
        double lidar_fov  = 3.0;  // 水平视场角 [rad]
        double lidar_vfov = 1.03; // 垂直视场角 [rad]
    };

    [[nodiscard]] inline CameraFrame parse_camera(std::span<const uint8_t> payload)
    {
        throw_if(payload.size() < 12, "Camera payload too short");

        uint64_t ts       = read_be64(payload.data());
        uint16_t w        = read_be16(payload.data() + 8);
        uint16_t h        = read_be16(payload.data() + 10);
        size_t   expected = static_cast<size_t>(w) * h * 4;

        throw_if(payload.size() != 12 + expected, "Camera payload size mismatch");

        CameraFrame frame;
        frame.timestamp_us = ts;
        frame.width        = w;
        frame.height       = h;
        frame.image.assign(payload.begin() + 12, payload.end());
        return frame;
    }

    [[nodiscard]] inline LidarFrame parse_lidar(std::span<const uint8_t> payload)
    {
        throw_if(payload.size() < 12, "Lidar payload too short");

        uint64_t ts       = read_be64(payload.data());
        uint16_t layers   = read_be16(payload.data() + 8);
        uint16_t hres     = read_be16(payload.data() + 10);
        size_t   n_pts    = static_cast<size_t>(layers) * hres;
        size_t   expected = 12 + n_pts * 4;

        throw_if(payload.size() != expected, "Lidar payload size mismatch");

        LidarFrame frame;
        frame.timestamp_us = ts;
        frame.num_layers   = layers;
        frame.horiz_res    = hres;
        frame.ranges.resize(n_pts);
        for (size_t i = 0; i < n_pts; ++i)
        {
            frame.ranges[i] = read_be_float(payload.data() + 12 + i * 4);
        }
        return frame;
    }

    [[nodiscard]] inline ImuFrame parse_imu(std::span<const uint8_t> payload)
    {
        throw_if(payload.size() != 32, "IMU payload size mismatch (expected 32 bytes)");

        ImuFrame frame;
        frame.timestamp_us = read_be64(payload.data());
        frame.accel[0]     = read_be_float(payload.data() + 8);
        frame.accel[1]     = read_be_float(payload.data() + 12);
        frame.accel[2]     = read_be_float(payload.data() + 16);
        frame.gyro[0]      = read_be_float(payload.data() + 20);
        frame.gyro[1]      = read_be_float(payload.data() + 24);
        frame.gyro[2]      = read_be_float(payload.data() + 28);
        return frame;
    }

    [[nodiscard]] inline SensorInfo parse_metadata(std::string_view json)
    {
        SensorInfo info;
        try
        {
            auto j = nlohmann::json::parse(json);
            if (j.contains("lidar"))
            {
                auto& lidar = j["lidar"];
                if (lidar.contains("fov"))
                {
                    info.lidar_fov = lidar["fov"].get<double>();
                }
                if (lidar.contains("vertical_fov"))
                {
                    info.lidar_vfov = lidar["vertical_fov"].get<double>();
                }
            }
        }
        catch (...)
        {
            // 解析失败时使用默认值
        }
        return info;
    }

    // ============================================================================
    //  距离阵 → 原始点云 (球坐标 → 笛卡尔 → int32 mm)
    // ============================================================================

    [[nodiscard]] inline std::vector<raw_point> lidar_ranges_to_raw_points(
        const LidarFrame& lidar, double fov, double vfov)
    {
        std::vector<raw_point> points;
        points.reserve(lidar.ranges.size());

        for (uint16_t j = 0; j < lidar.num_layers; ++j)
        {
            double phi     = -vfov / 2.0 + (static_cast<double>(j) + 0.5) * vfov / lidar.num_layers;
            double cos_phi = std::cos(phi);
            double sin_phi = std::sin(phi);

            for (uint16_t i = 0; i < lidar.horiz_res; ++i)
            {
                double theta = -fov / 2.0 + (static_cast<double>(i) + 0.5) * fov / lidar.horiz_res;
                double r     = lidar.ranges[static_cast<size_t>(j) * lidar.horiz_res + i];

                if (! std::isfinite(r) || r <= 0.0)
                {
                    continue;
                }

                // LiDAR 坐标系: X 前, Y 左, Z 上
                double x = r * cos_phi * std::sin(theta);
                double y = r * cos_phi * std::cos(theta);
                double z = r * sin_phi;

                raw_point pt;
                pt.x            = static_cast<int32_t>(std::round(x * 1000.0));
                pt.y            = static_cast<int32_t>(std::round(y * 1000.0));
                pt.z            = static_cast<int32_t>(std::round(z * 1000.0));
                pt.reflectivity = 128;  // 默认强度
                pt.tag          = 0x10; // 有效点标志: (tag & 0x30) == 0x10
                points.push_back(pt);
            }
        }
        return points;
    }
} // namespace hsm::webot_proto

#endif // HSM_WEBOT_PROTO_H
