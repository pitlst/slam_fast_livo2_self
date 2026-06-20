#ifndef HSM_WEBOT_PROTO_H
#define HSM_WEBOT_PROTO_H

// ============================================================================
//  Webots ZeroMQ 协议共享定义 — webot_camera / webot_lidar 内部头文件
//
//  协议基于 wsl_client_zmq.cpp:
//    ZMQ PUB/SUB, 多部分消息 (topic + payload)
//    topic: "camera" / "lidar" / "info"
//
//    Camera payload: [timestamp_us:8B][width:2B][height:2B][BGRA: w*h*4B]
//    Lidar  payload: [timestamp_us:8B][num_layers:2B][horiz_res:2B][ranges: N*4B]
//    Info   payload: JSON 字符串 (含 lidar.fov / lidar.vertical_fov)
// ============================================================================

#include <cstdint>
#include <vector>
#include <string>
#include <string_view>
#include <span>
#include <optional>
#include <stdexcept>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace hsm::webot_proto
{
    // ============================================================================
    //  大端序读取 (与 Python struct.pack("!...") 一致)
    // ============================================================================

    [[nodiscard]] inline constexpr uint16_t read_be16(const uint8_t* p) noexcept
    {
        return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
    }

    [[nodiscard]] inline constexpr uint32_t read_be32(const uint8_t* p) noexcept
    {
        return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16)
             | (uint32_t(p[2]) <<  8) | uint32_t(p[3]);
    }

    [[nodiscard]] inline constexpr uint64_t read_be64(const uint8_t* p) noexcept
    {
        return (uint64_t(p[0]) << 56) | (uint64_t(p[1]) << 48)
             | (uint64_t(p[2]) << 40) | (uint64_t(p[3]) << 32)
             | (uint64_t(p[4]) << 24) | (uint64_t(p[5]) << 16)
             | (uint64_t(p[6]) <<  8) | uint64_t(p[7]);
    }

    [[nodiscard]] inline float read_be_float(const uint8_t* p) noexcept
    {
        uint32_t bits = read_be32(p);
        float result;
        std::memcpy(&result, &bits, sizeof(result));
        return result;
    }

    // ============================================================================
    //  传感器帧数据结构 (ZMQ 版本, timestamp_us 内嵌在 payload 中)
    // ============================================================================

    struct CameraFrame
    {
        uint64_t            timestamp_us;
        uint16_t            width;
        uint16_t            height;
        std::vector<uint8_t> image;   // BGRA, size = width * height * 4
    };

    struct LidarFrame
    {
        uint64_t           timestamp_us;
        uint16_t           num_layers;
        uint16_t           horiz_res;
        std::vector<float> ranges;    // size = num_layers * horiz_res
    };

    struct SensorInfo
    {
        double lidar_fov  = 3.0;     // 水平视场角 [rad]
        double lidar_vfov = 1.03;    // 垂直视场角 [rad]
    };

    // ============================================================================
    //  解析器 (ZMQ payload: 12 字节头部 + 数据)
    // ============================================================================

    [[nodiscard]] inline CameraFrame parse_camera(std::span<const uint8_t> payload)
    {
        if (payload.size() < 12)
            throw std::runtime_error("Camera payload too short");

        uint64_t ts = read_be64(payload.data());
        uint16_t w  = read_be16(payload.data() + 8);
        uint16_t h  = read_be16(payload.data() + 10);
        size_t expected = static_cast<size_t>(w) * h * 4;

        if (payload.size() != 12 + expected)
            throw std::runtime_error("Camera payload size mismatch");

        CameraFrame frame;
        frame.timestamp_us = ts;
        frame.width  = w;
        frame.height = h;
        frame.image.assign(payload.begin() + 12, payload.end());
        return frame;
    }

    [[nodiscard]] inline LidarFrame parse_lidar(std::span<const uint8_t> payload)
    {
        if (payload.size() < 12)
            throw std::runtime_error("Lidar payload too short");

        uint64_t ts    = read_be64(payload.data());
        uint16_t layers = read_be16(payload.data() + 8);
        uint16_t hres  = read_be16(payload.data() + 10);
        size_t   n_pts  = static_cast<size_t>(layers) * hres;
        size_t   expected = 12 + n_pts * 4;

        if (payload.size() != expected)
            throw std::runtime_error("Lidar payload size mismatch");

        LidarFrame frame;
        frame.timestamp_us = ts;
        frame.num_layers = layers;
        frame.horiz_res  = hres;
        frame.ranges.resize(n_pts);
        for (size_t i = 0; i < n_pts; ++i)
            frame.ranges[i] = read_be_float(payload.data() + 12 + i * 4);
        return frame;
    }

    // ============================================================================
    //  元信息 JSON 解析 (极简版)
    // ============================================================================

    [[nodiscard]] inline std::optional<double> json_extract_double(
        std::string_view json, std::string_view key)
    {
        auto key_start = json.find(key);
        if (key_start == std::string_view::npos) return std::nullopt;

        auto colon = json.find(':', key_start + key.size());
        if (colon == std::string_view::npos) return std::nullopt;

        auto val_start = json.find_first_not_of(" \t\n\r", colon + 1);
        if (val_start == std::string_view::npos) return std::nullopt;

        auto val_end = val_start;
        while (val_end < json.size() && json[val_end] != ','
               && json[val_end] != '}' && json[val_end] != ']'
               && json[val_end] != '\n' && json[val_end] != ' ')
            ++val_end;

        try {
            std::string num{json.substr(val_start, val_end - val_start)};
            return std::stod(num);
        } catch (...) {
            return std::nullopt;
        }
    }

    [[nodiscard]] inline SensorInfo parse_metadata(std::string_view json)
    {
        SensorInfo info;
        auto lidar_pos = json.find("\"lidar\"");
        if (lidar_pos == std::string_view::npos) return info;

        auto lidar_block = json.substr(lidar_pos);
        if (auto fov = json_extract_double(lidar_block, "\"fov\""))
            info.lidar_fov = *fov;
        if (auto vfov = json_extract_double(lidar_block, "\"vertical_fov\""))
            info.lidar_vfov = *vfov;

        return info;
    }

    // ============================================================================
    //  LiDAR 原始结构 (与 mid360_lidar 的 raw_point 一致)
    // ============================================================================

    struct RawPoint
    {
        int32_t x;              // mm
        int32_t y;              // mm
        int32_t z;              // mm
        uint8_t reflectivity;
        uint8_t tag;
    };

    // ============================================================================
    //  距离阵 → 原始点云 (球坐标 → 笛卡尔 → int32 mm)
    // ============================================================================

    [[nodiscard]] inline std::vector<RawPoint> lidar_ranges_to_raw_points(
        const LidarFrame& lidar, double fov, double vfov)
    {
        std::vector<RawPoint> points;
        points.reserve(lidar.ranges.size());

        for (uint16_t j = 0; j < lidar.num_layers; ++j)
        {
            double phi = -vfov / 2.0
                       + (static_cast<double>(j) + 0.5) * vfov / lidar.num_layers;
            double cos_phi = std::cos(phi);
            double sin_phi = std::sin(phi);

            for (uint16_t i = 0; i < lidar.horiz_res; ++i)
            {
                double theta = -fov / 2.0
                             + (static_cast<double>(i) + 0.5) * fov / lidar.horiz_res;
                double r = lidar.ranges[static_cast<size_t>(j) * lidar.horiz_res + i];

                if (!std::isfinite(r) || r <= 0.0)
                    continue;

                // LiDAR 坐标系: X 前, Y 左, Z 上
                double x = r * cos_phi * std::sin(theta);
                double y = r * cos_phi * std::cos(theta);
                double z = r * sin_phi;

                RawPoint pt;
                pt.x             = static_cast<int32_t>(std::round(x * 1000.0));
                pt.y             = static_cast<int32_t>(std::round(y * 1000.0));
                pt.z             = static_cast<int32_t>(std::round(z * 1000.0));
                pt.reflectivity  = 128;     // 默认强度
                pt.tag           = 0x10;    // 有效点标志: (tag & 0x30) == 0x10
                points.push_back(pt);
            }
        }
        return points;
    }

} // namespace hsm::webot_proto

#endif // HSM_WEBOT_PROTO_H
