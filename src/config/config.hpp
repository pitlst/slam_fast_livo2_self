#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

#include <chrono>
#include <filesystem>

namespace hsm
{
    struct camera_config
    {
    public:
        camera_config();
        explicit camera_config(const std::filesystem::path& input_path);

    private:
        void parser();

    public:
        int  device_id;
        int  width;
        int  height;
        int  offset_x;
        int  offset_y;
        int  exposure;
        int  gain;

    private:
        const std::filesystem::path _path;
    };

    struct mid360_config
    {
    public:
        mid360_config();
        explicit mid360_config(const std::filesystem::path& input_path);

    private:
        void parser();

    public:
        std::string host_ip;
        std::string multicast_ip;
        int cmd_port;
        int push_port;
        int point_port;
        int imu_port;
        int log_port;
        std::string lidar_type;

    private:
        const std::filesystem::path _path;
    };
} // namespace hsm

#endif