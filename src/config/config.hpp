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
} // namespace hsm

#endif