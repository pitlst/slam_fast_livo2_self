#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

#include <chrono>
#include <filesystem>

namespace hsm
{
    class config
    {
    public:
        // 读取并解析数据
        config();
        explicit config(const std::filesystem::path& input_path);

    private:
        void parser();

    public:
        int  device_id;
        int  width;
        int  height;
        int  offset_x;
        int  offset_y;
        int  ADC_bit_depth;
        int  exposure;
        int  gain;
        int  balck_level;

    private:
        const std::filesystem::path _path;
    };
} // namespace hsm

#endif