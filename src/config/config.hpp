#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

#include "fmt/core.h"
#include "fmt/format.h"
#include "fmt/compile.h"
#include "toml.hpp"

#include "common/exception.hpp"

namespace hsm
{
    // 读取toml配置中的数据
    template<typename T, typename... Keys>
    T parser_config_item(const std::filesystem::path& input_path, const toml::table& toml_data, std::string_view first_key, Keys... rest_keys)
    {
        // 链式访问配置节点: toml_data[key1][key2][key3]...
        auto node = toml_data[first_key];
        ((node = node[rest_keys]), ...);

        std::optional<T> value = node.value<T>();
        if (! value.has_value())
        {
            // 构建完整键路径用于错误信息 (如: "database.host")
            std::vector<std::string> full_key_array;
            full_key_array.emplace_back(first_key);
            ((full_key_array.emplace_back(fmt::format(FMT_COMPILE("{}"), rest_keys))), ...);

            std::string full_key = absl::StrJoin(full_key_array, ".");

            throw_runtime(fmt::format(FMT_COMPILE("路径{},配置项{}类型不正确或不存在\n"), input_path.string(), full_key));
        }
        return value.value();
    }

    // 检查文件是否存在
    inline void check_file_exist(const std::filesystem::path& input_path)
    {
        throw_if(! std::filesystem::exists(input_path), fmt::format(FMT_COMPILE("路径{}不存在\n"), input_path.string()));
        throw_if(! std::filesystem::is_regular_file(input_path), fmt::format(FMT_COMPILE("路径{}对应不是文件\n"), input_path.string()));
    }

    // 检查文件夹是否存在
    inline void check_dir_exist(const std::filesystem::path& input_path)
    {
        throw_if(! std::filesystem::exists(input_path), fmt::format(FMT_COMPILE("路径{}不存在\n"), input_path.string()));
        throw_if(! std::filesystem::is_directory(input_path), fmt::format(FMT_COMPILE("路径{}对应不是文件夹\n"), input_path.string()));
    }

    // 读取文件内容
    inline std::string read_file(const std::filesystem::path& input_path)
    {
        check_file_exist(input_path);
        std::string content;
        content.reserve(std::filesystem::file_size(input_path));
        std::ifstream file(input_path);
        content.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
        return content;
    }

    struct camera_config
    {
    public:
        camera_config();
        explicit camera_config(const std::filesystem::path& input_path);

    private:
        void parser();

    public:
        int device_id;
        int width;
        int height;
        int offset_x;
        int offset_y;
        int exposure;
        int gain;

    private:
        const std::filesystem::path _path;
    };

    struct config
    {
    public:
        config();
        explicit config(const std::filesystem::path& input_path);

    private:
        void parser();

    public:
        int    cameara_median_MAD_windows_size;
        int    point_median_MAD_windows_size;
        int    imu_median_MAD_windows_size;

    private:
        const std::filesystem::path _path;
    };
} // namespace hsm

#endif