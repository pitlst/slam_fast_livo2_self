#ifndef SLAM_GENERAL_H
#define SLAM_GENERAL_H

#include <expected>
#include <string>
#include <string_view>
#include <array>
#include <source_location>
#include <concepts>
#include <type_traits>
#include <vector>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <exception>

#include "fmt/core.h"
#include "fmt/format.h"
#include "fmt/compile.h"
#include "toml.hpp"
#include "phmap.hpp"

namespace hsm
{
    template<typename BaseException>
    class enhanced_exception : public BaseException
    {
    public:
        enhanced_exception(const BaseException& original_exception, const std::string& add_message, const std::source_location& location)
            : BaseException(original_exception)
        {
            this->enhanced_message = std::format(
                "[{}:{}] {}() - \n-----Add Info-----\n{}\n-----Error-----\n{}",
                location.file_name(),
                location.line(),
                location.function_name(),
                add_message,
                original_exception.what());
        }
        enhanced_exception(const BaseException& original_exception, const std::source_location& location)
            : BaseException(original_exception)
        {
            this->enhanced_message = std::format(
                "[{}:{}] {}() - \n-----Error-----\n{}",
                location.file_name(),
                location.line(),
                location.function_name(),
                original_exception.what());
        }
        [[nodiscard]] inline const char* what() const noexcept override
        {
            return this->enhanced_message.c_str();
        }
        [[nodiscard]] inline const std::string& getEnhancedMessage() const
        {
            return this->enhanced_message;
        }

    private:
        std::string enhanced_message;
    };

    template<typename ExceptionType>
    [[noreturn]] void throw_enhanced(const ExceptionType& exception, const std::string& add_message, const std::source_location& location = std::source_location::current())
    {
        static_assert(std::is_base_of_v<std::exception, ExceptionType>, "Exception type must inherit from std::exception");
        throw enhanced_exception<ExceptionType>(exception, add_message, location);
    }

    template<typename ExceptionType>
    [[noreturn]] void throw_enhanced(const ExceptionType& exception, const std::source_location& location = std::source_location::current())
    {
        static_assert(std::is_base_of_v<std::exception, ExceptionType>, "Exception type must inherit from std::exception");
        throw enhanced_exception<ExceptionType>(exception, location);
    }

    template<typename ExceptionType = std::runtime_error>
    [[noreturn]] void throw_runtime(const std::string& error_message, const std::source_location& location = std::source_location::current())
    {
        static_assert(std::is_base_of_v<std::exception, ExceptionType>, "Exception type must inherit from std::exception");
        throw_enhanced(ExceptionType(error_message), location);
    }

    template<typename ExceptionType = std::runtime_error>
    void throw_if(bool condition, const std::string& error_message, const std::source_location& location = std::source_location::current())
    {
        static_assert(std::is_base_of_v<std::exception, ExceptionType>, "Exception type must inherit from std::exception");
        if (condition)
            throw_enhanced(ExceptionType(error_message), location);
    }

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

    // 从hash map中只读的抽取对应的key为hash set
    template<typename K, typename V>
    gtl::flat_hash_set<K> extract_keys(const gtl::flat_hash_map<K, V>& map)
    {
        gtl::flat_hash_set<K> keys;
        keys.reserve(map.size());
        for (const auto& [key, _] : map)
        {
            keys.insert(key);
        }
        return keys;
    }

} // namespace hsm

#endif