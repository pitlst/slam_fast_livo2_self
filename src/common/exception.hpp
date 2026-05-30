#ifndef SLAM_GENERAL_H
#define SLAM_GENERAL_H

#include <cstddef>
#include <cstdint>

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
#include <filesystem>
#include <chrono>

#include "fmt/core.h"
#include "fmt/format.h"
#include "fmt/compile.h"

namespace hsm
{
    template<typename BaseException>
    class enhanced_exception : public BaseException
    {
    public:
        enhanced_exception(const BaseException& original_exception, const std::string& add_message, const std::source_location& location)
            : BaseException(original_exception)
        {
            this->enhanced_message = fmt::format(
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
            this->enhanced_message = fmt::format(
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


} // namespace hsm

#endif