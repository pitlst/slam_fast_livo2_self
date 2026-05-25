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
#include <format>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <exception>

namespace hsm
{
    struct Error
    {
        enum class Code {
            sensor_camera = 0,
            sensor_lidar,
            config,
            general,
            main
        };
        
        static constexpr std::array<std::string_view, 5> _Code_str = {
            "sensor_camera",
            "sensor_lidar",
            "config",
            "general",
            "main"};

        const Code               code;
        std::string              message;
        std::vector<std::string> context;

        Error(Code c, std::string msg, std::vector<std::string> ctx = {}) noexcept
            : code(c), message(std::move(msg)), context(std::move(ctx)) {}

        inline std::string to_string() const noexcept
        {
            std::string context_joined;
            for (size_t i = 0; i < context.size(); ++i)
            {
                if (i > 0)
                    context_joined += "\n    at ";
                context_joined += context[i];
            }
            return std::format(
                "[{}]{}{}",
                _Code_str.at(static_cast<int>(code)),
                context_joined.empty() ? "" : std::format("\n    at {}", context_joined),
                message.empty() ? "" : std::format("\n    {}", message));
        }
    };

    template<Error::Code code>
    inline std::unexpected<Error> make_error(std::string_view msg, std::string_view func = "", const std::source_location& location = std::source_location::current()) noexcept
    {
        std::vector<std::string> ctx;
        if (! func.empty())
            ctx.emplace_back(std::format("{}:{} {}", location.file_name(), location.line(), func));
        else
            ctx.emplace_back(std::format("{}:{} {}", location.file_name(), location.line(), location.function_name()));
        return std::unexpected<Error>(Error {code, std::string(msg), std::move(ctx)});
    }

    inline std::unexpected<Error> make_error(Error::Code code, std::string_view msg, std::string_view func = "", const std::source_location& location = std::source_location::current()) noexcept
    {
        std::vector<std::string> ctx;
        if (! func.empty())
            ctx.emplace_back(std::format("{}:{} {}", location.file_name(), location.line(), func));
        else
            ctx.emplace_back(std::format("{}:{} {}", location.file_name(), location.line(), location.function_name()));
        return std::unexpected<Error>(Error {code, std::string(msg), std::move(ctx)});
    }

    template<typename T>
    struct is_expected : std::false_type
    {
    };

    template<typename T, typename E>
    struct is_expected<std::expected<T, E>> : std::true_type
    {
    };

    template<typename T>
    concept expected_like = is_expected<T>::value;

    template<Error::Code code, std::invocable F>
        requires expected_like<std::invoke_result_t<F>>
    inline auto try_catch(F&& body, const std::source_location& location = std::source_location::current()) noexcept -> std::invoke_result_t<F>
    {
        auto _context = std::format("{}:{} {}", location.file_name(), location.line(), location.function_name());
        try
        {
            auto result = body();
            if (! result)
                result.error().context.emplace_back(_context);
            return result;
        }
        catch (const std::exception& e)
        {
            return make_error<code>(e.what(), _context);
        }
        catch (...)
        {
            return make_error<code>("unknown error", _context);
        }
    }

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
} // namespace hsm

#define ETRY(var, expr)                                             \
    auto&& _expected_##var = (expr);                                \
    if (! (_expected_##var)) [[unlikely]]                           \
    {                                                               \
        return std::unexpected(std::move(_expected_##var).error()); \
    }                                                               \
    auto var = std::move(_expected_##var).value();

#define ECHECKE(expr)                                                 \
    do                                                                \
    {                                                                 \
        auto&& _expected_tmp = (expr);                                \
        if (! _expected_tmp) [[unlikely]]                             \
        {                                                             \
            return std::unexpected(std::move(_expected_tmp).error()); \
        }                                                             \
    }                                                                 \
    while (0)

#define ECHECKV(expected_var)                                        \
    do                                                               \
    {                                                                \
        if (! (expected_var)) [[unlikely]]                           \
        {                                                            \
            return std::unexpected(std::move(expected_var).error()); \
        }                                                            \
    }                                                                \
    while (0)

#define STRY(var, expr, error_ptr)                                                    \
    auto&& _expected_##var = (expr);                                                  \
    if (! (_expected_##var)) [[unlikely]]                                             \
    {                                                                                 \
        error_ptr = std::make_shared<hsm::Error>(std::move(_expected_##var).error()); \
        return false;                                                                 \
    }                                                                                 \
    auto var = std::move(_expected_##var).value();

#define SCHECKE(expr, error_ptr)                                                        \
    do                                                                                  \
    {                                                                                   \
        auto&& _expected_tmp = (expr);                                                  \
        if (! _expected_tmp) [[unlikely]]                                               \
        {                                                                               \
            error_ptr = std::make_shared<hsm::Error>(std::move(_expected_tmp).error()); \
            return false;                                                               \
        }                                                                               \
    }                                                                                   \
    while (0)

#define SCHECKV(expected_var, error_ptr)                                               \
    do                                                                                 \
    {                                                                                  \
        if (! (expected_var)) [[unlikely]]                                             \
        {                                                                              \
            error_ptr = std::make_shared<hsm::Error>(std::move(expected_var).error()); \
            return false;                                                              \
        }                                                                              \
    }                                                                                  \
    while (0)

#define MTRY(var, expr)                                                \
    auto _expected_##var = (expr);                                     \
    if (! (_expected_##var)) [[unlikely]]                              \
    {                                                                  \
        std::cout << _expected_##var.error().to_string() << std::endl; \
        return 1;                                                      \
    }                                                                  \
    auto var = std::move(_expected_##var.value());

#define MCHECKE(expr)                                                    \
    do                                                                   \
    {                                                                    \
        auto&& _expected_tmp = (expr);                                   \
        if (! _expected_tmp) [[unlikely]]                                \
        {                                                                \
            std::cout << _expected_tmp.error().to_string() << std::endl; \
            return 1;                                                    \
        }                                                                \
    }                                                                    \
    while (0)

#endif