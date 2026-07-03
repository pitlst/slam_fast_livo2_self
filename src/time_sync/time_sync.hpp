#ifndef HSM_TIME_SYNC_H
#define HSM_TIME_SYNC_H

#include <cstdint>
#include <deque>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <cmath>

#include "fmt/core.h"
#include "fmt/format.h"
#include "fmt/compile.h"

#include "common/common.hpp"
#include "common/struct.hpp"

namespace hsm
{
    // 激光雷达和相机的初始化时间不相同
    // 这个类就是通过获取相机和激光雷达消息的帧时间与计算机本地时间
    // 估算激光雷达和相机硬件获取到到达计算机的时间差
    // 让时间差减去对应的pc时间获取对应的pc基准的硬件数据获取时间
    class time_sync
    {
    public:
        struct clock_model
        {
            double   offset_sec     = 0.0; // 当前偏移的时间数
            double   mad            = 0.0; // 绝对中位差
            uint64_t last_update_ns = 0;

            // 格式化输出，便于调试
            static std::string to_string(clock_model const& input_data);
        };

        time_sync(size_t windows_size): windows_size(windows_size) {}

        // 更新对应的时间序列，用于获取偏差
        double update(uint64_t device_timestamp, uint64_t host_timestamp);
        // 获取当前时间同步器的状态，用于评估
        inline clock_model get_model() const
        {
            return this->status;
        }

    private:
        // 时间窗口大小限制
        size_t const windows_size;
        // 当前所有的时间数据
        std::deque<timestamped<double>> time_buffer;
        // 时间同步器的状态
        clock_model status;
    };
} // namespace hsm

#endif