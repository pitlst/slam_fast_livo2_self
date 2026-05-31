#ifndef SLAM_COMMON_H
#define SLAM_COMMON_H

#include <chrono>
#include <cstdint>
#include <vector>
#include <numeric>
#include <tuple>

#include "common/struct.hpp"
#include "common/exception.hpp"

namespace hsm
{
    // 获取uint64_t格式的当前时间
    inline uint64_t get_now_pc_time()
    {
        auto now = std::chrono::system_clock::now();
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    }

    // 计算中位数
    inline double compute_median(const std::vector<double>& values)
    {
        throw_if(values.empty(), "计算中位数的时候输入数据为空");
        std::vector<double> sorted(values.begin(), values.end());
        std::sort(sorted.begin(), sorted.end());
        size_t n = sorted.size();
        if (n % 2 == 1)
        {
            return sorted[n / 2];
        }
        else
        {
            return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
        }
    }

    // 计算标准差
    inline double standard_deviation(const std::vector<double>& value)
    {
        throw_if(value.empty(), "计算标准差的时候输入数据为空");
        double mean   = std::accumulate(value.begin(), value.end(), 0.0) / value.size();
        double sq_sum = 0.0;
        for (double x : value)
        {
            double diff = x - mean;
            sq_sum += diff * diff;
        }
        return std::sqrt(sq_sum / value.size());
    }

    // 计算绝对中位差 MAD
    inline std::tuple<double, double> median_absolute_deviation(const std::vector<double>& value)
    {
        throw_if(value.empty(), "计算绝对中位差的时候输入数据为空");
        // 计算中位数
        double med = compute_median(value);
        // 计算各数据点与中位数的绝对偏差
        std::vector<double> absDeviations;
        absDeviations.reserve(value.size());
        for (double x : value)
        {
            absDeviations.emplace_back(std::abs(x - med));
        }
        // 绝对偏差的中位数即为 MAD
        auto mad = compute_median(absDeviations);
        return std::make_tuple(med, mad);
    }

    // 转换雷达驱动的点位结构数据为算法需要的标准数据
    inline raw_point_data trans_data(const point_data&)
    {

    }

    // 转换雷达驱动imu结构数据为算法需要的标准数据
    inline raw_imu_data trans_data(const imu_data&)
    {
        
    }

} // namespace hsm

#endif