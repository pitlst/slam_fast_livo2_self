#include "time_sync/time_sync.hpp"

using namespace hsm;

static std::string to_string(const time_sync::clock_model& input_data)
{
    // 将纳秒时间戳拆分为 Unix 秒 + 纳秒余数，比 19 位纯数字更直观
    const uint64_t sec  = input_data.last_update_ns / 1'000'000'000ULL;
    const uint64_t nsec = input_data.last_update_ns % 1'000'000'000ULL;

    return fmt::format(
        FMT_COMPILE("offset={:.6f}s, mad={}, last_update={}.{:09}"),
        input_data.offset_sec,
        input_data.mad,
        sec,
        nsec);
}

double time_sync::update(uint64_t device_timestamp, uint64_t host_timestamp)
{
    // 添加新的数据并维护对应的结果
    timestamped<double> input_res;
    input_res.device_timestamp = device_timestamp;
    input_res.host_timestamp   = host_timestamp;
    input_res.payload          = host_timestamp - device_timestamp;
    this->time_buffer.emplace_front(input_res);
    if (this->time_buffer.size() > windows_size)
    {
        this->time_buffer.pop_back();
    }
    // 转换std::deque<timestamped<double>>为std::vector<double>
    std::vector<double> prepare_data;
    prepare_data.reserve(this->time_buffer.size());
    for (const auto& ch : this->time_buffer)
    {
        prepare_data.emplace_back(ch.payload);
    }
    // 获取当前中位数,绝对中位差
    auto [med, mad] = median_absolute_deviation(prepare_data);
    // 过滤异常数据
    std::vector<double> mean_data_;
    if (mad < 1e-6)
    {
        for (const auto& ch : this->time_buffer)
        {
            if (std::abs(ch.payload - med) <= 0.001)
            {
                mean_data_.emplace_back(ch.payload);
            }
        }
    }
    else
    {
        for (const auto& ch : this->time_buffer)
        {
            if (std::abs(ch.payload - med) <= 3 * mad)
            {
                mean_data_.emplace_back(ch.payload);
            }
        }
    }    
    // 更新当前状态
    this->status.offset_sec     = std::accumulate(mean_data_.begin(), mean_data_.end(), 0.0) / mean_data_.size();
    this->status.mad            = mad;
    this->status.last_update_ns = host_timestamp;
    // 返回之后的预测值
    return this->status.offset_sec + device_timestamp;
}