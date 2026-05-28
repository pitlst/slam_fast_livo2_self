#include <cstring>
#include <bit>


#include "sensor/mid360_lidar.hpp"

using namespace hsm;

static point_queue _point_queue(K_BUFFER_CAPACITY);
static imu_queue   _imu_queue(K_BUFFER_CAPACITY);

uint64_t _parse_livox_timestamp(const uint8_t ts[8])
{
    uint64_t raw;
    std::memcpy(&raw, ts, sizeof(raw));
    if constexpr (std::endian::native == std::endian::little)
    {
        raw = __builtin_bswap64(raw);
    }
    return raw;
}

void _livox_point_callback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    if (! data || data->data_type != kLivoxLidarCartesianCoordinateHighData)
    {
        return;
    }
    // 雷达时间戳
    uint64_t device_ts = _parse_livox_timestamp(data->timestamp);
    // 主机时间戳
    auto host_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // 雷达数据
    auto*      raw = (LivoxLidarCartesianHighRawPoint*) data->data;
    point_data buf(raw, raw + data->dot_num);
    // 加入缓冲区
    timestamped<point_data> item {device_ts, host_ts, std::move(buf)};
    _point_queue.try_enqueue(std::move(item));
}

void _livox_imu_callback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    if (! data || data->data_type != kLivoxLidarImuData)
    {
        return;
    }
    // 雷达时间戳
    uint64_t device_ts = _parse_livox_timestamp(data->timestamp);
    // 主机时间戳
    auto host_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // imu数据
    auto* raw = (LivoxLidarImuRawPoint*) data->data;
    // 加入缓冲区
    timestamped<imu_data> item {device_ts, host_ts, *raw};
    _imu_queue.try_enqueue(std::move(item));
}

void _livox_device_online_callback(uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
    if (! info) return;
    fmt::print("[Livox] 设备在线, SN: {}\n", info->sn);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
}

std::unique_ptr<livox_lidar> hsm::make_livox_lidar(const std::filesystem::path& input_path)
{
    auto lidar_ptr = std::make_unique<livox_lidar>();
    auto label     = LivoxLidarSdkInit(input_path.c_str());
    throw_if(label, fmt::format(FMT_COMPILE("LivoxLidarSdkInit fail!,激光雷达初始化失败\n")));

    SetLivoxLidarPointCloudCallBack(_livox_point_callback, nullptr);
    SetLivoxLidarImuDataCallback(_livox_imu_callback, nullptr);
    SetLivoxLidarInfoChangeCallback(_livox_device_online_callback, nullptr);

    fmt::print("[Livox] 激光雷达初始化完成\n");
    return lidar_ptr;
}

livox_lidar::~livox_lidar()
{
    LivoxLidarSdkUninit();
    fmt::print("[Livox] 成功关闭\n");
}

timestamped<point_data> livox_lidar::get_points(uint64_t& timestamp)
{
    timestamped<point_data> item;
    if (_point_queue.try_dequeue(item))
    {
        return std::move(item);
    }
    return {};
}

timestamped<imu_data> livox_lidar::get_imu(uint64_t& timestamp)
{
    timestamped<imu_data> item;
    if (_imu_queue.try_dequeue(item))
    {
        return std::move(item);
    }
    return {};
}
