#include "sensor/mid360_lidar.hpp"

using namespace hsm;

static point_queue _point_queue(K_BUFFER_CAPACITY);
static imu_queue   _imu_queue(K_BUFFER_CAPACITY);

// 外参旋转矩阵
static Eigen::Matrix3f _imu_rotation = Eigen::Matrix3f::Identity();

uint64_t _parse_livox_timestamp(uint8_t const ts[8])
{
    uint64_t raw;
    std::memcpy(&raw, ts, sizeof(raw));
    if constexpr (std::endian::native == std::endian::little)
    {
        raw = __builtin_bswap64(raw);
    }
    return raw;
}

void _livox_point_callback(uint32_t handle, uint8_t const dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    if (! data || data->data_type != kLivoxLidarCartesianCoordinateHighData)
    {
        return;
    }
    // 雷达时间戳
    uint64_t const device_ts = _parse_livox_timestamp(data->timestamp);
    // 主机时间戳
    uint64_t const host_ts = hsm::get_now_pc_time();
    // 雷达数据
    uint16_t const count = data->dot_num;
    auto const*    src   = reinterpret_cast<LivoxLidarCartesianHighRawPoint const*>(data->data);

    point_data buf;
    buf.dot_num       = count;
    buf.time_interval = data->time_interval;
    buf.points.reserve(count);
    for (uint16_t i = 0; i < count; ++i)
    {
        raw_point temp_data;
        temp_data.x            = src[i].x;
        temp_data.y            = src[i].y;
        temp_data.z            = src[i].z;
        temp_data.reflectivity = src[i].reflectivity;
        temp_data.tag          = src[i].tag;
        buf.points.emplace_back(std::move(temp_data));
    }
    // 加入缓冲区
    _point_queue.try_enqueue({device_ts, host_ts, std::move(buf)});
}

void _livox_imu_callback(uint32_t handle, uint8_t const dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    if (! data || data->data_type != kLivoxLidarImuData)
    {
        return;
    }
    // 雷达时间戳 (所有采样共享同一包时间戳)
    uint64_t const device_ts = _parse_livox_timestamp(data->timestamp);
    // 主机时间戳
    uint64_t const host_ts = hsm::get_now_pc_time();
    // 本包中的 IMU 采样个数
    uint16_t const count = data->dot_num;
    auto const*    src   = reinterpret_cast<LivoxLidarImuRawPoint const*>(data->data);

    for (uint16_t i = 0; i < count; ++i)
    {
        imu_data raw;
        std::memcpy(&raw, &src[i], sizeof(imu_data));

        Eigen::Vector3f const gyro(raw.gyro_x, raw.gyro_y, raw.gyro_z);
        Eigen::Vector3f const accel(raw.acc_x, raw.acc_y, raw.acc_z);
        Eigen::Vector3f const g = _imu_rotation * gyro;
        Eigen::Vector3f const a = _imu_rotation * accel;

        raw.gyro_x = g.x();
        raw.gyro_y = g.y();
        raw.gyro_z = g.z();
        raw.acc_x  = a.x();
        raw.acc_y  = a.y();
        raw.acc_z  = a.z();

        _imu_queue.try_enqueue({device_ts, host_ts, raw});
    }
}

void _livox_device_online_callback(uint32_t handle, LivoxLidarInfo const* info, void* client_data)
{
    if (! info) return;
    fmt::print("[Livox] 设备在线, SN: {}\n", info->sn);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
}

std::unique_ptr<livox_lidar> hsm::make_livox_lidar(std::filesystem::path const& input_path, Eigen::Matrix3f const& imu_rotation)
{
    static bool is_init = false;
    throw_if(is_init, fmt::format(FMT_COMPILE("尝试重复初始化激光雷达\n")));

    _imu_rotation = imu_rotation;

    auto lidar_ptr = std::make_unique<livox_lidar>();
    auto label     = LivoxLidarSdkInit(input_path.c_str());
    throw_if(! label, fmt::format(FMT_COMPILE("LivoxLidarSdkInit fail!,激光雷达初始化失败\n")));

    SetLivoxLidarPointCloudCallBack(_livox_point_callback, nullptr);
    SetLivoxLidarImuDataCallback(_livox_imu_callback, lidar_ptr.get());
    SetLivoxLidarInfoChangeCallback(_livox_device_online_callback, nullptr);

    fmt::print("[Livox] 激光雷达初始化完成\n");
    is_init = true;
    return lidar_ptr;
}

livox_lidar::~livox_lidar()
{
    LivoxLidarSdkUninit();
    fmt::print("[Livox] 成功关闭\n");
}

bool livox_lidar::get_points(timestamped<point_data>& out)
{
    return _point_queue.try_dequeue(out);
}

bool livox_lidar::get_imu(timestamped<imu_data>& out)
{
    return _imu_queue.try_dequeue(out);
}
