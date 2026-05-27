#include "sensor/mid360_lidar.hpp"

using namespace hsm;

std::vector<LivoxLidarCartesianHighRawPoint> _livox_points;
std::mutex                                   _livox_points_mutex;
LivoxLidarImuRawPoint                        _livox_imu;
std::mutex                                   _livox_imu_mutex;

void _livox_point_callback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    if (! data || data->data_type != kLivoxLidarCartesianCoordinateHighData)
    {
        return;
    }

    auto*                                        raw = (LivoxLidarCartesianHighRawPoint*) data->data;
    std::vector<LivoxLidarCartesianHighRawPoint> buf(raw, raw + data->dot_num);
    {
        std::lock_guard<std::mutex> _lock(_livox_points_mutex);
        std::swap(_livox_points, buf);
    }
}

void _livox_imu_callback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    if (! data || data->data_type != kLivoxLidarImuData)
    {
        return;
    }

    auto* raw = (LivoxLidarImuRawPoint*) data->data;
    {
        std::lock_guard<std::mutex> _lock(_livox_imu_mutex);
        _livox_imu = *raw;
    }
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
    // 注册回调
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

std::vector<LivoxLidarCartesianHighRawPoint> livox_lidar::get_points()
{
    std::lock_guard<std::mutex> _lock(_livox_points_mutex);
    return _livox_points;
}

LivoxLidarImuRawPoint livox_lidar::get_imu()
{
    std::lock_guard<std::mutex> _lock(_livox_imu_mutex);
    return _livox_imu;
}