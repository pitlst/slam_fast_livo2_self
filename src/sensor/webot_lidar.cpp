#include <chrono>
#include <thread>
#include <string>

#include <zmq.hpp>

#include "sensor/webot_lidar.hpp"
#include "sensor/webot_proto.hpp"

using namespace hsm;

static point_queue _point_queue(K_BUFFER_CAPACITY);
static imu_queue   _imu_queue(K_BUFFER_CAPACITY);

// 外参旋转矩阵
static Eigen::Matrix3f _imu_rotation = Eigen::Matrix3f::Identity();

std::shared_ptr<webot_lidar> hsm::make_webot_lidar(const std::string& host, size_t port, const Eigen::Matrix3f& imu_rotation)
{
    static bool is_init = false;
    throw_if(is_init, fmt::format(FMT_COMPILE("尝试重复初始化激光雷达\n")));

    _imu_rotation        = imu_rotation;
    std::string endpoint = "tcp://" + host + ":" + std::to_string(port);

    auto webots_lidar_ptr = std::make_shared<webot_lidar>();
    webots_lidar_ptr->running_label.store(true);
    webots_lidar_ptr->back_thread = std::jthread(
        [endpoint, webots_lidar_ptr]()
        {
            // 从 "info" 主题中提取的传感器参数
            webot_proto::SensorInfo sensor_info;
            // ZMQ 上下文与 SUB 套接字 (线程局部)
            zmq::context_t ctx(1);
            zmq::socket_t  sub(ctx, zmq::socket_type::sub);
            // 订阅 "lidar" + "info" + "imu" 主题
            sub.set(zmq::sockopt::subscribe, "lidar");
            sub.set(zmq::sockopt::subscribe, "info");
            sub.set(zmq::sockopt::subscribe, "imu");
            sub.set(zmq::sockopt::rcvhwm, 4);
            sub.set(zmq::sockopt::linger, 0);
            sub.connect(endpoint);
            fmt::print(stderr, "[webot_lidar] ZMQ SUB connected to {}\n", endpoint);

            zmq::pollitem_t poll_items[] = {{static_cast<void*>(sub), 0, ZMQ_POLLIN, 0}};
            while (webots_lidar_ptr->running_label.load())
            {
                int rc = zmq::poll(poll_items, 1, std::chrono::milliseconds(100));
                if (rc > 0 && (poll_items[0].revents & ZMQ_POLLIN))
                {
                    try
                    {
                        // 多部分消息: topic + payload
                        zmq::message_t topic_msg, payload_msg;
                        std::ignore = sub.recv(topic_msg);
                        std::ignore = sub.recv(payload_msg);

                        std::string_view topic(static_cast<const char*>(topic_msg.data()), topic_msg.size());
                        auto             payload = std::span<const uint8_t>(static_cast<const uint8_t*>(payload_msg.data()), payload_msg.size());
                        if (topic == "info")
                        {
                            std::string_view json(reinterpret_cast<const char*>(payload.data()), payload.size());
                            sensor_info = webot_proto::parse_metadata(json);
                            fmt::print("[webot_lidar] Metadata: FOV={:.3f} rad (H) x {:.3f} rad (V)\n", sensor_info.lidar_fov, sensor_info.lidar_vfov);
                        }
                        else if (topic == "lidar")
                        {
                            auto lidar = webot_proto::parse_lidar(payload);
                            // 距离阵 → 原始点云 (int32 mm)
                            auto raw_pts = webot_proto::lidar_ranges_to_raw_points(lidar, sensor_info.lidar_fov, sensor_info.lidar_vfov);
                            if (raw_pts.empty())
                            {
                                continue;
                            }
                            // 打包为 point_data, 与 livox_lidar 输出格式一致
                            point_data pd;
                            pd.dot_num       = static_cast<uint16_t>(raw_pts.size());
                            pd.time_interval = 0; // 所有点时间戳 = packet timestamp
                            pd.points        = std::move(raw_pts);

                            uint64_t host_ts = get_now_pc_time();
                            _point_queue.try_enqueue(timestamped<point_data> {lidar.timestamp_us, host_ts, std::move(pd)});
                        }
                        else if (topic == "imu")
                        {
                            auto imu_frame = webot_proto::parse_imu(payload);

                            imu_data raw;
                            raw.gyro_x = imu_frame.gyro[0];
                            raw.gyro_y = imu_frame.gyro[1];
                            raw.gyro_z = imu_frame.gyro[2];
                            raw.acc_x  = imu_frame.accel[0];
                            raw.acc_y  = imu_frame.accel[1];
                            raw.acc_z  = imu_frame.accel[2];

                            // 应用 IMU 外参旋转 (与 mid360_lidar 的 _livox_imu_callback 一致)
                            const Eigen::Vector3f gyro(raw.gyro_x, raw.gyro_y, raw.gyro_z);
                            const Eigen::Vector3f accel(raw.acc_x, raw.acc_y, raw.acc_z);
                            const Eigen::Vector3f g = _imu_rotation * gyro;
                            const Eigen::Vector3f a = _imu_rotation * accel;

                            raw.gyro_x = g.x();
                            raw.gyro_y = g.y();
                            raw.gyro_z = g.z();
                            raw.acc_x  = a.x();
                            raw.acc_y  = a.y();
                            raw.acc_z  = a.z();

                            uint64_t host_ts = get_now_pc_time();
                            _imu_queue.try_enqueue(timestamped<imu_data> {imu_frame.timestamp_us, host_ts, raw});
                        }
                    }
                    catch (const std::exception& e)
                    {
                        fmt::print(stderr, "[webot_lidar] Parse error: {}\n", e.what());
                    }
                }
            }
            fmt::print(stderr, "[webot_lidar] Recv thread exit\n");
        });
    return webots_lidar_ptr;
}

webot_lidar::~webot_lidar()
{
    this->running_label.store(false);
    fmt::print("[webot_lidar] Shutdown\n");
}

bool webot_lidar::get_points(timestamped<point_data>& out)
{
    return _point_queue.try_dequeue(out);
}

bool webot_lidar::get_imu(timestamped<imu_data>& out)
{
    return _imu_queue.try_dequeue(out);
}
