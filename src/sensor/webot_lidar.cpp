// ============================================================================
//  webot_lidar.cpp  —  Webots 仿真激光雷达驱动 (ZeroMQ 版)
//
//  通过 ZMQ SUB 从 Webots 控制器接收:
//    - "info" 主题: JSON 元信息 (含 LiDAR FOV)
//    - "lidar" 主题: 距离阵 [layers][horiz_res] float32
//
//  转换为 raw_point (int32 mm) 并推入 SPSC 队列。
//  接口与 livox_lidar 一致:  get_points(timestamped<point_data>&)
//  get_imu() 始终返回 false (Webots 协议不含 IMU)。
//
//  ZMQ 自动处理重连, 无需手动 reconnection 循环。
// ============================================================================

#include "sensor/webot_lidar.hpp"
#include "sensor/webot_proto.hpp"

#include <zmq.hpp> // zmq::context_t, zmq::socket_t, zmq::pollitem_t

#include <chrono>
#include <thread>
#include <string>

using namespace hsm;

// 确保 webot_proto::RawPoint 与 hsm::raw_point 布局兼容
static_assert(sizeof(webot_proto::RawPoint) == sizeof(raw_point), "RawPoint layout mismatch between webot_proto and hsm");
static_assert(alignof(webot_proto::RawPoint) == alignof(raw_point), "RawPoint alignment mismatch between webot_proto and hsm");

static point_queue _point_queue(K_BUFFER_CAPACITY);
static imu_queue   _imu_queue(K_BUFFER_CAPACITY);

// 外参旋转矩阵
static Eigen::Matrix3f _imu_rotation = Eigen::Matrix3f::Identity();

std::shared_ptr<webot_lidar> hsm::make_webot_lidar(const std::string& host, uint16_t port, const Eigen::Matrix3f& imu_rotation)
{
    static bool is_init = false;
    throw_if(is_init, fmt::format(FMT_COMPILE("尝试重复初始化激光雷达\n")));

    _imu_rotation        = imu_rotation;
    std::string endpoint = "tcp://" + host + ":" + std::to_string(port);

    auto webots_lidar_ptr = std::make_shared<webot_lidar>();
    webots_lidar_ptr->running_label.store(true);
    webots_lidar_ptr->back_thread = std::jthread(
        [&endpoint, webots_lidar_ptr]()
        {
            // 从 "info" 主题中提取的传感器参数
            webot_proto::SensorInfo sensor_info;
            // ZMQ 上下文与 SUB 套接字 (线程局部)
            zmq::context_t ctx(1);
            zmq::socket_t  sub(ctx, zmq::socket_type::sub);
            // 订阅 "lidar" + "info" 主题
            sub.set(zmq::sockopt::subscribe, "lidar");
            sub.set(zmq::sockopt::subscribe, "info");
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
                        sub.recv(topic_msg);
                        sub.recv(payload_msg);

                        std::string_view topic(static_cast<const char*>(topic_msg.data()), topic_msg.size());
                        auto payload = std::span<const uint8_t>(static_cast<const uint8_t*>(payload_msg.data()), payload_msg.size());
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
                                continue;
                            // 打包为 point_data, 与 livox_lidar 输出格式一致
                            point_data pd;
                            pd.dot_num       = static_cast<uint16_t>(raw_pts.size());
                            pd.time_interval = 0; // 所有点时间戳 = packet timestamp
                            pd.points.assign(
                                reinterpret_cast<const raw_point*>(raw_pts.data()),
                                reinterpret_cast<const raw_point*>(raw_pts.data() + raw_pts.size()));

                            uint64_t host_ts = get_now_pc_time();
                            _point_queue.try_enqueue(
                                timestamped<point_data> {lidar.timestamp_us, host_ts, std::move(pd)});
                        }
                        // 忽略其他主题 (订阅过滤已保证不会出现)
                    }
                    catch (const std::exception& e)
                    {
                        fmt::print(stderr, "[webot_lidar] Error: {}\n", e.what());
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
    fmt::print("[webot_lidar] 成功关闭\n");
}

bool webot_lidar::get_points(timestamped<point_data>& out)
{
    return _point_queue.try_dequeue(out);
}

bool webot_lidar::get_imu(timestamped<imu_data>& out)
{
    return _imu_queue.try_dequeue(out);
}
