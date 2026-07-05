#include "fmt/printf.h"

#include "sensor/webot_lidar.hpp"
#include "common/enhanced_exception.hpp"

using namespace hsm;

static point_queue _point_queue(K_BUFFER_CAPACITY);
static imu_queue   _imu_queue(K_BUFFER_CAPACITY);

std::shared_ptr<webot_lidar> hsm::make_webot_lidar(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url, Eigen::Matrix3f const& imu_rotation)
{
    static bool is_init = false;
    throw_if(is_init, fmt::format(FMT_COMPILE("尝试重复初始化激光雷达\n")));

    auto webots_lidar_ptr = std::make_shared<webot_lidar>();
    webots_lidar_ptr->running_label.store(true);
    webots_lidar_ptr->back_thread = std::jthread(
        [conetxt, connect_url, imu_rotation, webots_lidar_ptr]()
        {
            zmq::socket_t socket(*conetxt, zmq::socket_type::sub);
            socket.connect(connect_url);
            socket.set(zmq::sockopt::subscribe, "lidar");
            socket.set(zmq::sockopt::subscribe, "imu");
            socket.set(zmq::sockopt::rcvtimeo, 100);
            fmt::print("[webot_lidar] ZMQ 已经连接到 {}\n", connect_url);

            zmq::pollitem_t poll_items[] = {{static_cast<void*>(socket), 0, ZMQ_POLLIN, 0}};
            while (webots_lidar_ptr->running_label.load())
            {
                try
                {
                    int rc = zmq::poll(poll_items, 1, std::chrono::milliseconds(100));
                    if (rc > 0 && (poll_items[0].revents & ZMQ_POLLIN))
                    {
                        // 多部分消息: topic + payload
                        zmq::message_t topic_msg, payload_msg;
                        std::ignore = socket.recv(topic_msg);
                        std::ignore = socket.recv(payload_msg);

                        std::string_view topic(static_cast<char const*>(topic_msg.data()), topic_msg.size());
                        auto             payload = std::span<uint8_t const>(static_cast<uint8_t const*>(payload_msg.data()), payload_msg.size());
                        if (topic == "lidar")
                        {
                            point_data res_data;
                            
                            // 解析头部
                            {
                                LidarHeader header;
                                std::memcpy(&header, payload.data(), sizeof(LidarHeader));
                                res_data.dot_num = header.point_num;
                                res_data.time_interval = timebase
                            }




                            size_t points_offset  = sizeof(LidarHeader);
                            size_t expected_bytes = points_offset + header.point_num * sizeof(CustomPoint);

                            if (payload.size() < expected_bytes)
                            {
                                fmt::print("[webot_lidar] 数据长度和自描述不符");
                                continue;
                            }

                            auto points = std::span<CustomPoint const>(reinterpret_cast<CustomPoint const*>(payload.data() + points_offset), header.point_num);
                            
                            if (points.empty())
                            {
                                fmt::print("[webot_lidar] 点云数据为空");
                                continue;
                            }

                            for 

                            point_data res_data;

                            raw_point 


                            uint64_t host_ts = get_now_pc_time();
                            // _point_queue.try_enqueue(timestamped<point_data> {lidar.timestamp_us, host_ts, std::move(pd)});
                        }
                        else if (topic == "imu")
                        {
                            double timestamp_us = unpack_le<double>(payload);

                            imu_data raw;
                            raw.gyro_x = unpack_le<double>(payload, 4 * sizeof(double));
                            raw.gyro_y = unpack_le<double>(payload, 5 * sizeof(double));
                            raw.gyro_z = unpack_le<double>(payload, 6 * sizeof(double));
                            raw.acc_x  = unpack_le<double>(payload, 1 * sizeof(double));
                            raw.acc_y  = unpack_le<double>(payload, 2 * sizeof(double));
                            raw.acc_z  = unpack_le<double>(payload, 3 * sizeof(double));

                            // 应用 IMU 外参旋转 (与 mid360_lidar 的 _livox_imu_callback 一致)
                            Eigen::Vector3f const gyro(raw.gyro_x, raw.gyro_y, raw.gyro_z);
                            Eigen::Vector3f const accel(raw.acc_x, raw.acc_y, raw.acc_z);
                            Eigen::Vector3f const g = imu_rotation * gyro;
                            Eigen::Vector3f const a = imu_rotation * accel;

                            raw.gyro_x = g.x();
                            raw.gyro_y = g.y();
                            raw.gyro_z = g.z();
                            raw.acc_x  = a.x();
                            raw.acc_y  = a.y();
                            raw.acc_z  = a.z();

                            uint64_t host_ts          = get_now_pc_time();
                            uint64_t device_timestamp = static_cast<uint64_t>(timestamp_us * 1000000000);
                            _imu_queue.try_enqueue(timestamped<imu_data> {device_timestamp, host_ts, raw});
                        }
                    }
                }
                catch (std::exception const& e)
                {
                    fmt::print("[webot_lidar] 解析发生错误: {}\n", e.what());
                }
            }
            socket.close();
            fmt::print("[webot_lidar] 激光雷达获取线程退出\n");
        });
    return webots_lidar_ptr;
}

webot_lidar::~webot_lidar()
{
    this->running_label.store(false);
    fmt::print("[webot_lidar] 已析构\n");
}

bool webot_lidar::get_points(timestamped<point_data>& out)
{
    return _point_queue.try_dequeue(out);
}

bool webot_lidar::get_imu(timestamped<imu_data>& out)
{
    return _imu_queue.try_dequeue(out);
}