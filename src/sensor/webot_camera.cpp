#include <chrono>
#include <thread>
#include <string>

#include <zstd.h>

#include "sensor/webot_camera.hpp"
#include "zmq.hpp"

using namespace hsm;

static frame_queue _camera_queue(K_BUFFER_CAPACITY);

static void _webot_camera_recv_loop(
    std::atomic<bool>& running,
    std::string        host,
    uint16_t           port)
{
}

std::shared_ptr<webot_camera> hsm::make_webot_camera(const std::string& host, size_t port)
{
    static bool is_init = false;
    throw_if(is_init, fmt::format(FMT_COMPILE("尝试重复初始化相机\n")));

    std::string endpoint = "tcp://" + host + ":" + std::to_string(port);

    auto webots_camera_ptr = std::make_shared<webot_camera>();
    webots_camera_ptr->running_label.store(true);
    webots_camera_ptr->back_thread = std::jthread(
        [endpoint, webots_camera_ptr]()
        {
            // ZMQ 上下文与 SUB 套接字 (线程局部, 析构自动清理)
            zmq::context_t ctx(1);
            zmq::socket_t  sub(ctx, zmq::socket_type::sub);
            // 只订阅 "camera" 主题 — 其他消息 (lidar, info) 不会被传递到此 socket
            sub.set(zmq::sockopt::subscribe, "camera");
            // 低水位: 最多缓存 4 帧, 防止慢消费时积压
            sub.set(zmq::sockopt::rcvhwm, 4);
            sub.set(zmq::sockopt::linger, 0);
            sub.connect(endpoint);
            fmt::print(stderr, "[webot_camera] ZMQ SUB 连接到 {}\n", endpoint);

            // poll item: 用于非阻塞检查 + 可被 running 中断
            zmq::pollitem_t poll_items[] = {{static_cast<void*>(sub), 0, ZMQ_POLLIN, 0}};
            while (webots_camera_ptr->running_label.load())
            {
                // 100ms 超时 poll: 定期检查 running 标志, 允许干净退出
                int rc = zmq::poll(poll_items, 1, std::chrono::milliseconds(100));
                if (rc > 0 && (poll_items[0].revents & ZMQ_POLLIN))
                {
                    try
                    {
                        // ZMQ 多部分消息: 第1部分 topic, 第2部分 payload
                        zmq::message_t topic_msg, payload_msg;
                        std::ignore = sub.recv(topic_msg);
                        std::ignore = sub.recv(payload_msg);

                        // 构造 span 供解析器使用
                        auto payload = std::span<const uint8_t>(static_cast<const uint8_t*>(payload_msg.data()), payload_msg.size());
                        auto cam     = webot_proto::parse_camera(payload);

                        // BGRA → RGB (cv::cvtColor 创建新矩阵, 不依赖原数据生命周期)
                        cv::Mat rgb;
                        cv::Mat bgra(cam.height, cam.width, CV_8UC4, const_cast<uint8_t*>(cam.image.data()));
                        cv::cvtColor(bgra, rgb, cv::COLOR_BGRA2RGB);

                        uint64_t             host_ts = get_now_pc_time();
                        timestamped<cv::Mat> item {cam.timestamp_us, host_ts, std::move(rgb)};
                        _camera_queue.try_enqueue(std::move(item));
                    }
                    catch (const std::exception& e)
                    {
                        fmt::print(stderr, "[webot_camera] 解析错误: {}\n", e.what());
                    }
                }
            }
            fmt::print(stderr, "[webot_camera] 接受后台线程退出\n");
        });
    return webots_camera_ptr;
}

webot_camera::~webot_camera()
{
    this->running_label.store(false);
    fmt::print("[webot_camera] 成功关闭\n");
}

bool webot_camera::get(timestamped<cv::Mat>& out)
{
    return _camera_queue.try_dequeue(out);
}
