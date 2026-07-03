#include "zstd.h"
#include "sensor/webot_camera.hpp"
#include "common/zstd_warpper.hpp"
#include "common/enhanced_exception.hpp"

using namespace hsm;

static frame_queue _camera_queue(K_BUFFER_CAPACITY);

std::shared_ptr<webot_camera> hsm::make_webot_camera(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url = "tcp://localhost:5555")
{
    static bool is_init = false;
    throw_if(is_init, fmt::format(FMT_COMPILE("尝试重复初始化相机\n")));

    auto webots_camera_ptr = std::make_shared<webot_camera>();
    webots_camera_ptr->running_label.store(true);
    webots_camera_ptr->back_thread = std::jthread(
        [conetxt, connect_url, webots_camera_ptr]()
        {
            zstd::decompressor decomp;
            zmq::socket_t      socket(*conetxt, zmq::socket_type::sub);
            socket.connect(connect_url);
            socket.set(zmq::sockopt::subscribe, "camera");
            socket.set(zmq::sockopt::rcvtimeo, 100);
            fmt::print(stderr, "[webot camera] ZMQ 已经连接到 {}\n", connect_url);
            try
            {
                while (webots_camera_ptr->running_label.load())
                {
                    zmq::message_t topic, payload;
                    socket.recv(topic, zmq::recv_flags::none);
                    socket.recv(payload, zmq::recv_flags::none);

                    uint64_t host_ts           = get_now_pc_time();
                    auto [timestamp_us, frame] = decomp.decompress(payload.data(), payload.size());
                    // webots仿真提返回的是基于秒的浮点数，这里转换为统一的纳秒
                    uint64_t device_timestamp = static_cast<uint64_t>(timestamp_us * 1000000000);
                    _camera_queue.try_enqueue(timestamped<cv::Mat> {device_timestamp, host_ts, frame});
                }
            }
            catch (std::exception const& e)
            {
                fmt::print("图像获取线程发生错误：{}", e.what());
            }
            socket.close();
        });
    return webots_camera_ptr;
}

webot_camera::~webot_camera()
{
    fmt::print("[webot_camera] 成功关闭\n");
}

bool webot_camera::get(timestamped<cv::Mat>& out)
{
    return _camera_queue.try_dequeue(out);
}
