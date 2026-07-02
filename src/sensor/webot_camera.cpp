#include "zstd.h"
#include "sensor/webot_camera.hpp"
#include "common/enhanced_exception.hpp"

using namespace hsm;

static frame_queue _camera_queue(K_BUFFER_CAPACITY);

std::shared_ptr<webot_camera> hsm::make_webot_camera(std::shared_ptr<zmq::context_t> conetxt, const std::string& connect_url = "tcp://localhost:5555")
{
    static bool is_init = false;
    throw_if(is_init, fmt::format(FMT_COMPILE("尝试重复初始化相机\n")));

    auto webots_camera_ptr = std::make_shared<webot_camera>();
    webots_camera_ptr->conetxt = conetxt;
    webots_camera_ptr->socket = std::make_unique<zmq::socket_t>(*(webots_camera_ptr->conetxt), zmq::socket_type::sub);
    webots_camera_ptr->socket->connect(connect_url);
    webots_camera_ptr->socket->set(zmq::sockopt::subscribe, "camera");
    webots_camera_ptr->socket->set(zmq::sockopt::rcvtimeo, 100);
    webots_camera_ptr->running_label.store(true);
}

webot_camera::~webot_camera()
{
    this->socket->close();
    fmt::print("[webot_camera] 成功关闭\n");
}

bool webot_camera::get(timestamped<cv::Mat>& out)
{
    return _camera_queue.try_dequeue(out);
}
