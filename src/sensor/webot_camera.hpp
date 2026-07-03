#ifndef HSM_WEBOTCAMERA_H
#define HSM_WEBOTCAMERA_H

#include <string>
#include <string_view>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include "zmq.hpp"
#include "opencv2/opencv.hpp"

#include "common/common.hpp"
#include "common/struct.hpp"

namespace hsm
{
    struct webot_camera
    {
        friend std::shared_ptr<webot_camera> make_webot_camera(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url);

    public:
        ~webot_camera();

        bool get(timestamped<cv::Mat>& out);

        std::shared_ptr<zmq::context_t> conetxt;
        std::unique_ptr<zmq::socket_t>  socket;
        std::jthread                    back_thread;
        std::atomic<bool>               running_label = false;
    };

    // 工厂函数，所有使用zmq的接收器共用一个上下文
    std::shared_ptr<webot_camera> make_webot_camera(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url = "tcp://localhost:5555");
} // namespace hsm

#endif
