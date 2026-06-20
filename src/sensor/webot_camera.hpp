#ifndef HSM_WEBOTCAMERA_H
#define HSM_WEBOTCAMERA_H

#include <string>
#include <string_view>
#include <memory>
#include <thread>
#include <atomic>

#include "opencv2/opencv.hpp"

#include "common/common.hpp"
#include "common/struct.hpp"

namespace hsm
{
    struct webot_camera
    {
        friend std::shared_ptr<webot_camera> make_webot_camera(const std::string& host, size_t port);

    public:
        ~webot_camera();

        bool get(timestamped<cv::Mat>& out);

        std::jthread       back_thread;
        std::atomic<bool> running_label = false;
    };

    // 工厂函数，因为使用后台线程处理消息接受，所以webot_camera需要使用shared_ptr
    std::shared_ptr<webot_camera> make_webot_camera(const std::string& host = "127.0.0.1", size_t port = 12345);
} // namespace hsm

#endif
