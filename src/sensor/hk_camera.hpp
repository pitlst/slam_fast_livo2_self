#ifndef HSM_HKCAMERA_H
#define HSM_HKCAMERA_H

#include <string>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <stdlib.h>

#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include "readerwriterqueue.h"

#include "common/common.hpp"
#include "common/struct.hpp"
#include "common/enhanced_exception.hpp"

namespace hsm
{
    // 相机相关的配置，直接写死在这里
    constexpr int device_id = 0;
    constexpr int width     = 1440;
    constexpr int height    = 1080;
    constexpr int offset_x  = 0;
    constexpr int offset_y  = 0;
    constexpr int exposure  = 5000;
    constexpr int gain      = 0;

    struct hk_camera
    {
        friend std::unique_ptr<hk_camera> make_hk_camera();

    public:
        ~hk_camera();

        bool get(timestamped<cv::Mat>& out);

    private:
        void* handle = nullptr;
    };

    // 工厂函数
    std::unique_ptr<hk_camera> make_hk_camera();
} // namespace hsm

#endif
