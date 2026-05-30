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
#include "common/exception.hpp"
#include "config/config.hpp"

namespace hsm
{
    using frame_queue = moodycamel::ReaderWriterQueue<timestamped<cv::Mat>>;

    struct hk_camera
    {
        friend std::unique_ptr<hk_camera> make_hk_camera(const std::filesystem::path& input_path);

    public:
        ~hk_camera();

        timestamped<cv::Mat> get();

    private:
        void* handle = nullptr;
    };

    std::unique_ptr<hk_camera> make_hk_camera(const std::filesystem::path& input_path = std::filesystem::path(PROJECT_PATH) / "config" / "camera.toml");
} // namespace hsm

#endif
