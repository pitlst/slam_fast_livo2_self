#ifndef SLAM_HKCAMERA_H
#define SLAM_HKCAMERA_H

#include <memory>

#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include "readerwriterqueue.h"

#include "general.hpp"
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
