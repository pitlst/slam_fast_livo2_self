#ifndef SLAM_HKCAMERA_H
#define SLAM_HKCAMERA_H

#include <mutex>
#include <array>
#include <expected>

#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "toml.hpp"

#include "general.hpp"
#include "config/config.hpp"

namespace hsm
{
    // #define DEBUE 1
    struct hk_camera
    {
        friend std::unique_ptr<hk_camera> make_hk_camera(std::shared_ptr<camera_config> _config_data);
    public:
        hk_camera() = default;
        ~hk_camera();

        cv::Mat get();

    private:
        std::shared_ptr<camera_config> _config;
        // 海康相机指针
        void* handle = nullptr;
    };

    // 工厂函数
    std::unique_ptr<hk_camera> make_hk_camera(std::shared_ptr<camera_config> _config_data);
} // namespace hsm

// 暂时仅做了同时接入1个海康相机的支持
extern void __stdcall _hk_camera_callback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
extern cv::Mat        _hk_camera_frame;
extern std::mutex     _hk_camera_mutex;

#endif