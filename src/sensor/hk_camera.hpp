#ifndef SLAM_HKCAMERA_H
#define SLAM_HKCAMERA_H

#include <mutex>
#include <array>

#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "toml.hpp"

#include "general.hpp"

namespace hsm
{

    // #define DEBUE 1
    struct hk_camera final
    {
    public:
        hk_camera();
        ~hk_camera();

        std::expected<void, Error>

        bool hik_init(const nlohmann::json &input_json, int devive_num);
        bool hik_end();

    private:
        std::mutex frame_mutex;
        cv::Mat frame;

        // 海康相机指针
        void *handle = nullptr;
    };
}

// 暂时仅做了同时接入1个海康相机的支持
extern void __stdcall image_callback(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
extern cv::Mat _frame;
extern std::mutex _mutex;

#endif