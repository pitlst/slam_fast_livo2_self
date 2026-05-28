#include <string>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <stdlib.h>

#include "readerwriterqueue.h"
#include "sensor/hk_camera.hpp"

using namespace hsm;

static frame_queue _camera_queue(K_BUFFER_CAPACITY);

void __stdcall _hk_camera_callback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo)
    {
        // 相机时间戳
        uint64_t device_ts = 0;
        {
            auto hi   = static_cast<uint64_t>(pFrameInfo->nDevTimeStampHigh);
            auto lo   = static_cast<uint64_t>(pFrameInfo->nDevTimeStampLow);
            device_ts = (hi << 32) | lo;
        }
        // 主机时间戳
        auto host_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // 图片数据
        cv::Mat img_bayerrg(cv::Size(pFrameInfo->nWidth, pFrameInfo->nHeight), CV_8UC1, pData);
        cv::Mat result;
        cv::cvtColor(img_bayerrg, result, cv::COLOR_BayerRG2RGB);
        // 加入缓冲区
        timestamped<cv::Mat> item {device_ts, host_ts, std::move(result)};
        _camera_queue.try_enqueue(std::move(item));
    }
}

std::unique_ptr<hk_camera> hsm::make_hk_camera(const std::filesystem::path& input_path)
{
    auto config_data   = std::make_unique<camera_config>(input_path);
    auto hk_camera_ptr = std::make_unique<hk_camera>();

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("枚举设备失败，错误号为：{}\n"), nRet));
    throw_if(stDeviceList.nDeviceNum <= 0, "没有找到相机设备");
    for (int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        throw_if(pDeviceInfo == NULL, fmt::format(FMT_COMPILE("找到的设备报错，对应设备号为 {}\n"), i));
    }

    unsigned int nIndex = config_data->device_id;
    nRet                = MV_CC_CreateHandle(&(hk_camera_ptr->handle), stDeviceList.pDeviceInfo[nIndex]);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_CreateHandle fail! nRet {}\n"), nRet));
    nRet = MV_CC_OpenDevice(hk_camera_ptr->handle);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_OpenDevice fail! nRet {}\n"), nRet));
    nRet = MV_CC_SetEnumValue(hk_camera_ptr->handle, "TriggerMode", 0);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_SetTriggerMode fail! nRet {}\n"), nRet));

    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "OffsetX", 0);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置OffsetX错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "OffsetY", 0);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置OffsetY错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetFloatValue(hk_camera_ptr->handle, "ExposureTime", config_data->exposure);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置曝光错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "Width", config_data->width);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置Width错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "Height", config_data->height);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置Height错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "OffsetX", config_data->offset_x);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置OffsetX错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "OffsetY", config_data->offset_y);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置OffsetY错误,错误码: {}\n"), nRet));

    nRet = MV_CC_SetEnumValue(hk_camera_ptr->handle, "PixelFormat", 0x01080009);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置传输图像格式错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetFloatValue(hk_camera_ptr->handle, "Gain", config_data->gain);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置增益错误,错误码: {}\n"), nRet));

    nRet = MV_CC_RegisterImageCallBackEx(hk_camera_ptr->handle, _hk_camera_callback, nullptr);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_RegisterImageCallBackEx fail! nRet {}\n"), nRet));

    nRet = MV_CC_StartGrabbing(hk_camera_ptr->handle);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_StartGrabbing fail! nRet {}\n"), nRet));
    fmt::print("[hik camera] 相机初始化完成\n");
    return hk_camera_ptr;
}

hk_camera::~hk_camera()
{
    int nRet = MV_CC_StopGrabbing(this->handle);
    if (MV_OK != nRet)
    {
        fmt::print(FMT_COMPILE("MV_CC_StopGrabbing fail! nRet [{}]\n"), nRet);
        exit(1);
    }

    nRet = MV_CC_CloseDevice(this->handle);
    if (MV_OK != nRet)
    {
        fmt::print(FMT_COMPILE("MV_CC_CloseDevice fail! nRet [{}]\n"), nRet);
        exit(1);
    }

    nRet = MV_CC_DestroyHandle(this->handle);
    if (MV_OK != nRet)
    {
        fmt::print(FMT_COMPILE("MV_CC_DestroyHandle fail! nRet [{}]\n"), nRet);
        exit(1);
    }

    if (nRet != MV_OK)
    {
        if (this->handle != NULL)
        {
            MV_CC_DestroyHandle(this->handle);
            this->handle = NULL;
        }
    }
    fmt::print("[hik camera] 成功关闭\n");
}

timestamped<cv::Mat> hk_camera::get()
{
    timestamped<cv::Mat> item;
    if (_camera_queue.try_dequeue(item))
    {
        return std::move(item);
    }
    return {};
}
