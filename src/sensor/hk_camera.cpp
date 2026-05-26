#include <string>
#include <sstream>
#include <stdexcept>
#include <stdlib.h>

#include "sensor/hk_camera.hpp"

using namespace hsm;

cv::Mat    _frame;
std::mutex _mutex;

void __stdcall image_callback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo)
    {
        cv::Mat img_bayerrg(cv::Size(pFrameInfo->nWidth, pFrameInfo->nHeight), CV_8UC1, pData);
        cv::Mat result;
        cv::cvtColor(img_bayerrg, result, cv::COLOR_BayerRG2RGB);
        {
            std::lock_guard<std::mutex> _lock(_mutex);
            std::swap(_frame, result);
        }
    }
}

std::unique_ptr<hk_camera> hsm::make_hk_camera(std::shared_ptr<config> _config_data)
{
    throw_if(!_config_data, "配置文件的指针为空\n");
    // 根据json解析参数

    // 创建相机实例
    auto hk_camera_ptr = std::make_unique<hk_camera>();

    // 相机初始化
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举设备
    // enum device
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("V_CC_EnumDevices fail! nRet {}\n"), nRet));
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                throw_runtime(fmt::format(FMT_COMPILE("找到的设备报错，对应设备号为 {}\n"), i));
            }
        }
    }
    else
    {
        throw_runtime("Find No Devices!");
    }

    unsigned int nIndex = _config_data->device_id;
    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&(hk_camera_ptr->handle), stDeviceList.pDeviceInfo[nIndex]);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_CreateHandle fail! nRet {}\n"), nRet));
    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(hk_camera_ptr->handle);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_OpenDevice fail! nRet {}\n"), nRet));
    // 设置触发模式为off
    // set trigger mode as off
    nRet = MV_CC_SetEnumValue(hk_camera_ptr->handle, "TriggerMode", 0);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_SetTriggerMode fail! nRet {}\n"), nRet));

    // ch：设置曝光时间，图像的长宽,和所取图像的偏移
    // 注意，这里对offset的值应当提前归零，防止出现长度溢出问题
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "OffsetX", 0);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置OffsetX错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "OffsetY", 0);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置OffsetY错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetFloatValue(hk_camera_ptr->handle, "ExposureTime", _config_data->exposure);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置曝光错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "Width", _config_data->width);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置Width错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "Height", _config_data->height);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置Height错误,错误码: {}\n"), nRet));
    // 这里设置相机偏移两遍是因为有的时候上次窗长宽与偏移相冲突
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "OffsetX", _config_data->offset_x);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置OffsetX错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetIntValue(hk_camera_ptr->handle, "OffsetY", _config_data->offset_y);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置OffsetY错误,错误码: {}\n"), nRet));

    // RGB格式0x02180014
    // bayerRG格式0x01080009
    nRet = MV_CC_SetEnumValue(hk_camera_ptr->handle, "PixelFormat", 0x01080009);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置传输图像格式错误,错误码: {}\n"), nRet));
    nRet = MV_CC_SetFloatValue(hk_camera_ptr->handle, "Gain",  _config_data->gain);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("设置增益错误,错误码: {}\n"), nRet));
    // 注册抓图回调
    // register image callback
    nRet = MV_CC_RegisterImageCallBackEx(hk_camera_ptr->handle, image_callback, hk_camera_ptr->handle);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_RegisterImageCallBackEx fail! nRet {}\n"), nRet));
    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(hk_camera_ptr->handle);
    throw_if(MV_OK != nRet, fmt::format(FMT_COMPILE("MV_CC_StartGrabbing fail! nRet {}\n"), nRet));
    fmt::print("hik init\n");
    return hk_camera_ptr;
}

hk_camera::~hk_camera()
{
    // 停止取流
    // end grab image
    int nRet = MV_CC_StopGrabbing(this->handle);
    if (MV_OK != nRet)
    {
        fmt::print(FMT_COMPILE("MV_CC_StopGrabbing fail! nRet [{}]\n"), nRet);
        exit(1);
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(this->handle);
    if (MV_OK != nRet)
    {
        fmt::print(FMT_COMPILE("MV_CC_CloseDevice fail! nRet [{}]\n"), nRet);
        exit(1);
    }

    // 销毁句柄
    // destroy handle
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
}
