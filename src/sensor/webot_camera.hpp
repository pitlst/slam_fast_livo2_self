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


#pragma pack(push, 1)
struct CameraPacketHeader
{
    double   timestamp;      // 8 bytes  —— robot.getTime()
    uint32_t width;          // 4 bytes  —— img.shape[1] (cols)
    uint32_t height;         // 4 bytes  —— img.shape[0] (rows)
    uint32_t channels;       // 4 bytes  —— img.shape[2]
    uint32_t compressed_len; // 4 bytes  —— len(compressed)
    uint32_t raw_len;        // 4 bytes  —— len(raw_bytes)
};
#pragma pack(pop)

static_assert(sizeof(CameraPacketHeader) == 28, "CameraPacketHeader 必须是 28 字节，与 Python struct 对齐");

namespace hsm
{

    struct compressor
    {
    private:
        ZSTD_CCtx* ctx_;

    public:
        compressor();
        ~compressor();

        // 禁止拷贝
        compressor(compressor const&)            = delete;
        compressor& operator=(compressor const&) = delete;

        // 允许移动
        compressor(compressor&& other) noexcept;
        compressor& operator=(compressor&& other) noexcept;

        // 压缩到外部已分配缓冲区，返回实际写入字节数
        size_t compress(double timestamp, cv::Mat const& img, void* dst, size_t dstCapacity, int level = 3);
        std::vector<uint8_t> compress(double timestamp, cv::Mat const& img, int level = 3);
    };

    class decompressor
    {
    private:
        ZSTD_DCtx* ctx_;

    public:
        decompressor();
        ~decompressor();

        // 禁止拷贝
        decompressor(decompressor const&)            = delete;
        decompressor& operator=(decompressor const&) = delete;

        // 允许移动
        decompressor(decompressor&& other) noexcept;

        decompressor& operator=(decompressor&& other) noexcept;

        // 解压缩到外部 cv::Mat，返回 timestamp
        std::pair<double, cv::Mat> decompress(void const* src, size_t srcSize);
        std::pair<double, cv::Mat> decompress(std::vector<uint8_t> const& src);
    };


    struct webot_camera
    {
        friend std::shared_ptr<webot_camera> make_webot_camera(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url);

    public:
        ~webot_camera();

        bool get(timestamped<cv::Mat>& out);

        std::jthread                    back_thread;
        std::atomic<bool>               running_label = false;
    };

    // 工厂函数，所有使用zmq的接收器共用一个上下文
    std::shared_ptr<webot_camera> make_webot_camera(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url = "tcp://localhost:5555");
} // namespace hsm

#endif
