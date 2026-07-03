#ifndef HSM_ZSTDWARPPER_H
#define HSM_ZSTDWARPPER_H

#include <stdexcept>
#include <vector>
#include <string>
#include <cstdint>
#include <cstddef>

#include "opencv2/opencv.hpp"
#include "zstd.h"

#include "common/struct.hpp"
#include "common/enhanced_exception.hpp"

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

namespace hsm::zstd
{
    class compressor
    {
        ZSTD_CCtx* ctx_;

    public:
        compressor(): ctx_(ZSTD_createCCtx())
        {
            throw_if(! (this->ctx_), "创建ZSTD_CCtx失败");
        }

        ~compressor()
        {
            if (this->ctx_)
            {
                ZSTD_freeCCtx(ctx_);
            }
        }

        // 禁止拷贝
        compressor(compressor const&)            = delete;
        compressor& operator=(compressor const&) = delete;

        // 允许移动
        compressor(compressor&& other) noexcept: ctx_(other.ctx_)
        {
            other.ctx_ = nullptr;
        }
        compressor& operator=(compressor&& other) noexcept
        {
            if (this != &other)
            {
                if (ctx_)
                {
                    ZSTD_freeCCtx(ctx_);
                }
                ctx_       = other.ctx_;
                other.ctx_ = nullptr;
            }
            return *this;
        }

        // 压缩到外部已分配缓冲区，返回实际写入字节数
        size_t compress(double timestamp, cv::Mat const& img, void* dst, size_t dstCapacity, int level = 3)
        {
            throw_if(img.empty(), "cv::Mat 为空，无法压缩");

            // 非连续 Mat（ROI 等）先 clone 成连续内存
            cv::Mat const cont = img.isContinuous() ? img : img.clone();

            uint32_t const w = static_cast<uint32_t>(cont.cols);
            uint32_t const h = static_cast<uint32_t>(cont.rows);
            uint32_t const c = static_cast<uint32_t>(cont.channels());

            size_t const raw_len = cont.total() * cont.elemSize();
            throw_if(raw_len > std::numeric_limits<uint32_t>::max(), "图像数据超过 4GB，超出 Header 支持范围");

            // 直接压缩到 Header 后面的位置，避免额外拷贝
            size_t const bound = ZSTD_compressBound(raw_len);
            throw_if(sizeof(CameraPacketHeader) + bound > dstCapacity, "目标缓冲区容量不足");

            size_t const compressed_len = ZSTD_compressCCtx(
                ctx_,
                static_cast<uint8_t*>(dst) + sizeof(CameraPacketHeader),
                dstCapacity - sizeof(CameraPacketHeader),
                cont.data,
                raw_len,
                level);
            throw_if(ZSTD_isError(compressed_len), ZSTD_getErrorName(compressed_len));
            throw_if(compressed_len > std::numeric_limits<uint32_t>::max(), "压缩后数据超过 4GB");

            // 填充 Header
            CameraPacketHeader header;
            header.timestamp      = timestamp;
            header.width          = w;
            header.height         = h;
            header.channels       = c;
            header.compressed_len = static_cast<uint32_t>(compressed_len);
            header.raw_len        = static_cast<uint32_t>(raw_len);

            std::memcpy(dst, &header, sizeof(header));
            return sizeof(CameraPacketHeader) + compressed_len;
        }

        std::vector<uint8_t> compress(double timestamp, cv::Mat const& img, int level = 3)
        {
            size_t const         raw_len     = img.total() * img.elemSize();
            size_t const         dstCapacity = sizeof(CameraPacketHeader) + ZSTD_compressBound(raw_len);
            std::vector<uint8_t> dst(dstCapacity);
            size_t const         written = compress(timestamp, img, dst.data(), dstCapacity, level);
            dst.resize(written);
            return dst;
        }
    };

    class decompressor
    {
        ZSTD_DCtx* ctx_;

    public:
        decompressor(): ctx_(ZSTD_createDCtx())
        {
            throw_if(! (this->ctx_), "创建ZSTD_DCtx失败");
        }

        ~decompressor()
        {
            if (this->ctx_)
            {
                ZSTD_freeDCtx(ctx_);
            }
        }

        // 禁止拷贝
        decompressor(decompressor const&)            = delete;
        decompressor& operator=(decompressor const&) = delete;

        // 允许移动
        decompressor(decompressor&& other) noexcept: ctx_(other.ctx_)
        {
            other.ctx_ = nullptr;
        }

        decompressor& operator=(decompressor&& other) noexcept
        {
            if (this != &other)
            {
                if (ctx_) ZSTD_freeDCtx(ctx_);
                ctx_       = other.ctx_;
                other.ctx_ = nullptr;
            }
            return *this;
        }

        // 解压缩到外部 cv::Mat，返回 timestamp
        std::pair<double, cv::Mat> decompress(void const* src, size_t srcSize)
        {
            throw_if(srcSize < sizeof(CameraPacketHeader), "数据小于 Header 大小，数据损坏");

            CameraPacketHeader header;
            std::memcpy(&header, src, sizeof(header));

            size_t const expected = sizeof(CameraPacketHeader) + header.compressed_len;
            throw_if(srcSize < expected, "数据长度不足，compressed_len 超出实际数据");

            // 解压图像裸数据
            std::vector<uint8_t> raw(header.raw_len);
            size_t const         written = ZSTD_decompressDCtx(
                ctx_,
                raw.data(), header.raw_len,
                static_cast<uint8_t const*>(src) + sizeof(CameraPacketHeader),
                header.compressed_len);
            throw_if(ZSTD_isError(written), ZSTD_getErrorName(written));
            throw_if(written != header.raw_len, "解压后大小与 raw_len 声明不一致");

            // 根据 channels 推断 OpenCV 类型（Python 端为 uint8）
            int cv_type = CV_8UC1;
            if (header.channels == 1)
            {
                cv_type = CV_8UC1;
            }
            else if (header.channels == 3)
            {
                cv_type = CV_8UC3;
            }
            else if (header.channels == 4)
            {
                cv_type = CV_8UC4;
            }
            else
            {
                throw_if(true, "不支持的通道数，仅支持 1/3/4");
            }
            cv::Mat dst;
            dst.create(static_cast<int>(header.height), static_cast<int>(header.width), cv_type);
            throw_if(static_cast<size_t>(dst.total() * dst.elemSize()) != header.raw_len, "cv::Mat 尺寸与 Header 数据大小不匹配");

            std::memcpy(dst.data, raw.data(), written);
            return std::make_pair(header.timestamp, dst);
        }

        std::pair<double, cv::Mat> decompress(std::vector<uint8_t> const& src)
        {
            return decompress(src.data(), src.size());
        }
    };
} // namespace hsm::zstd

#endif