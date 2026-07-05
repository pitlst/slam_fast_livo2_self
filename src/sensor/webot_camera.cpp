#include "zstd.h"
#include "sensor/webot_camera.hpp"
#include "sensor/zstd_warpper.hpp"
#include "common/enhanced_exception.hpp"

using namespace hsm;

static frame_queue _camera_queue(K_BUFFER_CAPACITY);

compressor::compressor(): ctx_(ZSTD_createCCtx())
{
    throw_if(! (this->ctx_), "创建ZSTD_CCtx失败");
}

compressor::~compressor()
{
    if (this->ctx_)
    {
        ZSTD_freeCCtx(ctx_);
    }
}

compressor::compressor(compressor&& other) noexcept
    : ctx_(other.ctx_)
{
    other.ctx_ = nullptr;
}

compressor& compressor::operator=(compressor&& other) noexcept
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

size_t compressor::compress(double timestamp, cv::Mat const& img, void* dst, size_t dstCapacity, int level = 3)
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

std::vector<uint8_t> compressor::compress(double timestamp, cv::Mat const& img, int level = 3)
{
    size_t const         raw_len     = img.total() * img.elemSize();
    size_t const         dstCapacity = sizeof(CameraPacketHeader) + ZSTD_compressBound(raw_len);
    std::vector<uint8_t> dst(dstCapacity);
    size_t const         written = this->compress(timestamp, img, dst.data(), dstCapacity, level);
    dst.resize(written);
    return dst;
}

decompressor::decompressor(): ctx_(ZSTD_createDCtx())
{
    throw_if(! (this->ctx_), "创建ZSTD_DCtx失败");
}

decompressor::~decompressor()
{
    if (this->ctx_)
    {
        ZSTD_freeDCtx(ctx_);
    }
}

decompressor::decompressor(decompressor&& other) noexcept
    : ctx_(other.ctx_)
{
    other.ctx_ = nullptr;
}

decompressor& decompressor::operator=(decompressor&& other) noexcept
{
    if (this != &other)
    {
        if (ctx_) ZSTD_freeDCtx(ctx_);
        ctx_       = other.ctx_;
        other.ctx_ = nullptr;
    }
    return *this;
}

std::pair<double, cv::Mat> decompressor::decompress(void const* src, size_t srcSize)
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

std::pair<double, cv::Mat> decompressor::decompress(std::vector<uint8_t> const& src)
{
    return decompress(src.data(), src.size());
}

std::shared_ptr<webot_camera> hsm::make_webot_camera(std::shared_ptr<zmq::context_t> conetxt, std::string const& connect_url = "tcp://localhost:5555")
{
    static bool is_init = false;
    throw_if(is_init, fmt::format(FMT_COMPILE("尝试重复初始化相机\n")));

    auto webots_camera_ptr = std::make_shared<webot_camera>();
    webots_camera_ptr->running_label.store(true);
    webots_camera_ptr->back_thread = std::jthread(
        [conetxt, connect_url, webots_camera_ptr]()
        {
            zstd::decompressor decomp;
            zmq::socket_t      socket(*conetxt, zmq::socket_type::sub);
            socket.connect(connect_url);
            socket.set(zmq::sockopt::subscribe, "camera");
            socket.set(zmq::sockopt::rcvtimeo, 100);
            fmt::print("[webot camera] ZMQ 已经连接到 {}\n", connect_url);

            while (webots_camera_ptr->running_label.load())
            {
                try
                {
                    zmq::message_t topic, payload;
                    std::ignore = socket.recv(topic, zmq::recv_flags::none);
                    std::ignore = socket.recv(payload, zmq::recv_flags::none);

                    uint64_t host_ts           = get_now_pc_time();
                    auto [timestamp_us, frame] = decomp.decompress(payload.data(), payload.size());
                    // webots仿真提返回的是基于秒的浮点数，这里转换为统一的纳秒
                    uint64_t device_timestamp = static_cast<uint64_t>(timestamp_us * 1000000000);
                    _camera_queue.try_enqueue(timestamped<cv::Mat> {device_timestamp, host_ts, frame});
                }
                catch (std::exception const& e)
                {
                    fmt::print("[webot camera] 图像获取线程发生错误：{}", e.what());
                }
            }
            socket.close();
            fmt::print("[webot camera] 图像获取线程退出 \n");
        });
    return webots_camera_ptr;
}

webot_camera::~webot_camera()
{
    this->running_label.store(false);
    fmt::print("[webot_camera] 已析构\n");
}

bool webot_camera::get(timestamped<cv::Mat>& out)
{
    return _camera_queue.try_dequeue(out);
}
