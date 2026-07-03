#include "zmq.hpp"

#include "common/zstd_warpper.hpp"

int main()
{
    zmq::context_t ctx(1);
    zmq::socket_t  socket(ctx, zmq::socket_type::sub);
    socket.connect("tcp://172.17.192.1:5555");
    socket.set(zmq::sockopt::subscribe, "camera");

    hsm::zstd::decompressor decomp;

    fmt::print("开始接收消息");

    while (true)
    {
        zmq::message_t topic, payload;
        socket.recv(topic, zmq::recv_flags::none);
        socket.recv(payload, zmq::recv_flags::none);

        cv::Mat img;
        double  ts = decomp.decompress(payload.data(), payload.size(), img);

        // img 现在是 BGR 格式，可直接用于 imshow 或 cv::imwrite
        cv::imshow("Received", img);
        auto key = cv::waitKey(1);
        if (key == 27)
        {
            break;
        }
    }
}