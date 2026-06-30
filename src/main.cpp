#include <cmath>
#include <algorithm>
#include <exception>

#include "Eigen/Core"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#include "rerun.hpp"

#include "sensor/webot_lidar.hpp"
#include "sensor/webot_camera.hpp"

#include "sensor/hk_camera.hpp"
#include "sensor/mid360_lidar.hpp"


int main()
{
    try
    {
        const auto rec = rerun::RecordingStream("slam_fast_livo2");
        rec.connect_grpc("rerun+http://127.0.0.1:9876/proxy").exit_on_failure();


        auto camera = hsm::make_webot_camera();
        auto lidar  = hsm::make_webot_lidar();

        // auto camera = hsm::make_hk_camera();
        // auto lidar  = hsm::make_livox_lidar();

        cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

        std::vector<Eigen::Vector3d> point_buf;
        std::vector<Eigen::Vector3d> color_buf;
        point_buf.reserve(65536);
        color_buf.reserve(65536);

        uint64_t frame_idx = 0;

        while (true)
        {
            point_buf.clear();
            color_buf.clear();
            for (;;)
            {
                hsm::timestamped<hsm::point_data> item;
                if (! lidar->get_points(item)) break;

                fmt::print(FMT_COMPILE("[lidar] {} {}\n"), item.device_timestamp, item.host_timestamp);

                for (auto& raw : item.value.points)
                {
                    Eigen::Vector3d pt(
                        static_cast<double>(raw.x) / 1000.0,
                        static_cast<double>(raw.y) / 1000.0,
                        static_cast<double>(raw.z) / 1000.0);
                    float v = std::clamp(raw.reflectivity / 255.0f, 0.0f, 1.0f);
                    float r = std::clamp(1.5f - std::abs(1.0f - v * 4.0f), 0.0f, 1.0f);
                    float g = std::clamp(1.5f - std::abs(1.0f - v * 4.0f + 2.0f), 0.0f, 1.0f);
                    float b = std::clamp(1.5f - std::abs(1.0f - v * 4.0f - 2.0f), 0.0f, 1.0f);
                    point_buf.push_back(pt);
                    color_buf.push_back(Eigen::Vector3d(r, g, b));
                }
            }
            if (! point_buf.empty())
            {
                std::vector<rerun::Position3D> positions;
                std::vector<rerun::Color>      colors;
                positions.reserve(point_buf.size());
                colors.reserve(point_buf.size());
                for (size_t i = 0; i < point_buf.size(); ++i)
                {
                    const auto& pt    = point_buf[i];
                    const auto& color = color_buf[i];
                    positions.emplace_back(
                        static_cast<float>(pt.x()),
                        static_cast<float>(pt.y()),
                        static_cast<float>(pt.z()));
                    colors.emplace_back(
                        static_cast<uint8_t>(color.x() * 255.0f),
                        static_cast<uint8_t>(color.y() * 255.0f),
                        static_cast<uint8_t>(color.z() * 255.0f));
                }
                rec.set_time_sequence("frame", frame_idx);
                rec.log("lidar/points", rerun::Points3D(positions).with_colors(colors));
            }

            hsm::timestamped<cv::Mat> frame_item;
            if (camera->get(frame_item))
            {
                fmt::print(FMT_COMPILE("[camera] {} {}\n"), frame_item.device_timestamp, frame_item.host_timestamp);
                cv::imshow("Camera", frame_item.value);
            }

            ++frame_idx;

            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) break;
        }
    }
    catch (const std::exception& e)
    {
        fmt::print(stderr, "[FATAL] {}\n", e.what());
        cv::destroyAllWindows();
        return -1;
    }

    cv::destroyAllWindows();
    return 0;
}
