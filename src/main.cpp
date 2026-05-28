#include <cmath>
#include <algorithm>
#include <exception>

#include "Eigen/Core"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#include "config/config.hpp"
#include "sensor/hk_camera.hpp"
#include "sensor/mid360_lidar.hpp"

namespace
{
    // Livox mm → 米, 反射率 → 伪彩色
    std::pair<Eigen::Vector3d, Eigen::Vector3d> convert_point(
        const LivoxLidarCartesianHighRawPoint& raw)
    {
        Eigen::Vector3d pt(
            static_cast<double>(raw.x) / 1000.0,
            static_cast<double>(raw.y) / 1000.0,
            static_cast<double>(raw.z) / 1000.0);
        float v = std::clamp(raw.reflectivity / 255.0f, 0.0f, 1.0f);
        float r = std::clamp(1.5f - std::abs(1.0f - v * 4.0f), 0.0f, 1.0f);
        float g = std::clamp(1.5f - std::abs(1.0f - v * 4.0f + 2.0f), 0.0f, 1.0f);
        float b = std::clamp(1.5f - std::abs(1.0f - v * 4.0f - 2.0f), 0.0f, 1.0f);
        return {pt, Eigen::Vector3d(r, g, b)};
    }

    // 点云 3D → 2D 投影绘制 (XY 俯视图)
    void draw_pointcloud(cv::Mat& canvas,
                         const std::vector<Eigen::Vector3d>& pts,
                         const std::vector<Eigen::Vector3d>& colors,
                         double scale_m_to_px = 80.0)
    {
        canvas.setTo(cv::Scalar(25, 25, 38));
        int cx = canvas.cols / 2;
        int cy = canvas.rows / 2;
        for (size_t i = 0; i < pts.size(); ++i)
        {
            int px = cx + static_cast<int>(pts[i].x() * scale_m_to_px);
            int py = cy - static_cast<int>(pts[i].y() * scale_m_to_px);
            if (px < 0 || px >= canvas.cols || py < 0 || py >= canvas.rows) continue;
            auto& c = colors[i];
            canvas.at<cv::Vec3b>(py, px) = cv::Vec3b(
                static_cast<uchar>(c.z() * 255),
                static_cast<uchar>(c.y() * 255),
                static_cast<uchar>(c.x() * 255));
        }
    }
} // namespace

int main()
{
    try
    {
        auto camera = hsm::make_hk_camera();
        auto lidar  = hsm::make_livox_lidar();

        cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("LiDAR",  cv::WINDOW_AUTOSIZE);

        cv::Mat lidar_canvas(720, 720, CV_8UC3);

        std::vector<Eigen::Vector3d> point_buf;
        std::vector<Eigen::Vector3d> color_buf;
        point_buf.reserve(65536);
        color_buf.reserve(65536);

        while (true)
        {
            point_buf.clear();
            color_buf.clear();
            for (;;)
            {
                auto item = lidar->get_points();
                if (item.payload.empty()) break;
                for (auto& raw : item.payload)
                {
                    auto [pt, color] = convert_point(raw);
                    point_buf.push_back(pt);
                    color_buf.push_back(color);
                }
            }
            if (! point_buf.empty())
            {
                draw_pointcloud(lidar_canvas, point_buf, color_buf);
                cv::imshow("LiDAR", lidar_canvas);
            }

            auto frame_item = camera->get();
            if (! frame_item.payload.empty())
            {
                cv::imshow("Camera", frame_item.payload);
            }

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
