#include <iostream>
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#include "config/config.hpp"
#include "sensor/hk_camera.hpp"

extern cv::Mat    _frame;
extern std::mutex _mutex;

int main()
{
    auto _config_data = std::make_shared<hsm::config>();
    auto camera       = hsm::make_hk_camera(_config_data);

    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat frame_copy;
    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (! _frame.empty())
            {
                frame_copy = _frame.clone(); // clone 是必须的，因为下一帧会覆盖 _frame
            }
        }

        if (! frame_copy.empty())
        {
            cv::imshow("Camera", frame_copy);
        }

        if (cv::waitKey(1) == 'k')
        {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}