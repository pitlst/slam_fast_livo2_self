#include <iostream>
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#include "config/config.hpp"
#include "sensor/hk_camera.hpp"

int main()
{
    auto camera = hsm::make_hk_camera();

    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat frame_copy;
    while (true)
    {
        // uint64_t timestamp = 0;
        // frame_copy = camera->get(timestamp);

        // if (! frame_copy.empty())
        // {
        //     cv::imshow("Camera", frame_copy);
        // }

        // if (cv::waitKey(1) == 'k')
        // {
        //     break;
        // }
    }

    cv::destroyAllWindows();
    return 0;
}