#include "core/imu_process.hpp"

using namespace hsm;

imu_process::imu_process(std::shared_ptr<lidar_measure_group> lidar_measures, std::shared_ptr<states_group> states)
    : lidar_measures(lidar_measures), states(states)
{
    this->results = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
}

