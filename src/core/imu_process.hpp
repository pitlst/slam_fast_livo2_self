#ifndef HSM_CORE_IMU_PROCESS_H
#define HSM_CORE_IMU_PROCESS_H

#include <memory>

#include "Eigen/Eigen"

#include "common/struct.hpp"
namespace hsm
{
    struct imu_process
    {
    public:
        imu_process(std::shared_ptr<lidar_measure_group> lidar_measures, std::shared_ptr<states_group> states);

        void process();

    public:
        std::shared_ptr<lidar_measure_group> lidar_measures;
        std::shared_ptr<states_group>        states;

        double imu_mean_acc_norm;
        double cov_inv_expo;

        Eigen::Vector3d cov_acc;
        Eigen::Vector3d cov_gyr;
        Eigen::Vector3d cov_bias_gyr;
        Eigen::Vector3d cov_bias_acc;

        bool imu_time_init = false;
        bool imu_need_init = true;
    };

} // namespace hsm

#endif