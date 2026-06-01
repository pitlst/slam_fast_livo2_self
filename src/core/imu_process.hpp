#ifndef HSM_CORE_IMU_PROCESS_H
#define HSM_CORE_IMU_PROCESS_H

#include "Eigen/Eigen"

namespace hsm
{
    struct imu_process
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        imu_process()  = default;
        ~imu_process() = default;

        void process();

    private:
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