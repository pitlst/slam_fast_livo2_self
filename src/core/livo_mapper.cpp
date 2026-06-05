#include <thread>
#include <chrono>

#include "sensor/hk_camera.hpp"
#include "sensor/mid360_lidar.hpp"
#include "core/livo_mapper.hpp"

using namespace hsm;

livo_mapper::livo_mapper(std::unique_ptr<hk_camera> camera, std::unique_ptr<livox_lidar> lidar)
    : camera(std::move(camera)), lidar(std::move(lidar))
{
    this->lidar_measures = std::make_shared<lidar_measure_group>();

    this->p_imu   = std::make_unique<imu_process>();
    this->p_point = std::make_unique<point_preprocess>();

    this->image_sync = std::make_unique<time_sync>();
    this->point_sync = std::make_unique<time_sync>();
    this->imu_sync   = std::make_unique<time_sync>();
}

void livo_mapper::run()
{
    while (true)
    {
        this->drain_sensor_queues();
        if (! this->sync_packages())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        // processImu();
        // stateEstimationAndMapping();
    }
}

bool livo_mapper::sync_packages()
{
    // --- 基本检查：三个传感器都必须有数据 ---
    if (this->lidar_imu_buffer.empty())
    {
        return false;
    }
    if (this->lidar_point_buffer.empty())
    {
        return false;
    }
    if (this->frame_buffer.empty())
    {
        return false;
    }

    EKF_STATE last_flg = this->lidar_measures->lio_vio_flg;

    switch (last_flg)
    {
        // 状态 WAIT / VIO：准备 LIO 数据
        case EKF_STATE::WAIT:
        case EKF_STATE::VIO:
        {
            // 取图像捕获时间（曝光时间补偿暂设为0）
            double img_capture_time = this->frame_buffer.front().timestamp;
            // 首次初始化 last_lio_update_time
            if (this->lidar_measures->last_lio_update_time < 0.0)
            {
                this->lidar_measures->last_lio_update_time = this->lidar_point_buffer.front().timestamp;
            }
            // 获取当前最新 LiDAR 和 IMU 时间
            // Mid-360 每包约含 ~100 个点，最后一点的时间戳就是包的结束时间
            // 实际上 = 包首点时间 + 包内时间跨度
            double lid_newest_time = this->lidar_point_buffer.back().timestamp + this->lidar_point_buffer.back().value.points.back().curvature - this->lidar_point_buffer.back().timestamp;
            // 简化：包最后一点 = lidar_raw_buffer.back().points.back().timestamp
            double imu_newest_time = this->frame_buffer.back().timestamp;

            // 丢弃过期图像（早于上次处理的）
            if (img_capture_time < this->lidar_measures->last_lio_update_time + 1e-6)
            {
                this->frame_buffer.pop_front();
                return false;
            }
            // 等待数据追上图像时间
            if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
            {
                return false;
            }
            // 创建 MeasureGroup，收集 IMU
            measure_group m;
            m.lio_time = img_capture_time;
            while (! this->lidar_imu_buffer.empty())
            {
                if (this->lidar_imu_buffer.front().timestamp > m.lio_time)
                {
                    break;
                }
                // 只保留 last_lio_update_time 之后的新 IMU
                if (this->lidar_imu_buffer.front().timestamp > this->lidar_measures->last_lio_update_time)
                {
                    m.imu.emplace_back(this->lidar_imu_buffer.front());
                }
                this->lidar_imu_buffer.pop_front();
            }

            // 搬运上一轮留下的 next → cur
            *(this->lidar_measures->pcl_proc_cur) = *(this->lidar_measures->pcl_proc_next);
            this->lidar_measures->pcl_proc_next->clear();

            // 累积并切分 LiDAR 点云
            while (! this->lidar_point_buffer.empty())
            {
                // 如果这个包的起始时间已经超过图像时刻，留着下次处理
                if (this->lidar_point_buffer.front().timestamp > img_capture_time)
                {
                    break;
                }
                auto& pcl = this->lidar_point_buffer.front().value.points;

                for (size_t i = 0; i < pcl.size(); i++)
                {
                    const auto& pt = pcl[i];
                    if (pt.curvature < img_capture_time)
                    {
                        // 在图像时刻之前 → 本次 LIO 处理
                        this->lidar_measures->pcl_proc_cur->points.emplace_back(pt);
                    }
                    else
                    {
                        // 在图像时刻之后 → 留到下次 LIO
                        this->lidar_measures->pcl_proc_next->points.emplace_back(pt);
                    }
                }
                this->lidar_point_buffer.pop_front();
            }

            // 设置状态为 LIO
            this->lidar_measures->measures.emplace_back(m);
            this->lidar_measures->lio_vio_flg = EKF_STATE::LIO;
            return true;
        }
        // 状态 LIO：准备 VIO 数据
        case EKF_STATE::LIO:
        {
            double img_capture_time = this->frame_buffer.front().timestamp;

            this->lidar_measures->lio_vio_flg = EKF_STATE::VIO;
            this->lidar_measures->measures.clear();

            measure_group m;
            m.vio_time = img_capture_time;
            m.lio_time = this->lidar_measures->last_lio_update_time;
            m.img      = this->frame_buffer.front().value;
            this->frame_buffer.pop_front();

            this->lidar_measures->measures.emplace_back(m);
            // 允许新一轮 LiDAR 累积
            this->lidar_pushed = false;
            return true;
        }
        default:
            return false;
    }
}

void livo_mapper::drain_sensor_queues()
{
    // --- 拉 IMU 数据 ---
    timestamped<imu_data> imu_item;
    while (this->lidar->get_imu(imu_item))
    {
        process_timestamped<raw_imu_data> rid;
        rid.timestamp = this->imu_sync->update(imu_item.device_timestamp, imu_item.host_timestamp);
        // LivoxLidarImuRawPoint → gyro(rad/s), accel(m/s²)
        rid.value.gyro = Eigen::Vector3d(
            imu_item.value.gyro_x,
            imu_item.value.gyro_y,
            imu_item.value.gyro_z);
        rid.value.accel = Eigen::Vector3d(
            imu_item.value.acc_x,
            imu_item.value.acc_y,
            imu_item.value.acc_z);
        this->lidar_imu_buffer.emplace_back(std::move(rid));
    }

    // --- 拉 LiDAR 点云 ---
    timestamped<point_data> pt_item;
    while (lidar->get_points(pt_item))
    {
        process_timestamped<point_data> temp_data;
        temp_data.value     = std::move(pt_item.value);
        temp_data.timestamp = this->point_sync->update(pt_item.device_timestamp, pt_item.host_timestamp);

        pcl::PointCloud<pcl::PointXYZINormal> pc;
        this->p_point->process(temp_data, pc);
        if (pc.empty())
        {
            continue;
        }
        process_timestamped<pcl::PointCloud<pcl::PointXYZINormal>> res;
        res.timestamp = temp_data.timestamp;
        res.value     = std::move(pc);
        this->lidar_point_buffer.emplace_back(std::move(res));
    }

    // --- 拉相机图像 ---
    timestamped<cv::Mat> img_item;
    while (camera->get(img_item))
    {
        if (img_item.value.empty())
        {
            break;
        }
        process_timestamped<cv::Mat> res;
        res.timestamp = this->image_sync->update(img_item.device_timestamp, img_item.host_timestamp);
        res.value     = std::move(img_item.value);
        this->frame_buffer.emplace_back(std::move(res));
    }
}