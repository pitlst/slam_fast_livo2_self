#ifndef HSM_CORE_LIVMAPPER_H
#define HSM_CORE_LIVMAPPER_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "common/struct.hpp"
namespace hsm
{
    struct livo_mapper
    {
    public:
        livo_mapper();

        // 循环入口
        void run();
        // 同步驱动数据
        bool sync_packages();

    public:
        lidar_measure_group lidar_measures;

    private:
        // 根据首帧计算时间零点
        void handle_first_frame();

    private:
        double _first_lidar_time = 0.0;
    };
} // namespace hsm

#endif