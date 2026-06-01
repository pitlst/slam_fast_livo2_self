#ifndef HSM_CORE_POINT_PREPROCESS_H
#define HSM_CORE_POINT_PREPROCESS_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "common/struct.hpp"

namespace hsm
{
    struct point_prerpocess
    {
    public:
        point_prerpocess(double blind = 0.01);

        void process(const process_timestamped<point_data>& raw_points, pcl::PointCloud<pcl::PointXYZINormal>& out);

    private:
        // 盲区阈值，距离雷达原点小于 blind 米的点（如外壳反射、近处干扰）直接丢弃
        double blind_sqr;
    };

} // namespace hsm

#endif