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
        point_prerpocess()  = default;
        ~point_prerpocess() = default;

        void process(const point_data& raw_points, double timestamp_s, pcl::PointCloud<pcl::PointXYZINormal>& out);

    private:
        // 盲区阈值，距离雷达原点小于 blind 米的点（如外壳反射、近处干扰）直接丢弃
        const double blind_sqr = 0.01 * 0.01;
    };

} // namespace hsm

#endif