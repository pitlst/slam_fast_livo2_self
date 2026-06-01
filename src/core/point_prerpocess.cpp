#include "core/point_prerpocess.hpp"

using namespace hsm;

void point_prerpocess::process(
    const point_data&                      raw_points,
    double                                 timestamp_s,
    pcl::PointCloud<pcl::PointXYZINormal>& out)
{
    // time_interval 单位 0.1µs → 秒
    double dt     = raw_points.time_interval * 0.1e-6;
    size_t plsize = raw_points.points.size();

    for (size_t i = 0; i < plsize; ++i)
    {
        const auto& raw = raw_points.points[i];

        // tag 过滤: 只保留有效点
        if ((raw.tag & 0x30) != 0x10) continue;

        pcl::PointXYZINormal pt;

        // mm → m
        pt.x = static_cast<double>(raw.x) / 1000.0;
        pt.y = static_cast<double>(raw.y) / 1000.0;
        pt.z = static_cast<double>(raw.z) / 1000.0;

        // 强度
        pt.intensity = raw.reflectivity;

        // 绝对时间戳（秒）= packet起始时间 + 点内偏移
        pt.curvature = timestamp_s + static_cast<double>(i) * dt;

        // 盲区过滤: 3D 距离² < blind_sqr 则丢弃
        double dist_sqr = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (dist_sqr < blind_sqr) continue;

        out.push_back(pt);
    }
}