#ifndef HSM_CORE_POINT_PREPROCESS_H
#define HSM_CORE_POINT_PREPROCESS_H

#include <cstdint>

#include "Eigen/Core"
#include "phmap.hpp"

namespace hsm
{
    typedef struct pointWithVar
    {
        Eigen::Vector3d point_b;     // point in the lidar body frame
        Eigen::Vector3d point_i;     // point in the imu body frame
        Eigen::Vector3d point_w;     // point in the world frame
        Eigen::Matrix3d var_nostate; // the var removed the state covarience
        Eigen::Matrix3d body_var;
        Eigen::Matrix3d var;
        Eigen::Matrix3d point_crossmat;
        Eigen::Vector3d normal;
        pointWithVar()
        {
            var_nostate    = Eigen::Matrix3d::Zero();
            var            = Eigen::Matrix3d::Zero();
            body_var       = Eigen::Matrix3d::Zero();
            point_crossmat = Eigen::Matrix3d::Zero();
            point_b        = Eigen::Vector3d::Zero();
            point_i        = Eigen::Vector3d::Zero();
            point_w        = Eigen::Vector3d::Zero();
            normal         = Eigen::Vector3d::Zero();
        };
    } pointWithVar;

    // 体素地图索引
    struct voxel_location
    {
        int64_t x;
        int64_t y;
        int64_t z;

        auto operator<=>(const voxel_location&) const = default;
    };

    // 自适应八叉树
    struct voxel_octo_tree
    {
        voxel_octo_tree() = default;

        std::vector<pointWithVar> temp_points_;
        VoxelPlane*               plane_ptr_;
        int                       layer_;
        int                       octo_state_; // 0 is end of tree, 1 is not
        VoxelOctoTree*            leaves_[8];
        double                    voxel_center_[3]; // x, y, z
        std::vector<int>          layer_init_num_;
        float                     quater_length_;
        float                     planer_threshold_;
        int                       points_size_threshold_;
        int                       update_size_threshold_;
        int                       max_points_num_;
        int                       max_layer_;
        int                       new_points_;
        bool                      init_octo_;
        bool                      update_enable_;
    };

    struct voxel_mapper
    {
    public:
        voxel_mapper() = default;

    public:
        gtl::flat_hash_map<voxel_location, VoxelOctoTree*> voxel_map_;
    };
} // namespace hsm

#endif