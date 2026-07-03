#ifndef HSM_CORE_POINT_PREPROCESS_H
#define HSM_CORE_POINT_PREPROCESS_H

#include <cstdint>
#include <array>
#include <memory>

#include "Eigen/Core"
#include "phmap.hpp"

namespace hsm
{
    struct point_with_var
    {
        Eigen::Vector3d point_b; // point in the lidar body frame
        Eigen::Vector3d point_w; // point in the world frame
        Eigen::Matrix3d body_var;
        Eigen::Matrix3d var;
        Eigen::Vector3d normal;

        point_with_var()
            : point_b(Eigen::Vector3d::Zero()),
              point_w(Eigen::Vector3d::Zero()),
              body_var(Eigen::Matrix3d::Zero()),
              var(Eigen::Matrix3d::Zero()),
              normal(Eigen::Vector3d::Zero()) {};
    };

    // 体素地图索引
    struct voxel_location
    {
        int64_t x;
        int64_t y;
        int64_t z;

        auto operator<=>(voxel_location const&) const = default;
    };

    struct voxel_plane
    {
        Eigen::Vector3d             center_;
        Eigen::Vector3d             normal_;
        Eigen::Vector3d             y_normal_;
        Eigen::Vector3d             x_normal_;
        Eigen::Matrix3d             covariance_;
        Eigen::Matrix<double, 6, 6> plane_var_;

        float radius_          = 0;
        float min_eigen_value_ = 1;
        float mid_eigen_value_ = 1;
        float max_eigen_value_ = 1;
        float d_               = 0;
        int   points_size_     = 0;
        bool  is_plane_        = false;
        bool  is_init_         = false;
        int   id_              = 0;
        bool  is_update_       = false;

        voxel_plane()
            : plane_var_(Eigen::Matrix<double, 6, 6>::Zero()),
              covariance_(Eigen::Matrix3d::Zero()),
              center_(Eigen::Vector3d::Zero()),
              normal_(Eigen::Vector3d::Zero())
        {}
    };

    // 自适应八叉树
    struct voxel_octo_tree
    {
        voxel_octo_tree() = default;

        std::vector<point_with_var>                     temp_points_;
        std::unique_ptr<voxel_plane>                    plane_ptr_;
        int                                             layer_;
        int                                             octo_state_; // 0 is end of tree, 1 is not
        std::array<std::unique_ptr<voxel_octo_tree>, 8> leaves;
        double                                          voxel_center_[3]; // x, y, z
        std::vector<int>                                layer_init_num_;
        float                                           quater_length_;
        float                                           planer_threshold_;
        int                                             points_size_threshold_;
        int                                             update_size_threshold_;
        int                                             max_points_num_;
        int                                             max_layer_;
        int                                             new_points_;
        bool                                            init_octo_;
        bool                                            update_enable_;
    };

    struct voxel_mapper
    {
    public:
        voxel_mapper() = default;

    public:
        gtl::flat_hash_map<voxel_location, std::unique_ptr<voxel_octo_tree>> voxel_map_;
    };
} // namespace hsm

#endif