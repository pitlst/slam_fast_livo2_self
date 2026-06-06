#ifndef HSM_SO3_MATH_H
#define HSM_SO3_MATH_H

#include <cmath>
#include <numbers>
#include <type_traits>

#include "Eigen/Core"

namespace hsm
{
    // // C++20 Concepts: 约束 Eigen 3维向量类型，避免错误推导
    // template<typename Derived>
    // concept EigenVector3Like = requires(const Eigen::MatrixBase<Derived>& v) {
    //     { v(0) } -> std::convertible_to<typename Derived::Scalar>;
    //     { v(1) } -> std::convertible_to<typename Derived::Scalar>;
    //     { v(2) } -> std::convertible_to<typename Derived::Scalar>;
    //     requires std::is_arithmetic_v<typename Derived::Scalar>;
    // } && (Derived::RowsAtCompileTime == 3 || Derived::RowsAtCompileTime == Eigen::Dynamic) && (Derived::ColsAtCompileTime == 1 || Derived::ColsAtCompileTime == Eigen::Dynamic);

    // // skew-symmetric: consteval 强制编译期 + constexpr 通用版本
    // template<EigenVector3Like Derived>
    // consteval Eigen::Matrix<typename Derived::Scalar, 3, 3> skew_sym_matrix_ce(const Eigen::MatrixBase<Derived>& v)
    // {
    //     using T = typename Derived::Scalar;
    //     return Eigen::Matrix<T, 3, 3>(
    //         T(0), -v(2), v(1),
    //         v(2), T(0), -v(0),
    //         -v(1), v(0), T(0));
    // }

    // template<EigenVector3Like Derived>
    // constexpr Eigen::Matrix<typename Derived::Scalar, 3, 3> skew_sym_matrix(const Eigen::MatrixBase<Derived>& v)
    // {
    //     using T = typename Derived::Scalar;
    //     return Eigen::Matrix<T, 3, 3>(
    //         T(0), -v(2), v(1),
    //         v(2), T(0), -v(0),
    //         -v(1), v(0), T(0));
    // }

    // template<typename T>
    // constexpr Eigen::Matrix<T, 3, 3> skew_sym_matrix(const T v[3])
    // {
    //     return Eigen::Matrix<T, 3, 3>(
    //         T(0), -v[2], v[1],
    //         v[2], T(0), -v[0],
    //         -v[1], v[0], T(0));
    // }

    // // Exp: C++23 if consteval 区分编译期/运行期路径
    // template<typename T>
    // constexpr Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang)
    // {
    //     T           ang_norm = ang.norm();
    //     constexpr T eps      = static_cast<T>(1e-7);

    //     if (ang_norm <= eps)
    //         return Eigen::Matrix<T, 3, 3>::Identity();

    //     Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    //     T                      x = r_axis(0), y = r_axis(1), z = r_axis(2);

    //     if consteval
    //     {
    //         // 编译期路径（C++23）：
    //         // Eigen 的 operator* 不是 constexpr，因此手动逐元素展开 Rodrigues 公式
    //         // 利用恒等式 K^2 = r*r^T - I（单位向量），得到：
    //         // R = cos(θ)I + (1-cos(θ))r*r^T + sin(θ)K
    //         T s  = std::sin(ang_norm); // C++23: std::sin 为 constexpr
    //         T c  = std::cos(ang_norm); // C++23: std::cos 为 constexpr
    //         T om = T(1) - c;

    //         Eigen::Matrix<T, 3, 3> R;
    //         R(0, 0) = c + om * x * x;
    //         R(0, 1) = om * x * y - s * z;
    //         R(0, 2) = om * x * z + s * y;
    //         R(1, 0) = om * x * y + s * z;
    //         R(1, 1) = c + om * y * y;
    //         R(1, 2) = om * y * z - s * x;
    //         R(2, 0) = om * x * z - s * y;
    //         R(2, 1) = om * y * z + s * x;
    //         R(2, 2) = c + om * z * z;
    //         return R;
    //     }
    //     else
    //     {
    //         // 运行期路径：保留 Eigen 表达式模板，享受向量化优化
    //         [[assume(ang_norm > eps)]]; // C++23: 告知编译器此分支角度非零
    //         Eigen::Matrix<T, 3, 3> K = skew_sym_matrix(r_axis);
    //         return Eigen::Matrix<T, 3, 3>::Identity() + std::sin(ang_norm) * K + (T(1) - std::cos(ang_norm)) * K * K;
    //     }
    // }

    // template<typename T, typename Ts>
    // constexpr Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt)
    // {
    //     T           ang_vel_norm = ang_vel.norm();
    //     constexpr T eps          = static_cast<T>(1e-7);

    //     if (ang_vel_norm <= eps)
    //         return Eigen::Matrix<T, 3, 3>::Identity();

    //     Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    //     T                      x = r_axis(0), y = r_axis(1), z = r_axis(2);
    //     T                      r_ang = ang_vel_norm * dt;

    //     if consteval
    //     {
    //         T s  = std::sin(r_ang);
    //         T c  = std::cos(r_ang);
    //         T om = T(1) - c;

    //         Eigen::Matrix<T, 3, 3> R;
    //         R(0, 0) = c + om * x * x;
    //         R(0, 1) = om * x * y - s * z;
    //         R(0, 2) = om * x * z + s * y;
    //         R(1, 0) = om * x * y + s * z;
    //         R(1, 1) = c + om * y * y;
    //         R(1, 2) = om * y * z - s * x;
    //         R(2, 0) = om * x * z - s * y;
    //         R(2, 1) = om * y * z + s * x;
    //         R(2, 2) = c + om * z * z;
    //         return R;
    //     }
    //     else
    //     {
    //         [[assume(ang_vel_norm > eps)]];
    //         Eigen::Matrix<T, 3, 3> K = skew_sym_matrix(r_axis);
    //         return Eigen::Matrix<T, 3, 3>::Identity() + std::sin(r_ang) * K + (T(1) - std::cos(r_ang)) * K * K;
    //     }
    // }

    // template<typename T>
    // constexpr Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3)
    // {
    //     // 修复：原代码 T&& 绑定到 std::sqrt 的返回值（右值引用延长临时对象生命周期，无意义且危险）
    //     T           norm = std::sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    //     constexpr T eps  = static_cast<T>(1e-5);

    //     if (norm <= eps)
    //         return Eigen::Matrix<T, 3, 3>::Identity();

    //     T r[3] = {v1 / norm, v2 / norm, v3 / norm};
    //     T x = r[0], y = r[1], z = r[2];

    //     if consteval
    //     {
    //         T s  = std::sin(norm);
    //         T c  = std::cos(norm);
    //         T om = T(1) - c;

    //         Eigen::Matrix<T, 3, 3> R;
    //         R(0, 0) = c + om * x * x;
    //         R(0, 1) = om * x * y - s * z;
    //         R(0, 2) = om * x * z + s * y;
    //         R(1, 0) = om * x * y + s * z;
    //         R(1, 1) = c + om * y * y;
    //         R(1, 2) = om * y * z - s * x;
    //         R(2, 0) = om * x * z - s * y;
    //         R(2, 1) = om * y * z + s * x;
    //         R(2, 2) = c + om * z * z;
    //         return R;
    //     }
    //     else
    //     {
    //         [[assume(norm > eps)]];
    //         Eigen::Matrix<T, 3, 3> K = skew_sym_matrix(r);
    //         return Eigen::Matrix<T, 3, 3>::Identity() + std::sin(norm) * K + (T(1) - std::cos(norm)) * K * K;
    //     }
    // }

    // // Log: C++23 constexpr + 编译期小角度泰勒展开
    // template<typename T>
    // constexpr Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R)
    // {
    //     T           trace_val = R.trace();
    //     constexpr T eps_trace = static_cast<T>(1e-6);
    //     constexpr T eps_theta = static_cast<T>(1e-3);

    //     T theta = (trace_val > T(3) - eps_trace)
    //                 ? T(0)
    //                 : std::acos(T(0.5) * (trace_val - T(1)));

    //     Eigen::Matrix<T, 3, 1> K(
    //         R(2, 1) - R(1, 2),
    //         R(0, 2) - R(2, 0),
    //         R(1, 0) - R(0, 1));

    //     if consteval
    //     {
    //         // 编译期：避免 std::sin 调用，用泰勒展开 θ/sin(θ) ≈ 1 + θ²/6
    //         if (std::abs(theta) < eps_theta)
    //             return T(0.5) * K;
    //         T theta2   = theta * theta;
    //         T inv_sinc = T(1) + theta2 / T(6); // 1/sinc(θ) 的近似
    //         return T(0.5) * theta * inv_sinc * K;
    //     }
    //     else
    //     {
    //         [[assume(std::abs(theta) >= T(0))]];
    //         return (std::abs(theta) < eps_theta)
    //                  ? (T(0.5) * K)
    //                  : (T(0.5) * theta / std::sin(theta) * K);
    //     }
    // }

    // // RotMtoEuler: 标准 constexpr 化
    // template<typename T>
    // constexpr Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3>& rot)
    // {
    //     T           sy       = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    //     constexpr T eps      = static_cast<T>(1e-6);
    //     bool        singular = sy < eps;

    //     T x, y, z;
    //     if (! singular)
    //     {
    //         x = std::atan2(rot(2, 1), rot(2, 2));
    //         y = std::atan2(-rot(2, 0), sy);
    //         z = std::atan2(rot(1, 0), rot(0, 0));
    //     }
    //     else
    //     {
    //         x = std::atan2(-rot(1, 2), rot(1, 1));
    //         y = std::atan2(-rot(2, 0), sy);
    //         z = T(0);
    //     }
    //     return Eigen::Matrix<T, 3, 1>(x, y, z);
    // }

} // namespace hsm
#endif