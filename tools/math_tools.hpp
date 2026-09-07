#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>
#include <chrono>

namespace tools
{
// 将弧度值限制在(-pi, pi]
double limit_rad(double angle);

// 从 current 到 target 的最短有符号角差，范围为 (-pi, pi]
double angle_difference(double target, double current);

// 沿最短圆弧在两个弧度角之间插值，ratio 通常位于 [0, 1]
double interpolate_angle(double from, double to, double ratio);

// 仅用于显示：把内部 yaw 的 +/-pi 分界位置映射为显示坐标的 0
double yaw_with_zero_at_wrap_boundary(double internal_yaw);

// 四元数转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// 参考：https://github.com/evbernardes/quaternion_to_euler
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

// 旋转矩阵转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

// 欧拉角转旋转矩阵
// zyx:先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr);

// 直角坐标系转球坐标系
// ypd为yaw、pitch、distance的缩写
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// 直角坐标系转球坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 球坐标系转直角坐标系
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd);

// 球坐标系转直角坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd);

// 计算时间差a - b，单位：s
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

// 向量夹角 总是返回 0 ~ pi 来自SJTU
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2);

// 返回输入值的平方
template <typename T>
T square(T const & a)
{
  return a * a;
};

double limit_min_max(double input, double min, double max);
}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP
