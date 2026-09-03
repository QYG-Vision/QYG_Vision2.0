#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  // Rotation from gimbal frame to IMU absolute/world-like stable frame.
  Eigen::Matrix3d R_gimbal2world() const;

  // Update gimbal->world rotation from IMU quaternion at current timestamp.
  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void set_R_gimbal2world(const Eigen::Vector3d & euler);

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

  std::vector<cv::Point2f> camera2pixel(const std::vector<cv::Point3f> & cameraPoints) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  // 时序一致性先验：跨帧记忆上一次PnP解
  mutable cv::Vec3d prev_rvec_;
  mutable cv::Vec3d prev_tvec_;
  mutable bool has_prev_solution_ = false;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP