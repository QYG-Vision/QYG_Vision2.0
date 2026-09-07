#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

struct TargetUpdateResult
{
  std::vector<std::size_t> accepted_observation_indices;

  bool matched() const { return !accepted_observation_indices.empty(); }
};

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  bool TuoLuo = false;
  int last_id;  // 连续确认后提交给 Planner 等下游消费者的物理板 ID

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, double max_match_distance = 0.5, double max_match_yaw_diff = 0.7,
    int armor_switch_confirm_frames = 3);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  bool update(const Armor & armor);
  TargetUpdateResult update(const std::vector<Armor> & armors);
  void mark_unmatched();

  Eigen::VectorXd ekf_x() const;
  tools::ExtendedKalmanFilter & ekf();
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  int associated_armor_id() const;
  int switch_candidate_id() const;
  int switch_candidate_count() const;
  int switch_confirm_frames() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;
  int association_id_ = 0;
  int switch_candidate_id_ = -1;
  int switch_candidate_count_ = 0;
  int armor_switch_confirm_frames_ = 3;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;
  double max_match_distance_ = 0.5;
  double max_match_yaw_diff_ = 0.7;

  double association_cost(
    const Armor & armor, int id, double & yaw_diff, double & distance_diff) const;
  Eigen::Matrix4d measurement_covariance(const Armor & armor) const;
  bool update_ypda(const Armor & armor, int id, double geometry_alpha);
  void clamp_geometry();
  void reset_switch_candidate();
  bool confirm_armor_switch(int candidate_id);
  void update_switch_diagnostics();

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
