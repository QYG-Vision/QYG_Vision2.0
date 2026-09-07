#include "target.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, double max_match_distance, double max_match_yaw_diff,
  int armor_switch_confirm_frames)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  armor_num_(armor_num),
  switch_count_(0),
  update_count_(0),
  armor_switch_confirm_frames_(armor_switch_confirm_frames),
  is_switch_(false),
  is_converged_(false),
  t_(t),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff)
{
  if (armor_switch_confirm_frames_ < 1) {
    throw std::invalid_argument{"armor_switch_confirm_frames must be at least 1"};
  }
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  // 初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  // 初始化滤波器（预测量、预测量协方差）
  update_switch_diagnostics();
}

Target::Target(double x, double vyaw, double radius, double h)
: name(ArmorName::three),
  armor_type(ArmorType::small),
  priority(ArmorPriority::first),
  jumped(false),
  last_id(0),
  armor_num_(4),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(true),
  t_(std::chrono::steady_clock::now())
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  // 初始化滤波器（预测量、预测量协方差）
  update_switch_diagnostics();
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 90;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 40;   // 加速度方差
    v2 = 300;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
}

bool Target::update(const Armor & armor) { return update(std::vector<Armor>{armor}).matched(); }

TargetUpdateResult Target::update(const std::vector<Armor> & armors)
{
  constexpr double NIS_GATE_THRESHOLD = 9.488;
  const double infinity = std::numeric_limits<double>::infinity();
  TargetUpdateResult result;
  ekf_.data["frame_observation_count"] = static_cast<double>(armors.size());
  ekf_.data["armor_association_switched"] = 0.0;
  ekf_.data["armor_switched"] = 0.0;

  if (armors.empty() || armor_num_ <= 0) {
    mark_unmatched();
    return result;
  }

  std::vector<std::vector<double>> costs(armors.size(), std::vector<double>(armor_num_, infinity));
  std::vector<std::vector<double>> yaw_diffs(
    armors.size(), std::vector<double>(armor_num_, infinity));
  std::vector<std::vector<double>> distance_diffs(
    armors.size(), std::vector<double>(armor_num_, infinity));
  for (std::size_t observation = 0; observation < armors.size(); ++observation) {
    for (int id = 0; id < armor_num_; ++id) {
      costs[observation][id] = association_cost(
        armors[observation], id, yaw_diffs[observation][id], distance_diffs[observation][id]);
      if (costs[observation][id] > NIS_GATE_THRESHOLD) costs[observation][id] = infinity;
    }
  }

  struct AssignmentState
  {
    bool reachable = false;
    double cost = std::numeric_limits<double>::infinity();
    std::vector<std::pair<std::size_t, int>> assignments;
  };

  const std::size_t mask_count = std::size_t{1} << armor_num_;
  std::vector<AssignmentState> states(mask_count);
  states[0] = {true, 0.0, {}};
  for (std::size_t observation = 0; observation < armors.size(); ++observation) {
    auto next = states;  // Skipping a rejected or duplicate observation is always allowed.
    for (std::size_t mask = 0; mask < mask_count; ++mask) {
      if (!states[mask].reachable) continue;
      for (int id = 0; id < armor_num_; ++id) {
        const std::size_t bit = std::size_t{1} << id;
        if ((mask & bit) != 0 || !std::isfinite(costs[observation][id])) continue;
        const std::size_t new_mask = mask | bit;
        const double new_cost = states[mask].cost + costs[observation][id];
        if (!next[new_mask].reachable || new_cost < next[new_mask].cost) {
          next[new_mask] = states[mask];
          next[new_mask].reachable = true;
          next[new_mask].cost = new_cost;
          next[new_mask].assignments.emplace_back(observation, id);
        }
      }
    }
    states = std::move(next);
  }

  const AssignmentState * best = &states[0];
  for (const auto & state : states) {
    if (!state.reachable) continue;
    if (
      state.assignments.size() > best->assignments.size() ||
      (state.assignments.size() == best->assignments.size() && state.cost < best->cost)) {
      best = &state;
    }
  }

  if (best->assignments.empty()) {
    reset_switch_candidate();
    ekf_.data["match_accepted"] = 0.0;
    ekf_.data["match_rejected"] = 1.0;
    ekf_.data["match_no_candidate"] = 0.0;
    ekf_.data["observation_accepted"] = 0.0;
    ekf_.data["prediction_only"] = 0.0;
    ekf_.data["armor_switched"] = 0.0;
    ekf_.data["switch_velocity_reset"] = 0.0;
    ekf_.data["frame_assigned_count"] = 0.0;
    ekf_.data["frame_accepted_count"] = 0.0;
    return result;
  }

  auto assignments = best->assignments;
  std::sort(assignments.begin(), assignments.end(), [&](const auto & lhs, const auto & rhs) {
    const double lhs_cost = costs[lhs.first][lhs.second];
    const double rhs_cost = costs[rhs.first][rhs.second];
    return lhs_cost == rhs_cost ? lhs.second < rhs.second : lhs_cost < rhs_cost;
  });

  struct AcceptedAssignment
  {
    std::size_t observation;
    int id;
  };
  std::vector<AcceptedAssignment> accepted;
  const double angular_velocity_before_update = ekf_.x[7];
  const double geometry_alpha = assignments.size() > 1 ? 1.0 : 0.25;
  for (const auto & [observation, id] : assignments) {
    if (update_ypda(armors[observation], id, geometry_alpha)) {
      accepted.push_back({observation, id});
      result.accepted_observation_indices.push_back(observation);
    }
  }

  ekf_.data["frame_assigned_count"] = static_cast<double>(assignments.size());
  ekf_.data["frame_accepted_count"] = static_cast<double>(accepted.size());
  if (accepted.empty()) {
    reset_switch_candidate();
    ekf_.data["match_accepted"] = 0.0;
    ekf_.data["match_rejected"] = 1.0;
    ekf_.data["observation_accepted"] = 0.0;
    ekf_.data["prediction_only"] = 0.0;
    ekf_.data["armor_switched"] = 0.0;
    ekf_.data["switch_velocity_reset"] = 0.0;
    return result;
  }

  auto primary = std::find_if(accepted.begin(), accepted.end(), [&](const auto & assignment) {
    return assignment.id == association_id_;
  });
  if (primary == accepted.end()) {
    primary =
      std::max_element(accepted.begin(), accepted.end(), [&](const auto & lhs, const auto & rhs) {
        return armors[lhs.observation].box.area() < armors[rhs.observation].box.area();
      });
  }

  const bool association_switched = primary->id != association_id_;
  is_switch_ = false;
  if (association_switched) {
    ekf_.x[7] = angular_velocity_before_update;
    for (const int index : {1, 3, 5}) {
      ekf_.P(index, index) = std::max(ekf_.P(index, index), 1.0);
    }
    association_id_ = primary->id;
  }

  for (const auto & assignment : accepted) {
    if (assignment.id != 0) jumped = true;
  }
  is_switch_ = confirm_armor_switch(primary->id);
  if (is_switch_) {
    last_id = primary->id;
    ++switch_count_;
  }
  ++update_count_;

  ekf_.data["match_accepted"] = 1.0;
  ekf_.data["match_rejected"] = 0.0;
  ekf_.data["match_no_candidate"] = 0.0;
  ekf_.data["match_yaw_diff"] = yaw_diffs[primary->observation][primary->id];
  ekf_.data["match_distance_diff"] = distance_diffs[primary->observation][primary->id];
  ekf_.data["match_yaw_rejected"] = 0.0;
  ekf_.data["match_distance_rejected"] = 0.0;
  ekf_.data["observation_accepted"] = 1.0;
  ekf_.data["prediction_only"] = 0.0;
  ekf_.data["nis_fail"] = 0.0;
  ekf_.data["armor_switched"] = is_switch_ ? 1.0 : 0.0;
  ekf_.data["armor_association_switched"] = association_switched ? 1.0 : 0.0;
  ekf_.data["armor_association_id"] = static_cast<double>(association_id_);
  ekf_.data["switch_velocity_reset"] = 0.0;
  return result;
}

void Target::mark_unmatched()
{
  reset_switch_candidate();
  ekf_.data["armor_association_switched"] = 0.0;
  ekf_.data["match_accepted"] = 0.0;
  ekf_.data["match_rejected"] = 1.0;
  ekf_.data["match_no_candidate"] = 1.0;
  ekf_.data["match_yaw_diff"] = 0.0;
  ekf_.data["match_distance_diff"] = 0.0;
  ekf_.data["match_yaw_rejected"] = 0.0;
  ekf_.data["match_distance_rejected"] = 0.0;
  ekf_.data["observation_accepted"] = 0.0;
  ekf_.data["prediction_only"] = 1.0;
  ekf_.data["nis_fail"] = 0.0;
  ekf_.data["armor_switched"] = 0.0;
  ekf_.data["switch_velocity_reset"] = 0.0;
}

void Target::reset_switch_candidate()
{
  switch_candidate_id_ = -1;
  switch_candidate_count_ = 0;
  update_switch_diagnostics();
}

bool Target::confirm_armor_switch(int candidate_id)
{
  if (candidate_id == last_id) {
    reset_switch_candidate();
    return false;
  }

  if (switch_candidate_id_ == candidate_id) {
    ++switch_candidate_count_;
  } else {
    switch_candidate_id_ = candidate_id;
    switch_candidate_count_ = 1;
  }
  update_switch_diagnostics();

  if (switch_candidate_count_ < armor_switch_confirm_frames_) return false;
  reset_switch_candidate();
  return true;
}

void Target::update_switch_diagnostics()
{
  ekf_.data["armor_association_id"] = static_cast<double>(association_id_);
  ekf_.data["armor_switch_candidate_id"] = static_cast<double>(switch_candidate_id_);
  ekf_.data["armor_switch_candidate_count"] = static_cast<double>(switch_candidate_count_);
  ekf_.data["armor_switch_confirm_frames"] =
    static_cast<double>(armor_switch_confirm_frames_);
}

Eigen::Matrix4d Target::measurement_covariance(const Armor & armor) const
{
  const double center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  const double abs_delta_angle = std::abs(tools::limit_rad(armor.ypr_in_world[0] - center_yaw));
  const double distance = std::abs(armor.ypd_in_world[2]);
  const double side_view_scale = 1.0 + 0.75 * std::min(abs_delta_angle, 1.2);
  Eigen::Vector4d variances;
  variances << 4e-4, 4e-4, std::pow((0.02 + 0.01 * distance) * side_view_scale, 2),
    std::pow(0.12 + 0.35 * abs_delta_angle, 2);
  return variances.asDiagonal();
}

double Target::association_cost(
  const Armor & armor, int id, double & yaw_diff, double & distance_diff) const
{
  const Eigen::Vector3d predicted_xyz = h_armor_xyz(ekf_.x, id);
  const Eigen::Vector3d predicted_ypd = tools::xyz2ypd(predicted_xyz);
  const double predicted_angle = tools::limit_rad(ekf_.x[6] + id * 2 * CV_PI / armor_num_);
  yaw_diff = std::abs(tools::limit_rad(armor.ypd_in_world[0] - predicted_ypd[0]));
  distance_diff = std::abs(armor.ypd_in_world[2] - predicted_ypd[2]);
  const double orientation_diff =
    std::abs(tools::limit_rad(armor.ypr_in_world[0] - predicted_angle));
  if (
    yaw_diff > max_match_yaw_diff_ || orientation_diff > max_match_yaw_diff_ ||
    distance_diff > max_match_distance_) {
    return std::numeric_limits<double>::infinity();
  }

  Eigen::Vector4d innovation;
  innovation << tools::limit_rad(armor.ypd_in_world[0] - predicted_ypd[0]),
    tools::limit_rad(armor.ypd_in_world[1] - predicted_ypd[1]),
    armor.ypd_in_world[2] - predicted_ypd[2],
    tools::limit_rad(armor.ypr_in_world[0] - predicted_angle);
  const Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  const Eigen::Matrix4d S = H * ekf_.P * H.transpose() + measurement_covariance(armor);
  const Eigen::LDLT<Eigen::Matrix4d> solver{S};
  if (solver.info() != Eigen::Success) return std::numeric_limits<double>::infinity();
  return innovation.dot(solver.solve(innovation));
}

bool Target::update_ypda(const Armor & armor, int id, double geometry_alpha)
{
  // 观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  auto prev_r = ekf_.x[8];
  auto prev_l = ekf_.x[9];
  auto prev_h = ekf_.x[10];

  // 观测噪声建模（顺序：yaw, pitch, distance, angle）
  // 快速横移时 angle 波动会增大；distance 不能长期被弱化，否则会通过拉大半径去拟合。
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  auto abs_delta_angle = std::abs(delta_angle);

  const Eigen::Matrix4d R = measurement_covariance(armor);

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  // 获得观测量

  ekf_.data["side_view_angle_rad"] = abs_delta_angle;
  ekf_.data["distance_observation_var"] = R(2, 2);
  ekf_.data["ekf_linear_speed_mps"] = std::hypot(ekf_.x[1], ekf_.x[3]);
  ekf_.data["ekf_angular_speed_radps"] = std::abs(ekf_.x[7]);

  // 四维观测在 95% 置信水平下的卡方阈值。超限观测只保留 predict 结果，不污染状态。
  constexpr double NIS_GATE_THRESHOLD = 9.488;
  ekf_.update(z, H, R, h, z_subtract, NIS_GATE_THRESHOLD);
  if (ekf_.data["nis_fail"] != 0.0) return false;

  ekf_.x[8] = prev_r + geometry_alpha * (ekf_.x[8] - prev_r);
  ekf_.x[9] = prev_l + geometry_alpha * (ekf_.x[9] - prev_l);
  ekf_.x[10] = prev_h + geometry_alpha * (ekf_.x[10] - prev_h);
  clamp_geometry();
  return true;
}

void Target::clamp_geometry()
{
  auto clamp = [](double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); };

  if (name == ArmorName::outpost) {
    ekf_.x[8] = clamp(ekf_.x[8], 0.20, 0.35);
    ekf_.x[9] = clamp(ekf_.x[9], -0.05, 0.05);
    ekf_.x[10] = clamp(ekf_.x[10], -0.05, 0.05);
  } else if (name == ArmorName::base) {
    ekf_.x[8] = clamp(ekf_.x[8], 0.24, 0.40);
    ekf_.x[9] = clamp(ekf_.x[9], -0.06, 0.06);
    ekf_.x[10] = clamp(ekf_.x[10], -0.06, 0.06);
  } else {
    ekf_.x[8] = clamp(ekf_.x[8], 0.12, 0.38);
    const double min_signed_offset = std::max(-0.20, 0.06 - ekf_.x[8]);
    const double max_signed_offset = std::min(0.20, 0.50 - ekf_.x[8]);
    ekf_.x[9] = clamp(ekf_.x[9], min_signed_offset, max_signed_offset);
    ekf_.x[10] = clamp(ekf_.x[10], -0.30, 0.30);
  }
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

int Target::associated_armor_id() const { return association_id_; }

int Target::switch_candidate_id() const { return switch_candidate_id_; }

int Target::switch_candidate_count() const { return switch_candidate_count_; }

int Target::switch_confirm_frames() const { return armor_switch_confirm_frames_; }

tools::ExtendedKalmanFilter & Target::ekf() { return ekf_; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  // 前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
};

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
