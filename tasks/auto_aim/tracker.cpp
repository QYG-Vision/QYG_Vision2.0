#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <limits>
#include <stdexcept>
#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace
{
auto_aim::Color parse_enemy_color(const std::string & enemy_color)
{
  if (enemy_color == "red") return auto_aim::Color::red;
  if (enemy_color == "blue") return auto_aim::Color::blue;
  throw std::invalid_argument{
    "enemy_color must be \"red\" or \"blue\", got \"" + enemy_color + "\""};
}
}  // namespace

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  if (!yaml["enemy_color"]) {
    throw std::invalid_argument{"Missing required config key: enemy_color"};
  }
  enemy_color_ = parse_enemy_color(yaml["enemy_color"].as<std::string>());
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
  max_match_distance_ = yaml["max_match_distance"] ? yaml["max_match_distance"].as<double>() : 0.5;
  max_match_yaw_diff_ = yaml["max_match_yaw_diff"] ? yaml["max_match_yaw_diff"].as<double>() : 0.7;
  armor_switch_confirm_frames_ = yaml["armor_switch_confirm_frames"]
                                   ? yaml["armor_switch_confirm_frames"].as<int>()
                                   : armor_switch_confirm_frames_;
  control_max_linear_speed_mps_ = yaml["control_max_linear_speed_mps"]
                                    ? yaml["control_max_linear_speed_mps"].as<double>()
                                    : control_max_linear_speed_mps_;
  control_max_angular_speed_radps_ = yaml["control_max_angular_speed_radps"]
                                       ? yaml["control_max_angular_speed_radps"].as<double>()
                                       : control_max_angular_speed_radps_;
  control_max_recent_nis_fail_rate_ = yaml["control_max_recent_nis_fail_rate"]
                                        ? yaml["control_max_recent_nis_fail_rate"].as<double>()
                                        : control_max_recent_nis_fail_rate_;
  TuoLuoConfig tuoluo_config;
  if (yaml["tuoluo_enter_speed_radps"])
    tuoluo_config.enter_speed_radps = yaml["tuoluo_enter_speed_radps"].as<double>();
  if (yaml["tuoluo_exit_speed_radps"])
    tuoluo_config.exit_speed_radps = yaml["tuoluo_exit_speed_radps"].as<double>();
  if (yaml["tuoluo_enter_duration_s"])
    tuoluo_config.enter_duration_s = yaml["tuoluo_enter_duration_s"].as<double>();
  if (yaml["tuoluo_exit_duration_s"])
    tuoluo_config.exit_duration_s = yaml["tuoluo_exit_duration_s"].as<double>();
  if (yaml["tuoluo_filter_alpha"])
    tuoluo_config.filter_alpha = yaml["tuoluo_filter_alpha"].as<double>();
  tuoluo_detector_ = TuoLuoDetector{tuoluo_config};
  if (
    max_match_distance_ <= 0.0 || max_match_yaw_diff_ <= 0.0 ||
    armor_switch_confirm_frames_ < 1) {
    throw std::invalid_argument{
      "max_match_distance/max_match_yaw_diff must be positive and "
      "armor_switch_confirm_frames must be at least 1"};
  }
  if (
    control_max_linear_speed_mps_ <= 0.0 || control_max_angular_speed_radps_ <= 0.0 ||
    control_max_recent_nis_fail_rate_ < 0.0 || control_max_recent_nis_fail_rate_ > 1.0) {
    throw std::invalid_argument{"invalid control safety gate parameters"};
  }
}

void Tracker::set_enemy_color(const std::string & enemy_color)
{
  enemy_color_ = parse_enemy_color(enemy_color);
}

void Tracker::reset()
{
  state_ = "lost";
  pre_state_ = "lost";
  detect_count_ = 0;
  temp_lost_count_ = 0;
  max_temp_lost_count_ = normal_temp_lost_count_;
  latest_armor_ypd_in_world_.reset();
  latest_armor_xyz_in_world_.reset();
  has_target_model_ = false;
  initialization_rejected_ = false;
  control_gate_rejected_ = false;
  tuoluo_detector_.reset();
}

std::string Tracker::state() const { return state_; }

bool Tracker::ekf_initializing() const { return state_ == "detecting" && has_target_model_; }

int Tracker::ekf_warmup_count() const
{
  if (state_ == "detecting") return detect_count_;
  if (state_ == "tracking") return min_detect_count_;
  return 0;
}

bool Tracker::ekf_initialization_rejected() const { return initialization_rejected_; }

bool Tracker::control_safe() const { return !control_gate_rejected_; }

bool Tracker::TuoLuo() const { return tuoluo_detector_.TuoLuo; }

std::optional<Eigen::Vector3d> Tracker::latest_armor_ypd_in_world() const
{
  return latest_armor_ypd_in_world_;
}

std::optional<Eigen::Vector3d> Tracker::latest_armor_xyz_in_world() const
{
  return latest_armor_xyz_in_world_;
}

std::optional<Target> Tracker::diagnostic_target() const
{
  if (!has_target_model_) return std::nullopt;
  return target_;
}

void Tracker::filter_armors_by_enemy_color(std::list<Armor> & armors, bool use_enemy_color) const
{
  if (!use_enemy_color) return;
  armors.remove_if([this](const Armor & armor) { return armor.color != enemy_color_; });
}

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  latest_armor_ypd_in_world_.reset();
  latest_armor_xyz_in_world_.reset();
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  filter_armors_by_enemy_color(armors, use_enemy_color);

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  initialization_rejected_ = state_ == "detecting" && !found;
  state_machine(found);
  update_tuoluo(found, dt);
  if (has_target_model_) {
    target_.ekf().data["temp_lost_count"] = static_cast<double>(temp_lost_count_);
    target_.ekf().data["max_temp_lost_count"] = static_cast<double>(max_temp_lost_count_);
  }

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost" || state_ == "detecting") return {};

  const auto & x = target_.ekf_x();
  const auto & data = target_.ekf().data;
  const auto get = [&data](const char * key, double fallback) {
    const auto it = data.find(key);
    return it == data.end() ? fallback : it->second;
  };
  const double linear_speed = std::hypot(x[1], x[3]);
  const double recent_nis_fail_rate = get("recent_nis_failures", 0.0);
  const bool prediction_only = get("prediction_only", 0.0) > 0.5;
  const bool allow_temp_lost_prediction = state_ == "temp_lost" && prediction_only;
  control_gate_rejected_ = get("nis_fail", 0.0) > 0.5 ||
                           (get("match_rejected", 0.0) > 0.5 && !allow_temp_lost_prediction) ||
                           linear_speed > control_max_linear_speed_mps_ ||
                           std::abs(x[7]) > control_max_angular_speed_radps_ ||
                           recent_nis_fail_rate > control_max_recent_nis_fail_rate_;
  if (control_gate_rejected_) {
    tools::logger()->debug("[Tracker] control gate rejected target state");
    return {};
  }

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  latest_armor_ypd_in_world_.reset();
  latest_armor_xyz_in_world_.reset();
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  filter_armors_by_enemy_color(armors, use_enemy_color);

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  initialization_rejected_ = state_ == "detecting" && !found;
  pre_state_ = state_;
  // 更新状态机
  state_machine(found);
  update_tuoluo(found, dt);
  if (has_target_model_) {
    target_.ekf().data["temp_lost_count"] = static_cast<double>(temp_lost_count_);
    target_.ekf().data["max_temp_lost_count"] = static_cast<double>(max_temp_lost_count_);
  }

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost" || state_ == "detecting") return {switch_target, {}};

  const auto & x = target_.ekf_x();
  const auto & data = target_.ekf().data;
  const auto get = [&data](const char * key, double fallback) {
    const auto it = data.find(key);
    return it == data.end() ? fallback : it->second;
  };
  const double linear_speed = std::hypot(x[1], x[3]);
  const double recent_nis_fail_rate = get("recent_nis_failures", 0.0);
  const bool prediction_only = get("prediction_only", 0.0) > 0.5;
  const bool allow_temp_lost_prediction = state_ == "temp_lost" && prediction_only;
  control_gate_rejected_ = get("nis_fail", 0.0) > 0.5 ||
                           (get("match_rejected", 0.0) > 0.5 && !allow_temp_lost_prediction) ||
                           linear_speed > control_max_linear_speed_mps_ ||
                           std::abs(x[7]) > control_max_angular_speed_radps_ ||
                           recent_nis_fail_rate > control_max_recent_nis_fail_rate_;
  if (control_gate_rejected_) {
    tools::logger()->debug("[Tracker] control gate rejected target state");
    return {switch_target, {}};
  }

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) {
      temp_lost_count_ = 0;
      return;
    }

    temp_lost_count_ = 1;
    max_temp_lost_count_ =
      target_.name == ArmorName::outpost ? outpost_max_temp_lost_count_ : normal_temp_lost_count_;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      temp_lost_count_ = 0;
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) {
    if (has_target_model_) target_.mark_unmatched();
    return false;
  }

  auto & armor = armors.front();
  solver_.solve(armor);
  latest_armor_ypd_in_world_ = armor.ypd_in_world;
  latest_armor_xyz_in_world_ = armor.xyz_in_world;

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(
      armor, t, 0.2, 2, P0_dig, max_match_distance_, max_match_yaw_diff_,
      armor_switch_confirm_frames_);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(
      armor, t, 0.2765, 3, P0_dig, max_match_distance_, max_match_yaw_diff_,
      armor_switch_confirm_frames_);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(
      armor, t, 0.3205, 3, P0_dig, max_match_distance_, max_match_yaw_diff_,
      armor_switch_confirm_frames_);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 0.0004, 1, 1}};
    target_ = Target(
      armor, t, 0.24, 4, P0_dig, max_match_distance_, max_match_yaw_diff_,
      armor_switch_confirm_frames_);
  }

  has_target_model_ = true;
  tuoluo_detector_.reset();
  target_.TuoLuo = false;

  return true;
}

void Tracker::update_tuoluo(bool found, double dt)
{
  if (!has_target_model_) {
    tuoluo_detector_.reset();
    return;
  }
  if (state_ == "lost") {
    tuoluo_detector_.reset();
    target_.TuoLuo = false;
    target_.ekf().data["TuoLuo"] = 0.0;
    return;
  }

  const auto & data = target_.ekf().data;
  const auto value = [&data](const char * key, double fallback) {
    const auto it = data.find(key);
    return it == data.end() ? fallback : it->second;
  };
  const bool reliable = state_ == "tracking" && found && value("observation_accepted", 0.0) > 0.5 &&
                        value("nis_fail", 0.0) < 0.5;
  tuoluo_detector_.update(target_.ekf_x()[7], dt, reliable);
  target_.TuoLuo = tuoluo_detector_.TuoLuo;
  target_.ekf().data["TuoLuo"] = target_.TuoLuo ? 1.0 : 0.0;
  target_.ekf().data["tuoluo_filtered_w_radps"] = tuoluo_detector_.filtered_w_radps();
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  std::vector<Armor> observations;
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    solver_.solve(armor);
    observations.push_back(armor);
  }

  Target updated_target = target_;
  const TargetUpdateResult update_result = updated_target.update(observations);
  if (!update_result.matched()) {
    target_.mark_unmatched();
    return false;
  }

  target_ = std::move(updated_target);
  const auto best = std::max_element(
    update_result.accepted_observation_indices.begin(),
    update_result.accepted_observation_indices.end(), [&](std::size_t lhs, std::size_t rhs) {
      return observations[lhs].box.area() < observations[rhs].box.area();
    });
  latest_armor_ypd_in_world_ = observations[*best].ypd_in_world;
  latest_armor_xyz_in_world_ = observations[*best].xyz_in_world;

  return true;
}

}  // namespace auto_aim
