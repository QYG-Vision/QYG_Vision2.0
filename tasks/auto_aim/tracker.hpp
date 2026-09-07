#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tuoluo_detector.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);
  Tracker(const std::string & config_path, Solver & solver, std::string enemy_color);
  void set_enemy_color(const std::string & enemy_color);
  void reset();

  std::string state() const;
  bool ekf_initializing() const;
  int ekf_warmup_count() const;
  bool ekf_initialization_rejected() const;
  bool control_safe() const;
  bool TuoLuo() const;
  std::optional<Eigen::Vector3d> latest_armor_ypd_in_world() const;
  std::optional<Eigen::Vector3d> latest_armor_xyz_in_world() const;
  std::optional<Target> diagnostic_target() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_{Color::red};
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  double max_match_distance_;
  double max_match_yaw_diff_;
  int armor_switch_confirm_frames_ = 3;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  std::optional<Eigen::Vector3d> latest_armor_ypd_in_world_;
  std::optional<Eigen::Vector3d> latest_armor_xyz_in_world_;
  bool has_target_model_ = false;
  bool initialization_rejected_ = false;
  bool control_gate_rejected_ = false;
  double control_max_linear_speed_mps_ = 8.0;
  double control_max_angular_speed_radps_ = 12.0;
  double control_max_recent_nis_fail_rate_ = 0.4;
  TuoLuoDetector tuoluo_detector_;

  void filter_armors_by_enemy_color(std::list<Armor> & armors, bool use_enemy_color) const;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
  void update_tuoluo(bool found, double dt);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
