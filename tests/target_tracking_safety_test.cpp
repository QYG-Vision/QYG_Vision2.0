#include <cassert>
#include <chrono>
#include <cmath>
#include <set>
#include <vector>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/tuoluo_detector.hpp"
#include "tools/math_tools.hpp"

namespace
{
auto_aim::Armor make_armor(const Eigen::Vector3d & xyz, double armor_yaw)
{
  std::vector<cv::Point2f> points{{0, 0}, {2, 0}, {2, 1}, {0, 1}};
  auto_aim::Armor armor{0, 3, 0.95F, cv::Rect{0, 0, 2, 1}, std::move(points)};
  armor.name = auto_aim::ArmorName::three;
  armor.type = auto_aim::ArmorType::small;
  armor.priority = auto_aim::ArmorPriority::first;
  armor.xyz_in_world = xyz;
  armor.ypd_in_world = tools::xyz2ypd(xyz);
  armor.ypr_in_world = {armor_yaw, 0.0, 0.0};
  return armor;
}

auto_aim::Target make_target(const auto_aim::Armor & armor, int switch_confirm_frames = 3)
{
  Eigen::VectorXd covariance{{1, 64, 1, 64, 1, 64, 0.4, 100, 0.0004, 1, 1}};
  return auto_aim::Target{
    armor, std::chrono::steady_clock::now(), 0.24, 4, covariance, 0.5, 0.7,
    switch_confirm_frames};
}

void rejects_an_observation_outside_association_gates()
{
  const auto initial = make_armor({3.0, 0.0, 0.2}, 0.0);
  auto target = make_target(initial);
  const Eigen::VectorXd before = target.ekf_x();

  auto invalid = make_armor({5.0, 5.0, 0.2}, 1.5);
  assert(!target.update(invalid));
  assert((target.ekf_x() - before).norm() < 1e-12);
  assert(target.ekf().data.at("match_rejected") == 1.0);
  assert(target.ekf().data.at("observation_accepted") == 0.0);
}

void side_view_increases_distance_observation_variance()
{
  const auto initial = make_armor({3.0, 0.0, 0.2}, 0.0);
  auto frontal_target = make_target(initial);
  auto side_target = make_target(initial);

  assert(frontal_target.update(initial));
  auto side_observation = initial;
  side_observation.ypr_in_world[0] = 0.6;
  side_target.update(side_observation);

  const double frontal_var = frontal_target.ekf().data.at("distance_observation_var");
  const double side_var = side_target.ekf().data.at("distance_observation_var");
  assert(side_var > frontal_var);
}

void armor_switch_preserves_vehicle_velocity()
{
  auto observation = make_armor({3.0, 0.0, 0.2}, 0.0);
  auto target = make_target(observation, 1);

  constexpr double vx = 0.8;
  constexpr double vy = -0.4;
  constexpr double vz = 0.2;
  target.ekf().x[1] = vx;
  target.ekf().x[3] = vy;
  target.ekf().x[5] = vz;
  const double angular_velocity_before_switch = target.ekf_x()[7];

  const Eigen::Vector4d predicted_switch = target.armor_xyza_list().at(1);
  const auto switched = make_armor(predicted_switch.head<3>(), predicted_switch[3]);
  assert(target.update(switched));
  const Eigen::VectorXd state = target.ekf_x();
  assert(std::abs(state[1] - vx) < 1e-12);
  assert(std::abs(state[3] - vy) < 1e-12);
  assert(std::abs(state[5] - vz) < 1e-12);
  assert(std::abs(state[7] - angular_velocity_before_switch) < 1e-12);
  assert(target.ekf().data.at("armor_switched") == 1.0);
  assert(target.ekf().data.at("switch_velocity_reset") == 0.0);
  assert(target.ekf().P(1, 1) >= 1.0);
  assert(target.ekf().P(3, 3) >= 1.0);
  assert(target.ekf().P(5, 5) >= 1.0);
  assert(state[8] >= 0.12 && state[8] <= 0.38);
  assert(state[9] >= -0.12 && state[9] <= 0.12);
  assert(state[10] >= -0.20 && state[10] <= 0.20);
}

void armor_switch_requires_consecutive_confirmation()
{
  const auto initial = make_armor({3.0, 0.0, 0.2}, 0.0);
  auto target = make_target(initial);

  for (int count = 1; count <= 2; ++count) {
    const auto predicted = target.armor_xyza_list().at(1);
    auto candidate = make_armor(predicted.head<3>(), predicted[3]);
    assert(target.update(candidate));
    assert(target.associated_armor_id() == 1);
    assert(target.last_id == 0);
    assert(target.switch_candidate_id() == 1);
    assert(target.switch_candidate_count() == count);
    assert(target.ekf().data.at("armor_switched") == 0.0);
  }

  const auto predicted = target.armor_xyza_list().at(1);
  auto candidate = make_armor(predicted.head<3>(), predicted[3]);
  assert(target.update(candidate));
  assert(target.associated_armor_id() == 1);
  assert(target.last_id == 1);
  assert(target.switch_candidate_id() == -1);
  assert(target.switch_candidate_count() == 0);
  assert(target.ekf().data.at("armor_switched") == 1.0);
}

void armor_switch_candidate_reversal_and_loss_reset_confirmation()
{
  const auto initial = make_armor({3.0, 0.0, 0.2}, 0.0);
  auto target = make_target(initial);

  auto predicted = target.armor_xyza_list().at(1);
  assert(target.update(make_armor(predicted.head<3>(), predicted[3])));
  assert(target.switch_candidate_count() == 1);

  predicted = target.armor_xyza_list().at(0);
  assert(target.update(make_armor(predicted.head<3>(), predicted[3])));
  assert(target.associated_armor_id() == 0);
  assert(target.last_id == 0);
  assert(target.switch_candidate_id() == -1);
  assert(target.switch_candidate_count() == 0);

  predicted = target.armor_xyza_list().at(1);
  assert(target.update(make_armor(predicted.head<3>(), predicted[3])));
  assert(target.switch_candidate_id() == 1);
  assert(target.switch_candidate_count() == 1);
  target.mark_unmatched();
  assert(target.switch_candidate_id() == -1);
  assert(target.switch_candidate_count() == 0);
  assert(target.ekf().data.at("armor_switched") == 0.0);
}

void committed_id_does_not_steer_raw_association()
{
  const auto initial = make_armor({3.0, 0.0, 0.2}, 0.0);
  auto target = make_target(initial);

  auto predicted = target.armor_xyza_list();
  assert(target.update(make_armor(predicted[1].head<3>(), predicted[1][3])));
  assert(target.associated_armor_id() == 1);
  assert(target.last_id == 0);
  assert(target.switch_candidate_count() == 1);

  predicted = target.armor_xyza_list();
  auto committed = make_armor(predicted[0].head<3>(), predicted[0][3]);
  auto associated = make_armor(predicted[1].head<3>(), predicted[1][3]);
  const auto result = target.update(std::vector<auto_aim::Armor>{committed, associated});
  assert(result.accepted_observation_indices.size() == 2);
  assert(target.associated_armor_id() == 1);
  assert(target.last_id == 0);
  assert(target.switch_candidate_id() == 1);
  assert(target.switch_candidate_count() == 2);
}

void tuoluo_requires_sustained_speed_and_hysteresis()
{
  auto_aim::TuoLuoConfig config;
  config.filter_alpha = 1.0;
  auto_aim::TuoLuoDetector detector{config};

  for (int i = 0; i < 9; ++i) assert(!detector.update(3.0, 0.01, true));
  assert(detector.update(3.0, 0.02, true));

  for (int i = 0; i < 29; ++i) assert(detector.update(1.0, 0.01, true));
  assert(!detector.update(1.0, 0.02, true));

  for (int i = 0; i < 20; ++i) assert(!detector.update(3.0, 0.01, false));
}

void multiple_armors_update_signed_geometry_in_one_frame()
{
  const auto initial = make_armor({3.0, 0.0, 0.2}, 0.0);
  auto target = make_target(initial);
  const auto predicted = target.armor_xyza_list();

  auto first = make_armor(predicted[0].head<3>(), predicted[0][3]);
  Eigen::Vector3d second_xyz = predicted[1].head<3>();
  second_xyz.y() += 0.06;  // id 1 has a 6 cm shorter radius.
  second_xyz.z() -= 0.10;  // id 1 is 10 cm lower than id 0.
  auto second = make_armor(second_xyz, predicted[1][3]);

  const auto result = target.update(std::vector<auto_aim::Armor>{first, second});
  assert(result.accepted_observation_indices.size() == 2);
  assert(target.ekf().data.at("frame_assigned_count") == 2.0);
  assert(target.ekf().data.at("frame_accepted_count") == 2.0);
  assert(target.ekf_x()[9] < 0.0);
  assert(target.ekf_x()[10] < 0.0);
}

void duplicate_observations_cannot_claim_the_same_or_adjacent_offset()
{
  const auto initial = make_armor({3.0, 0.0, 0.2}, 0.0);
  auto target = make_target(initial);
  const auto predicted = target.armor_xyza_list().front();
  auto duplicate_a = make_armor(predicted.head<3>(), predicted[3]);
  auto duplicate_b = duplicate_a;

  const auto result = target.update(std::vector<auto_aim::Armor>{duplicate_a, duplicate_b});
  assert(result.accepted_observation_indices.size() == 1);
  assert(target.ekf().data.at("frame_assigned_count") == 1.0);
}

void planner_uses_tuoluo_to_lock_or_switch_armor_offsets()
{
  auto_aim::Planner planner{"configs/QYG_hero.yaml"};
  planner.set_prediction_disabled(true);

  auto_aim::Target locked_target{3.0, 4.0, 0.24, 0.0};
  locked_target.TuoLuo = false;
  assert(planner.plan(locked_target, 22.0).control);
  const std::set<int> locked_ids{planner.debug_armor_ids.begin(), planner.debug_armor_ids.end()};
  assert(locked_ids.size() == 1);

  auto_aim::Target spinning_target{3.0, 4.0, 0.24, 0.0};
  spinning_target.TuoLuo = true;
  assert(planner.plan(spinning_target, 22.0).control);
  const std::set<int> spinning_ids{planner.debug_armor_ids.begin(), planner.debug_armor_ids.end()};
  assert(spinning_ids.size() > 1);
}

void planner_predicts_translation_over_bullet_flight_time()
{
  auto_aim::Target moving_target{3.0, 0.0, 0.24, 0.0};
  moving_target.ekf().x[1] = 1.0;

  auto_aim::Planner without_prediction{"configs/QYG_hero.yaml"};
  without_prediction.set_prediction_disabled(true);
  assert(without_prediction.plan(moving_target, 22.0).control);

  auto_aim::Planner with_prediction{"configs/QYG_hero.yaml"};
  with_prediction.set_prediction_disabled(false);
  assert(with_prediction.plan(moving_target, 22.0).control);

  assert(with_prediction.debug_fly_time > 0.0);
  assert(std::abs(with_prediction.debug_fly_time - without_prediction.debug_fly_time) < 1e-12);
  assert(with_prediction.debug_xyza.x() > without_prediction.debug_xyza.x() + 0.05);
}
}  // namespace

int main()
{
  rejects_an_observation_outside_association_gates();
  side_view_increases_distance_observation_variance();
  armor_switch_preserves_vehicle_velocity();
  armor_switch_requires_consecutive_confirmation();
  armor_switch_candidate_reversal_and_loss_reset_confirmation();
  committed_id_does_not_steer_raw_association();
  tuoluo_requires_sustained_speed_and_hysteresis();
  multiple_armors_update_signed_geometry_in_one_frame();
  duplicate_observations_cannot_claim_the_same_or_adjacent_offset();
  planner_uses_tuoluo_to_lock_or_switch_armor_offsets();
  planner_predicts_translation_over_bullet_flight_time();
  return 0;
}
