#include "tasks/auto_aim/planner_command_adapter.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/hero_command_gate.hpp"
#include "tools/thread_safe_queue.hpp"

namespace
{
auto_aim::Plan valid_plan()
{
  auto_aim::Plan plan{};
  plan.control = true;
  plan.fire = true;
  plan.yaw = 1.25F;
  plan.pitch = -0.5F;
  plan.yaw_vel = 2.75F;
  plan.pitch_vel = -1.5F;
  return plan;
}

void assert_stop_command(const io::Command & command)
{
  assert(!command.control);
  assert(!command.shoot);
  assert(command.yaw == 0.0);
  assert(command.pitch == 0.0);
  assert(command.horizon_distance == 0.0);
  assert(command.target_id == 0);
  assert(command.target_valid == 0);
  assert(command.yaw_vel == 0.0);
  assert(command.pitch_vel == 0.0);
}

void converts_all_valid_plan_fields_without_changing_pitch_sign()
{
  const auto command = auto_aim::command_from_plan(valid_plan(), auto_aim::ArmorName::outpost);

  assert(command.control);
  assert(command.shoot);
  assert(command.yaw == 1.25);
  assert(command.pitch == -0.5);
  assert(command.yaw_vel == 2.75);
  assert(command.pitch_vel == -1.5);
  assert(command.target_id == 7);
  assert(command.target_valid == 1);
}

void maps_each_protocol_target_id()
{
  assert(auto_aim::protocol_target_id(auto_aim::ArmorName::one) == 1);
  assert(auto_aim::protocol_target_id(auto_aim::ArmorName::two) == 2);
  assert(auto_aim::protocol_target_id(auto_aim::ArmorName::three) == 3);
  assert(auto_aim::protocol_target_id(auto_aim::ArmorName::four) == 4);
  assert(auto_aim::protocol_target_id(auto_aim::ArmorName::five) == 5);
  assert(auto_aim::protocol_target_id(auto_aim::ArmorName::sentry) == 6);
  assert(auto_aim::protocol_target_id(auto_aim::ArmorName::outpost) == 7);
  assert(auto_aim::protocol_target_id(auto_aim::ArmorName::base) == 8);
}

void rejects_disabled_control_and_unknown_targets()
{
  auto plan = valid_plan();
  plan.control = false;
  assert_stop_command(auto_aim::command_from_plan(plan, auto_aim::ArmorName::one));

  assert_stop_command(auto_aim::command_from_plan(valid_plan(), auto_aim::ArmorName::not_armor));
}

void hero_stop_commands_do_not_fall_back_to_legacy_active_semantics()
{
  auto plan = valid_plan();
  plan.control = false;
  const auto command = auto_aim::command_from_plan(plan, auto_aim::ArmorName::one);
  const auto packet = io::Gimbal::make_control_packet(command);

  assert(!command.control);
  assert(packet.mode == 0);
  assert(packet.yaw == 0.0F);
  assert(packet.pitch == 0.0F);
  assert(packet.yaw_vel == 0.0F);
  assert(packet.pitch_vel == 0.0F);
  assert(packet.target_id == 0);
  assert(packet.target_valid == 0);
}

void rejects_non_finite_angles_and_velocities()
{
  auto plan = valid_plan();
  plan.yaw = std::numeric_limits<float>::quiet_NaN();
  assert_stop_command(auto_aim::command_from_plan(plan, auto_aim::ArmorName::one));

  plan = valid_plan();
  plan.pitch = std::numeric_limits<float>::infinity();
  assert_stop_command(auto_aim::command_from_plan(plan, auto_aim::ArmorName::one));

  plan = valid_plan();
  plan.yaw_vel = -std::numeric_limits<float>::infinity();
  assert_stop_command(auto_aim::command_from_plan(plan, auto_aim::ArmorName::one));

  plan = valid_plan();
  plan.pitch_vel = std::numeric_limits<float>::quiet_NaN();
  assert_stop_command(auto_aim::command_from_plan(plan, auto_aim::ArmorName::one));
}

void rejects_live_commands_from_inactive_or_stale_epochs()
{
  auto_aim::HeroCommandGate gate;
  const auto live = auto_aim::command_from_plan(valid_plan(), auto_aim::ArmorName::one);

  const auto current = gate.command_for_context(live, 7, 7, true);
  assert(current.control);
  assert(current.shoot);

  assert_stop_command(gate.command_for_context(live, 7, 8, true));
  assert_stop_command(gate.command_for_context(live, 8, 8, false));
}

void send_preparation_rechecks_the_latest_control_context()
{
  auto_aim::HeroCommandGate gate;
  const auto live = auto_aim::command_from_plan(valid_plan(), auto_aim::ArmorName::one);

  const auto first = auto_aim::prepare_command_for_send(gate, live, 7, 7, true, false, false);
  assert(first.control);
  assert(first.shoot);

  assert_stop_command(auto_aim::prepare_command_for_send(gate, live, 7, 7, false, false, false));
  assert_stop_command(auto_aim::prepare_command_for_send(gate, live, 7, 8, true, false, false));
  assert_stop_command(auto_aim::prepare_command_for_send(gate, live, 8, 8, true, true, false));
  assert_stop_command(auto_aim::prepare_command_for_send(gate, live, 8, 8, true, false, true));
}

void a_pending_stop_request_cannot_be_replaced_by_target_work()
{
  auto_aim::PendingStopRequest pending_stop;
  pending_stop.request(7);

  tools::ThreadSafeQueue<int, true> latest_targets(1);
  latest_targets.push(1);
  latest_targets.push(2);

  const auto stop_generation = pending_stop.take();
  assert(stop_generation.has_value());
  assert(*stop_generation == 7);
  assert(!pending_stop.take().has_value());
}

void exit_observed_after_an_active_command_forces_the_next_send_to_stop()
{
  auto_aim::HeroCommandGate gate;
  const auto live = auto_aim::command_from_plan(valid_plan(), auto_aim::ArmorName::one);

  assert(auto_aim::prepare_command_for_send(gate, live, 9, 9, true, false, false).control);
  assert_stop_command(
    auto_aim::prepare_command_for_send(gate, live, 9, 9, true, false, true));
}

void emits_one_stop_for_each_inactive_period()
{
  auto_aim::HeroCommandGate gate;

  auto stop = gate.stop_for_inactive(false);
  assert(stop.has_value());
  assert_stop_command(*stop);
  assert(!gate.stop_for_inactive(false).has_value());

  assert(!gate.stop_for_inactive(true).has_value());
  stop = gate.stop_for_inactive(false);
  assert(stop.has_value());
  assert_stop_command(*stop);
  assert(!gate.stop_for_inactive(false).has_value());
}

void stale_exception_stop_suppresses_the_following_inactive_stop()
{
  auto_aim::HeroCommandGate gate;

  assert_stop_command(gate.stop_for_context(11, 11, true));
  auto stop = gate.stop_for_inactive(false);
  assert(stop.has_value());
  assert_stop_command(*stop);

  assert_stop_command(gate.stop_for_context(11, 12, true));
  assert(!gate.stop_for_inactive(false).has_value());

  assert(!gate.stop_for_inactive(true).has_value());
  assert_stop_command(gate.stop_for_context(12, 12, false));
  assert(!gate.stop_for_inactive(false).has_value());
}
}  // namespace

int main()
{
  converts_all_valid_plan_fields_without_changing_pitch_sign();
  maps_each_protocol_target_id();
  rejects_disabled_control_and_unknown_targets();
  hero_stop_commands_do_not_fall_back_to_legacy_active_semantics();
  rejects_non_finite_angles_and_velocities();
  rejects_live_commands_from_inactive_or_stale_epochs();
  send_preparation_rechecks_the_latest_control_context();
  a_pending_stop_request_cannot_be_replaced_by_target_work();
  exit_observed_after_an_active_command_forces_the_next_send_to_stop();
  emits_one_stop_for_each_inactive_period();
  stale_exception_stop_suppresses_the_following_inactive_stop();
  return 0;
}
