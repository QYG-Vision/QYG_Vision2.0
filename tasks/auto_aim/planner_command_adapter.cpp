#include "planner_command_adapter.hpp"

#include <cmath>

namespace auto_aim
{
std::uint8_t protocol_target_id(ArmorName name)
{
  switch (name) {
    case ArmorName::one:
      return 1;
    case ArmorName::two:
      return 2;
    case ArmorName::three:
      return 3;
    case ArmorName::four:
      return 4;
    case ArmorName::five:
      return 5;
    case ArmorName::sentry:
      return 6;
    case ArmorName::outpost:
      return 7;
    case ArmorName::base:
      return 8;
    default:
      return 0;
  }
}

io::Command command_from_plan(const Plan & plan, ArmorName name)
{
  const auto target_id = protocol_target_id(name);
  if (!plan.control || target_id == 0 || !std::isfinite(plan.yaw) || !std::isfinite(plan.pitch) ||
      !std::isfinite(plan.yaw_vel) || !std::isfinite(plan.pitch_vel)) {
    return {};
  }

  io::Command command{};
  command.control = plan.control;
  command.shoot = plan.fire;
  command.yaw = plan.yaw;
  command.pitch = plan.pitch;
  command.yaw_vel = plan.yaw_vel;
  command.pitch_vel = plan.pitch_vel;
  command.target_id = target_id;
  command.target_valid = 1;
  return command;
}
}  // namespace auto_aim
