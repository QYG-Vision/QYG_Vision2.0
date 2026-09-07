#ifndef AUTO_AIM__PLANNER_COMMAND_ADAPTER_HPP
#define AUTO_AIM__PLANNER_COMMAND_ADAPTER_HPP

#include <cstdint>

#include "io/command.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/planner/planner.hpp"

namespace auto_aim
{
std::uint8_t protocol_target_id(ArmorName name);
io::Command command_from_plan(const Plan & plan, ArmorName name);
}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_COMMAND_ADAPTER_HPP
