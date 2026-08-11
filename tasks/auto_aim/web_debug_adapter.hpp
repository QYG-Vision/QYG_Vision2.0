#ifndef AUTO_AIM__WEB_DEBUG_ADAPTER_HPP
#define AUTO_AIM__WEB_DEBUG_ADAPTER_HPP

#include "target.hpp"
#include "tools/web_debug/web_debug.hpp"

namespace auto_aim
{
tools::web_debug::TargetSnapshot make_web_debug_target(const Target & target);
}  // namespace auto_aim

#endif  // AUTO_AIM__WEB_DEBUG_ADAPTER_HPP
