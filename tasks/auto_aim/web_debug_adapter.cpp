#include "web_debug_adapter.hpp"

namespace auto_aim
{
tools::web_debug::TargetSnapshot make_web_debug_target(const Target & target)
{
  const Eigen::VectorXd state = target.ekf_x();
  tools::web_debug::TargetSnapshot snapshot;
  snapshot.name = target.name >= 0 && target.name < ARMOR_NAMES.size()
                    ? ARMOR_NAMES[target.name]
                    : "unknown";
  snapshot.type = target.armor_type >= 0 && target.armor_type < ARMOR_TYPES.size()
                    ? ARMOR_TYPES[target.armor_type]
                    : "unknown";
  if (state.size() >= 11) {
    snapshot.x_m = state[0];
    snapshot.vx_mps = state[1];
    snapshot.y_m = state[2];
    snapshot.vy_mps = state[3];
    snapshot.z_m = state[4];
    snapshot.vz_mps = state[5];
    snapshot.yaw_rad = state[6];
    snapshot.yaw_speed_radps = state[7];
    snapshot.radius_m = state[8];
  }
  snapshot.last_id = target.last_id;
  return snapshot;
}
}  // namespace auto_aim
