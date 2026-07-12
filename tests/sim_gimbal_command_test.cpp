#include "io/ros2/sim_gimbal_command.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{

constexpr double kEpsilon = 1e-6;

void expect_near(double actual, double expected, const char * label)
{
  if (std::abs(actual - expected) > kEpsilon) {
    std::cerr << "[FAIL] " << label << ": actual=" << actual
              << " expected=" << expected << std::endl;
    std::exit(1);
  }
}

void expect_true(bool value, const char * label)
{
  if (!value) {
    std::cerr << "[FAIL] " << label << std::endl;
    std::exit(1);
  }
}

}  // namespace

int main()
{
  constexpr double kPi = 3.14159265358979323846;

  const auto active = io::sim_bridge::make_gimbal_cmd_values(
    true, true, kPi / 6.0, -kPi / 12.0, 2.5, -1.0);
  expect_near(active.yaw_deg, 32.5, "active yaw converts rad to deg and applies offset");
  expect_near(active.pitch_deg, 74.0, "active pitch converts rad to deg, adds 90, and offset");
  expect_near(active.distance, 0.0, "active command distance is valid");
  expect_true(active.fire_advice, "active fire requires control and fire");

  const auto no_control = io::sim_bridge::make_gimbal_cmd_values(
    false, true, kPi / 6.0, -kPi / 12.0, 0.0, 0.0);
  expect_near(no_control.distance, -1.0, "inactive command distance disables Daedalus control");
  expect_true(!no_control.fire_advice, "inactive command never fires");

  std::cout << "[PASS] sim gimbal command conversion" << std::endl;
  return 0;
}
