#include <yaml-cpp/yaml.h>

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tasks/auto_aim/solver.hpp"

namespace
{
void transforms_camera_points_with_solver_extrinsics()
{
  const std::string config_path = "configs/QYG_hero.yaml";
  const auto yaml = YAML::LoadFile(config_path);
  const auto rotation_values = yaml["R_camera2gimbal"].as<std::vector<double>>();
  const auto translation_values = yaml["t_camera2gimbal"].as<std::vector<double>>();
  const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_camera2gimbal(rotation_values.data());
  const Eigen::Vector3d t_camera2gimbal(translation_values.data());

  auto_aim::Solver solver(config_path);
  const auto now = io::GimbalClock::now();
  const io::GimbalFeedbackSample feedback{Eigen::Quaterniond::Identity(), 0.0, now, now};
  solver.set_R_gimbal2world(feedback);

  const Eigen::Vector3d point_in_camera{1.2, -0.3, 4.0};
  const Eigen::Vector3d expected =
    solver.R_gimbal2world() * (R_camera2gimbal * point_in_camera + t_camera2gimbal);
  assert((solver.camera_to_world(point_in_camera) - expected).norm() < 1e-12);
}

void rejects_non_finite_camera_points()
{
  auto_aim::Solver solver("configs/QYG_hero.yaml");
  bool threw = false;
  try {
    static_cast<void>(solver.camera_to_world({std::numeric_limits<double>::infinity(), 0.0, 1.0}));
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);
}
}  // namespace

int main()
{
  transforms_camera_points_with_solver_extrinsics();
  rejects_non_finite_camera_points();
  return 0;
}
