#include <atomic>
#include <chrono>
#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/ros2/ros_camera.hpp"
#include "io/ros2/sim_gimbal.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_sentry_sim.yaml | 位置参数,yaml配置文件路径 }";

using namespace std::chrono_literals;

namespace
{

constexpr double kRadToDeg = 57.3;

void draw_armor_point_order(cv::Mat & img, const std::vector<cv::Point2f> & points)
{
  if (points.size() != 4) return;

  for (int i = 0; i < 4; ++i) {
    const auto p0 = points[i];
    const auto p1 = points[(i + 1) % 4];
    cv::line(img, p0, p1, {255, 255, 0}, 2);
    cv::circle(img, p0, 5, {0, 255, 255}, -1);
    tools::draw_text(
      img, std::to_string(i),
      {static_cast<int>(p0.x) + 8, static_cast<int>(p0.y) - 8},
      {0, 255, 255}, 0.9, 2);
  }
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Recorder recorder;

  io::RosCamera camera;
  io::SimGimbal gimbal(config_path);
  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  std::atomic<bool> display_target_exist{false};
  std::atomic<bool> display_control{false};
  std::atomic<bool> display_fire{false};
  std::atomic<double> display_yaw_cmd{0.0};
  std::atomic<double> display_pitch_cmd{0.0};
  std::atomic<double> display_rx_yaw_deg{0.0};
  std::atomic<double> display_rx_pitch_deg{0.0};

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!quit && !exiter.exit() && rclcpp::ok()) {
      camera.read(img, t);
      if (!img.empty()) detector.push(img, t);
    }
  });

  auto plan_thread = std::thread([&]() {
    while (!quit && rclcpp::ok()) {
      if (!target_queue.empty()) {
        auto target = target_queue.pop();
        auto gs = gimbal.state();
        auto plan = planner.plan(target, gs.bullet_speed);

        display_control = plan.control;
        display_fire = plan.fire;
        display_yaw_cmd = plan.yaw;
        display_pitch_cmd = plan.pitch;

        gimbal.send(plan.control, plan.fire, plan.yaw, plan.pitch);
        std::this_thread::sleep_for(10ms);
      } else {
        gimbal.send(false, false, 0.0f, 0.0f);
        std::this_thread::sleep_for(20ms);
      }
    }
  });

  auto last_fps_time = std::chrono::steady_clock::now();
  int fps_frame_count = 0;
  double current_fps = 0.0;

  while (!exiter.exit() && rclcpp::ok()) {
    auto [img, armors, t] = detector.debug_pop();
    auto orientation = gimbal.orientation(t);
    auto euler_deg = gimbal.euler(t);
    display_rx_yaw_deg = euler_deg[2];
    display_rx_pitch_deg = euler_deg[1];

    solver.set_R_gimbal2world(orientation);
    auto targets = tracker.track(armors, t);
    display_target_exist = !targets.empty();
    if (!targets.empty()) {
      target_queue.push(targets.front());
    } else {
      target_queue.push(std::nullopt);
    }

    auto debug_img = img.clone();
    for (const auto & armor : armors) {
      if (!armor.points.empty()) {
        tools::draw_points(debug_img, armor.points, {255, 255, 0}, 2);
        draw_armor_point_order(debug_img, armor.points);
      }

      const std::string armor_info = fmt::format(
        "{:.2f} {} {} {}", armor.confidence,
        auto_aim::COLORS[armor.color],
        auto_aim::ARMOR_NAMES[armor.name],
        auto_aim::ARMOR_TYPES[armor.type]);
      tools::draw_text(debug_img, armor_info, armor.center, {0, 255, 0}, 1.0, 2);
    }

    if (!targets.empty()) {
      auto target = targets.front();
      auto ekf_state = target.ekf_x();
      tools::draw_text(
        debug_img,
        fmt::format(
          "EKF pos=({:.2f},{:.2f},{:.2f}) yaw={:.1f}deg",
          ekf_state[0], ekf_state[2], ekf_state[4], ekf_state[6] * kRadToDeg),
        {10, 120}, {0, 255, 255}, 0.6, 2);
    } else {
      tools::draw_text(debug_img, "No Target", {10, 120}, {128, 128, 128}, 0.7, 2);
    }

    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
    fps_frame_count++;
    if (dt >= 1000) {
      current_fps = fps_frame_count * 1000.0 / dt;
      fps_frame_count = 0;
      last_fps_time = now;
    }

    tools::draw_text(
      debug_img,
      fmt::format("Daedalus SIM FPS: {:.1f}", current_fps),
      {10, 30}, {255, 255, 255}, 0.8, 2);
    tools::draw_text(
      debug_img,
      fmt::format(
        "target={} control={} fire={}",
        display_target_exist.load() ? "yes" : "no",
        display_control.load() ? "on" : "off",
        display_fire.load() ? "on" : "off"),
      {10, 60}, {0, 200, 255}, 0.8, 2);
    tools::draw_text(
      debug_img,
      fmt::format(
        "TX yaw={:.2f}deg pitch={:.2f}deg | RX yaw={:.2f}deg pitch={:.2f}deg",
        display_yaw_cmd.load() * kRadToDeg,
        display_pitch_cmd.load() * kRadToDeg,
        display_rx_yaw_deg.load(),
        display_rx_pitch_deg.load()),
      {10, 90}, {0, 255, 255}, 0.7, 2);

    recorder.record(debug_img, euler_deg / kRadToDeg, t);
    cv::Mat display_img;
    cv::resize(debug_img, display_img, {}, 0.75, 0.75);
    cv::imshow("QYG Sentry Sim", display_img);
    auto key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
  }

  quit = true;
  gimbal.send(false, false, 0.0f, 0.0f);
  rclcpp::shutdown();
  if (detect_thread.joinable()) detect_thread.join();
  if (plan_thread.joinable()) plan_thread.join();
  return 0;
}
