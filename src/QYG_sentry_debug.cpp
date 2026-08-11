#include <atomic>
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/web_debug_adapter.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"
#include "tools/web_debug/web_debug.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{display d      |      | 启用本地 OpenCV 调试窗口}"
  "{no-web         |      | 禁用 Web 发布，用于性能对照}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml 配置文件路径}";

struct PlanDebugSnapshot
{
  auto_aim::Plan plan{};
  Eigen::Vector4d aim_xyza = Eigen::Vector4d::Zero();
  Clock::time_point computed_at{};
  std::uint64_t observation_frame_id = 0;
  std::optional<auto_aim::ArmorName> target_name;
  std::optional<auto_aim::ArmorType> armor_type;
  double planner_ms = 0.0;
  bool valid = false;
  bool aim_valid = false;
};

struct TargetPacket
{
  std::optional<auto_aim::Target> target;
  std::uint64_t observation_frame_id = 0;
};

double elapsed_ms(Clock::time_point begin, Clock::time_point end = Clock::now())
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

tools::web_debug::DetectionOverlay detection_overlay(const auto_aim::Armor & armor)
{
  return {
    armor.points,
    fmt::format(
      "{} {} {}", auto_aim::COLORS.at(armor.color), auto_aim::ARMOR_NAMES.at(armor.name),
      auto_aim::ARMOR_TYPES.at(armor.type)),
    armor.confidence};
}

tools::web_debug::OverlaySnapshot make_overlays(
  const std::list<auto_aim::Armor> & armors,
  const std::optional<auto_aim::Target> & target,
  const PlanDebugSnapshot & plan,
  const auto_aim::Solver & solver)
{
  tools::web_debug::OverlaySnapshot overlays;
  overlays.detections.reserve(armors.size());
  for (const auto & armor : armors) overlays.detections.push_back(detection_overlay(armor));

  if (!target) return overlays;
  const auto predicted_armors = target->armor_xyza_list();
  overlays.projected_armors.reserve(predicted_armors.size() + (plan.aim_valid ? 1 : 0));
  for (std::size_t index = 0; index < predicted_armors.size(); ++index) {
    const auto & xyza = predicted_armors[index];
    overlays.projected_armors.push_back(
      {solver.reproject_armor(
         xyza.head(3), xyza[3], target->armor_type, target->name),
       false, index});
  }
  if (plan.aim_valid) {
    overlays.projected_armors.push_back(
      {solver.reproject_armor(
         plan.aim_xyza.head(3), plan.aim_xyza[3], target->armor_type, target->name),
       true, 0});
  }

  const auto state = target->ekf_x();
  if (state.size() < 8) return overlays;
  const Eigen::Vector3d center(state[0], state[2], state[4]);
  const Eigen::Vector3d velocity(state[1], state[3], state[5]);
  const double yaw_length = 0.25 + std::min(std::abs(state[7]), 5.0) * 0.03;
  const Eigen::Vector3d yaw_endpoint =
    center + yaw_length * Eigen::Vector3d(std::cos(state[6]), std::sin(state[6]), 0.0);
  const auto points = solver.project_world_points({center, center + velocity * 0.3, yaw_endpoint});
  if (points.size() == 3 && points[0]) {
    overlays.target_center = points[0];
    if (points[1]) overlays.velocity_endpoint = points[1];
    if (points[2]) overlays.yaw_endpoint = points[2];
  }
  return overlays;
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }
  const auto config_path = cli.get<std::string>("@config-path");
  const bool display_enabled = cli.has("display");
  const bool web_enabled = !cli.has("no-web");
  const bool debug_enabled = web_enabled || display_enabled;

  tools::Exiter exiter;
  tools::Recorder recorder;
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::web_debug::WebDebugRenderer display_renderer;
  std::unique_ptr<tools::web_debug::AsyncWebDebugPublisher> web_publisher;
  if (web_enabled) {
    web_publisher = std::make_unique<tools::web_debug::AsyncWebDebugPublisher>();
  }
  if (web_enabled && !web_publisher->ready()) {
    tools::logger()->warn("[WebDebug] /dev/shm frame publisher is unavailable; aiming continues");
  }

  tools::ThreadSafeQueue<TargetPacket, true> target_queue(1);
  target_queue.push({std::nullopt, 0});
  std::mutex plan_snapshot_mutex;
  PlanDebugSnapshot plan_snapshot;

  std::atomic<bool> quit = false;
  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode = io::GimbalMode::IDLE;
  int idle_counter = 0;

  auto last_fps_time = Clock::now();
  int fps_frame_count = 0;
  double current_fps = 0.0;
  std::uint64_t frame_id = 0;
  double last_publisher_ms = 0.0;
  std::atomic<std::uint64_t> web_submit_failure_count{0};
  auto last_heartbeat_submit = Clock::time_point{};

  auto detect_thread = std::thread([&]() {
    cv::Mat image;
    Clock::time_point timestamp;
    while (!quit && !exiter.exit()) {
      if (mode.load() == io::GimbalMode::AUTO_AIM) {
        camera.read(image, timestamp);
        detector.push(image, timestamp);
      } else {
        std::this_thread::sleep_for(10ms);
      }
    }
  });

  auto plan_thread = std::thread([&]() {
    while (!quit) {
      if (mode.load() == io::GimbalMode::AUTO_AIM && !target_queue.empty()) {
        auto packet = target_queue.pop();
        const auto gimbal_state = gimbal.state();
        Clock::time_point plan_begin{};
        if (debug_enabled) plan_begin = Clock::now();
        const auto plan = planner.plan(packet.target, gimbal_state.bullet_speed);

        if (debug_enabled) {
          PlanDebugSnapshot next;
          next.plan = plan;
          next.computed_at = Clock::now();
          next.observation_frame_id = packet.observation_frame_id;
          if (packet.target) {
            next.target_name = packet.target->name;
            next.armor_type = packet.target->armor_type;
          }
          next.planner_ms = elapsed_ms(plan_begin, next.computed_at);
          next.valid = true;
          next.aim_valid = packet.target.has_value() && plan.control;
          if (next.aim_valid) next.aim_xyza = planner.debug_xyza;
          {
            std::lock_guard<std::mutex> lock(plan_snapshot_mutex);
            plan_snapshot = next;
          }
        }

        gimbal.send(
          plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
          plan.pitch_vel, plan.pitch_acc);
        tools::logger()->info(
          "Sent Command - Control: {}, Fire: {}, Yaw: {:.2f}, Pitch: {:.2f}", plan.control,
          plan.fire, plan.yaw, plan.pitch);
        std::this_thread::sleep_for(10ms);
      } else {
        std::this_thread::sleep_for(50ms);
      }
    }
  });

  std::thread web_warning_thread;
  if (web_enabled) {
    web_warning_thread = std::thread([&]() {
      std::uint64_t observed_publish_failures = 0;
      std::uint64_t observed_submit_failures = 0;
      auto last_warning = Clock::time_point{};
      bool warning_pending = false;
      while (!quit) {
        const auto publish_failures = web_publisher->failure_count();
        const auto submit_failures = web_submit_failure_count.load(std::memory_order_relaxed);
        if (publish_failures != observed_publish_failures ||
            submit_failures != observed_submit_failures) {
          observed_publish_failures = publish_failures;
          observed_submit_failures = submit_failures;
          warning_pending = true;
        }
        const auto now = Clock::now();
        if (warning_pending &&
            (last_warning.time_since_epoch().count() == 0 || now - last_warning >= 5s)) {
          tools::logger()->warn(
            "[WebDebug] isolated failure(s): publish={}, submit={}; aiming continues",
            observed_publish_failures, observed_submit_failures);
          last_warning = now;
          warning_pending = false;
        }
        std::this_thread::sleep_for(250ms);
      }
    });
  }

  auto publish_heartbeat = [&](io::GimbalMode heartbeat_mode, Clock::time_point now) {
    if (!web_enabled ||
        (last_heartbeat_submit.time_since_epoch().count() != 0 &&
         now - last_heartbeat_submit < 50ms)) {
      return;
    }
    tools::web_debug::WebDebugContext context;
    context.system.source = "QYG_sentry_debug";
    context.system.mode = gimbal.str(heartbeat_mode);
    context.system.producer_online = true;
    context.system.fps = 0.0;
    context.system.frame_id = frame_id;
    context.detector.queue_depth = detector.queue_size();
    context.tracker.state = "inactive";
    const auto gimbal_state = gimbal.state();
    context.gimbal.yaw_rad = gimbal_state.yaw;
    context.gimbal.pitch_rad = gimbal_state.pitch;
    context.gimbal.bullet_speed_mps = gimbal_state.bullet_speed;
    const bool submitted = web_publisher->submit_status(
      std::move(context), std::chrono::duration<double>(now.time_since_epoch()).count());
    if (!submitted) web_submit_failure_count.fetch_add(1, std::memory_order_relaxed);
    last_heartbeat_submit = now;
  };

  while (!exiter.exit()) {
    mode = gimbal.mode();
    const auto current_mode = mode.load();
    if (last_mode != current_mode) {
      tools::logger()->info("Switch to {}", gimbal.str(current_mode));
      last_mode = current_mode;
    }

    if (current_mode == io::GimbalMode::AUTO_AIM) {
      Clock::time_point detector_wait_begin{};
      if (debug_enabled) detector_wait_begin = Clock::now();
      auto [image, armors, timestamp] = detector.debug_pop();
      const double detector_wait_ms =
        debug_enabled ? elapsed_ms(detector_wait_begin) : 0.0;
      const auto quaternion = gimbal.q(timestamp - 1ms);
      recorder.record(image, quaternion, timestamp);
      solver.set_R_gimbal2world(quaternion);

      Clock::time_point tracker_begin{};
      if (debug_enabled) tracker_begin = Clock::now();
      auto targets = tracker.track(armors, timestamp);
      const double tracker_ms = debug_enabled ? elapsed_ms(tracker_begin) : 0.0;
      const std::uint64_t observation_frame_id = debug_enabled ? ++frame_id : 0;
      target_queue.push(
        {targets.empty() ? std::optional<auto_aim::Target>{} : targets.front(),
         observation_frame_id});

      Clock::time_point now{};
      if (debug_enabled) {
        now = Clock::now();
        ++fps_frame_count;
        const auto fps_window_ms = elapsed_ms(last_fps_time, now);
        if (fps_window_ms >= 1000.0) {
          current_fps = fps_frame_count * 1000.0 / fps_window_ms;
          fps_frame_count = 0;
          last_fps_time = now;
        }
      }

      if (!debug_enabled) continue;

      try {
        PlanDebugSnapshot current_plan;
        {
          std::lock_guard<std::mutex> lock(plan_snapshot_mutex);
          current_plan = plan_snapshot;
        }

        std::optional<auto_aim::Target> current_target;
        if (!targets.empty()) current_target.emplace(std::move(targets.front()));
        const auto current_target_name = current_target
          ? std::optional<auto_aim::ArmorName>{current_target->name}
          : std::nullopt;
        const auto current_armor_type = current_target
          ? std::optional<auto_aim::ArmorType>{current_target->armor_type}
          : std::nullopt;
        const double planner_age_ms = current_plan.valid
          ? elapsed_ms(current_plan.computed_at, now)
          : 0.0;
        current_plan.valid = current_plan.valid && planner_age_ms >= 0.0 &&
                             planner_age_ms <= 250.0 &&
                             current_plan.target_name == current_target_name &&
                             current_plan.armor_type == current_armor_type;
        current_plan.aim_valid = current_plan.valid && current_plan.aim_valid;

        tools::web_debug::WebDebugContext context;
        context.system.source = "QYG_sentry_debug";
        context.system.mode = gimbal.str(current_mode);
        context.system.producer_online = true;
        context.system.fps = current_fps;
        context.system.frame_id = observation_frame_id;
        context.detector.armor_count = armors.size();
        context.detector.queue_depth = detector.queue_size();
        context.tracker.state = tracker.state();
        context.tracker.has_target = current_target.has_value();
        context.timing.frame_age_ms = elapsed_ms(timestamp, now);
        context.timing.detector_wait_ms = detector_wait_ms;
        context.timing.tracker_ms = tracker_ms;
        context.timing.planner_ms = current_plan.valid ? current_plan.planner_ms : 0.0;
        context.timing.publisher_ms = last_publisher_ms;

        const auto current_gimbal = gimbal.state();
        const Eigen::Vector3d gimbal_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
        context.gimbal.yaw_rad = gimbal_ypr[0];
        context.gimbal.pitch_rad = -gimbal_ypr[1];
        context.gimbal.bullet_speed_mps = current_gimbal.bullet_speed;

        if (current_target) context.target = auto_aim::make_web_debug_target(*current_target);
        if (current_plan.valid) {
          context.planner.valid = true;
          context.planner.observation_frame_id = current_plan.observation_frame_id;
          context.planner.age_ms = planner_age_ms;
          context.planner.control = current_plan.plan.control;
          context.planner.fire = current_plan.plan.fire;
          context.planner.target_yaw_rad = current_plan.plan.target_yaw;
          context.planner.target_pitch_rad = current_plan.plan.target_pitch;
          context.planner.command_yaw_rad = current_plan.plan.yaw;
          context.planner.command_pitch_rad = current_plan.plan.pitch;
        }

        auto overlay_factory =
          [overlay_armors = std::move(armors), overlay_target = std::move(current_target),
           current_plan, solver_snapshot = solver] {
            return make_overlays(
              overlay_armors, overlay_target, current_plan, solver_snapshot);
          };

        cv::Mat debug_image;
        if (display_enabled) {
          debug_image = display_renderer.render(image, context, overlay_factory());
        }
        if (web_enabled) {
          const bool submitted = web_publisher->submit(
            std::move(image), context, std::move(overlay_factory),
            std::chrono::duration<double>(now.time_since_epoch()).count());
          last_publisher_ms = web_publisher->last_publish_ms();
          if (!submitted) web_submit_failure_count.fetch_add(1, std::memory_order_relaxed);
        }

        if (display_enabled) {
          cv::Mat display_image;
          cv::resize(debug_image, display_image, {}, 0.5, 0.5);
          cv::imshow("QYG Sentry Debug", display_image);
          const int key = cv::waitKey(1);
          if (key == 'q' || key == 27) break;
        }
      } catch (...) {
        web_submit_failure_count.fetch_add(1, std::memory_order_relaxed);
      }
    } else if (current_mode == io::GimbalMode::IDLE) {
      publish_heartbeat(current_mode, Clock::now());
      if (++idle_counter >= 10) {
        gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
        idle_counter = 0;
      }
      std::this_thread::sleep_for(50ms);
    } else {
      publish_heartbeat(current_mode, Clock::now());
      std::this_thread::sleep_for(10ms);
    }
  }

  quit = true;
  if (detect_thread.joinable()) detect_thread.join();
  if (plan_thread.joinable()) plan_thread.join();
  if (web_warning_thread.joinable()) web_warning_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  if (display_enabled) cv::destroyAllWindows();
  return 0;
}
