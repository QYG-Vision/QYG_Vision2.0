#include <chrono>
#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/ros2/aim2nav.hpp"
#include "io/ros2/nav2aim.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/csv_logger.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"
#include "tools/vofa_plotter.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_sentry.yaml | 位置参数,yaml配置文件路径 }";

using namespace std::chrono_literals;

namespace
{
constexpr bool PNP_IMAGE_ONLY_TEST = true;
constexpr bool ENABLE_VOFA_PLOTTER = true;
constexpr bool ENABLE_CSV_LOGGER = true;
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

  auto aim2nav = std::make_shared<io::Aim2Nav>();
  io::Nav2Aim nav2aim;
  nav2aim.start();

  tools::Exiter exiter;
  tools::Recorder recorder;

  io::Gimbal gimbal(config_path);
  gimbal.set_aim2nav(aim2nav);
  io::Camera camera(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  const std::vector<std::string> plot_fields{
    "time_ms", "rx_yaw_deg", "rx_pitch_deg", "target_found", "armor_found", "armor_x_px",
    "armor_y_px", "xyz_g_z", "ypd_pitch_deg"};
  tools::VofaPlotter vofa_plotter(plot_fields);
  tools::CsvLogger csv_logger(
    plot_fields, "pnp_plot");
  if (ENABLE_VOFA_PLOTTER) {
    tools::logger()->info(
      "[VofaPlotter] VOFA+ realtime: UDP {}:{}, protocol FireWater",
      vofa_plotter.host(), vofa_plotter.port());
    tools::logger()->info("[VofaPlotter] channels: {}", vofa_plotter.channel_description());
  }
  if (ENABLE_CSV_LOGGER) {
    tools::logger()->info("[CsvLogger] VOFA+ offline CSV: {}", csv_logger.path());
  }

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};
  int idle_counter = 0;

  std::atomic<double> display_dist{0.0};
  std::atomic<double> display_pitch_raw{0.0};
  std::atomic<double> display_yaw_cmd{0.0};
  std::atomic<double> display_pitch_cmd{0.0};
  std::atomic<double> display_rx_yaw_deg{0.0};
  std::atomic<double> display_rx_pitch_deg{0.0};
  std::atomic<bool> display_control{false};
  std::atomic<bool> display_fire{false};
  std::atomic<bool> display_target_exist{false};
  std::atomic<bool> display_fixed_cmd{false};

  auto last_fps_time = std::chrono::steady_clock::now();
  int fps_frame_count = 0;
  double current_fps = 0.0;
  cv::Mat debug_img;
  const auto plot_start_time = std::chrono::steady_clock::now();

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!quit && !exiter.exit()) {
      if (mode.load() == io::GimbalMode::AUTO_AIM) {
        camera.read(img, t);
        detector.push(img, t);
      } else {
        std::this_thread::sleep_for(10ms);
      }
    }
  });

  auto plan_thread = std::thread([&]() {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    constexpr double kPitchCmdTau = 0.08;
    constexpr double kMinPitchRad = -25.0 * kDegToRad;
    constexpr double kMaxPitchRad = 15.0 * kDegToRad;
    constexpr double kMaxYawRateRadPerSec = 60.0 * kDegToRad;
    constexpr double kMaxPitchRateRadPerSec = 30.0 * kDegToRad;
    double filtered_pitch = 0.0;
    bool pitch_filter_initialized = false;
    double last_yaw = 0.0;
    double last_pitch = 0.0;
    bool has_last_cmd = false;
    auto last_cmd_time = std::chrono::steady_clock::now();

    while (!quit && rclcpp::ok()) {
      auto cmd_vel = nav2aim.get_latest_state();
      if (PNP_IMAGE_ONLY_TEST && mode.load() == io::GimbalMode::AUTO_AIM) {
        std::optional<auto_aim::Target> target = std::nullopt;
        if (!target_queue.empty()) {
          target = target_queue.pop();
        }

        display_dist = target.has_value() ? std::hypot(target->ekf_x()[0], target->ekf_x()[2]) : 0.0;
        display_pitch_raw = 0.0;
        display_target_exist = target.has_value();
        display_control = false;
        display_fire = false;
        display_yaw_cmd = 0.0;
        display_pitch_cmd = 0.0;
        display_fixed_cmd = true;

        {
          static auto last_pnp_image_only_log = std::chrono::steady_clock::now();
          auto now_pnp_image_only_log = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::milliseconds>(
                now_pnp_image_only_log - last_pnp_image_only_log)
                .count() >= 200) {
            tools::logger()->info(
              "Send [PNP_IMAGE_ONLY] -> target: {}, control: false, fire: false, tx_yaw: 0.0000, tx_pitch: 0.0000, tx_vx: {:.4f}, tx_vy: {:.4f}, tx_wz: {:.4f}, rx_yaw: {:.2f}deg, rx_pitch: {:.2f}deg",
              target.has_value(),
              cmd_vel.linear.x,
              cmd_vel.linear.y,
              cmd_vel.angular.z,
              display_rx_yaw_deg.load(),
              display_rx_pitch_deg.load());
            last_pnp_image_only_log = now_pnp_image_only_log;
          }
        }

        gimbal.send(
          false, false, 0.0f, 0.0f,
          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        std::this_thread::sleep_for(10ms);
        continue;
      }

      if (mode.load() == io::GimbalMode::AUTO_AIM && !target_queue.empty()) {
        auto target = target_queue.pop();
        auto gs = gimbal.state();
        auto plan = planner.plan(target, gs.bullet_speed);
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_cmd_time).count();
        last_cmd_time = now;

        double pitch_cmd = plan.pitch;
        if (plan.control) {
          if (!pitch_filter_initialized) {
            filtered_pitch = plan.pitch;
            pitch_filter_initialized = true;
          } else {
            double alpha = dt / (kPitchCmdTau + dt);
            alpha = tools::limit_min_max(alpha, 0.0, 1.0);
            filtered_pitch += alpha * (plan.pitch - filtered_pitch);
          }
          pitch_cmd = filtered_pitch;
        } else {
          pitch_filter_initialized = false;
        }

        double tx_yaw = plan.yaw;
        double tx_pitch = pitch_cmd;
        double tx_vx = cmd_vel.linear.x;
        double tx_vy = cmd_vel.linear.y;
        double tx_wz = cmd_vel.angular.z;
        bool tx_control = target.has_value() && plan.control;
        bool tx_fire = plan.fire;
        bool pitch_limited = false;
        bool yaw_rate_limited = false;
        bool pitch_rate_limited = false;

        if (tx_control) {
          auto raw_tx_pitch = tx_pitch;
          tx_pitch = tools::limit_min_max(tx_pitch, kMinPitchRad, kMaxPitchRad);
          pitch_limited = std::abs(tx_pitch - raw_tx_pitch) > 1e-6;

          if (has_last_cmd) {
            auto max_yaw_step = kMaxYawRateRadPerSec * dt;
            auto max_pitch_step = kMaxPitchRateRadPerSec * dt;

            auto yaw_delta = tools::limit_rad(tx_yaw - last_yaw);
            auto limited_yaw_delta = tools::limit_min_max(yaw_delta, -max_yaw_step, max_yaw_step);
            yaw_rate_limited = std::abs(limited_yaw_delta - yaw_delta) > 1e-6;
            tx_yaw = tools::limit_rad(last_yaw + limited_yaw_delta);

            auto pitch_delta = tx_pitch - last_pitch;
            auto limited_pitch_delta =
              tools::limit_min_max(pitch_delta, -max_pitch_step, max_pitch_step);
            pitch_rate_limited = std::abs(limited_pitch_delta - pitch_delta) > 1e-6;
            tx_pitch = last_pitch + limited_pitch_delta;
          }

          last_yaw = tx_yaw;
          last_pitch = tx_pitch;
          has_last_cmd = true;
        } else {
          if (!has_last_cmd) {
            last_yaw = gs.vyaw * kDegToRad;
            last_pitch = gs.vpitch * kDegToRad;
            has_last_cmd = true;
          }
          tx_yaw = last_yaw;
          tx_pitch = last_pitch;
          tx_fire = false;
        }
        tx_control = true;

        if (target.has_value()) {
          display_dist = std::hypot(target->ekf_x()[0], target->ekf_x()[2]);
          display_pitch_raw = plan.pitch;
        } else {
          display_dist = 0.0;
          display_pitch_raw = 0.0;
        }
        display_target_exist = target.has_value();
        display_control = tx_control;
        display_fire = tx_fire;
        display_yaw_cmd = tx_yaw;
        display_pitch_cmd = tx_pitch;
        display_fixed_cmd = false;

        tools::logger()->info(
          "Send [{}] -> target: {}, plan_control: {}, tx_control: {}, fire: {}, tx_yaw: {:.4f}, tx_pitch: {:.4f}, planner_yaw: {:.4f}, planner_pitch: {:.4f}, pitch_cmd: {:.4f}, pitch_limited: {}, yaw_rate_limited: {}, pitch_rate_limited: {}, tx_vx: {:.4f}, tx_vy: {:.4f}, tx_wz: {:.4f}",
          target.has_value() && plan.control ? "PLANNER" : "HOLD_NO_TARGET",
          target.has_value(), plan.control, tx_control, tx_fire, tx_yaw, tx_pitch,
          plan.yaw, plan.pitch, pitch_cmd,
          pitch_limited, yaw_rate_limited, pitch_rate_limited,
          tx_vx, tx_vy, tx_wz);

        gimbal.send(
          tx_control, tx_fire, tx_yaw, tx_pitch,
          tx_vx, tx_vy, tx_wz);
        std::this_thread::sleep_for(10ms);
      } else if (mode.load() == io::GimbalMode::AUTO_AIM) {
        auto gs = gimbal.state();
        if (!has_last_cmd) {
          last_yaw = gs.vyaw * kDegToRad;
          last_pitch = gs.vpitch * kDegToRad;
          has_last_cmd = true;
        }
        auto tx_yaw = last_yaw;
        auto tx_pitch = last_pitch;

        display_dist = 0.0;
        display_pitch_raw = 0.0;
        display_target_exist = false;
        display_control = true;
        display_fire = false;
        display_yaw_cmd = tx_yaw;
        display_pitch_cmd = tx_pitch;
        display_fixed_cmd = false;

        {
          static auto last_queue_empty_log = std::chrono::steady_clock::now();
          auto now_queue_empty_log = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::milliseconds>(now_queue_empty_log - last_queue_empty_log).count() >= 200) {
            tools::logger()->info(
              "Send [HOLD_QUEUE_EMPTY] -> mode: {}, control: true, fire: false, tx_yaw: {:.4f}, tx_pitch: {:.4f}, tx_vx: {:.4f}, tx_vy: {:.4f}, tx_wz: {:.4f}",
              gimbal.str(mode.load()), tx_yaw, tx_pitch,
              cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            last_queue_empty_log = now_queue_empty_log;
          }
        }

        gimbal.send(
          true, false, tx_yaw, tx_pitch,
          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        std::this_thread::sleep_for(10ms);
      } else {
        has_last_cmd = false;
        pitch_filter_initialized = false;
        display_fixed_cmd = false;
        {
          static auto last_stop_log = std::chrono::steady_clock::now();
          auto now_stop_log = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::milliseconds>(now_stop_log - last_stop_log).count() >= 200) {
            tools::logger()->info(
              "Send [STOP] -> mode: {}, target_queue_empty: {}, tx_vx: {:.4f}, tx_vy: {:.4f}, tx_wz: {:.4f}",
              gimbal.str(mode.load()), target_queue.empty(),
              cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            last_stop_log = now_stop_log;
          }
        }
        gimbal.send(
          false, false, 0.0f, 0.0f,
          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z); 
        std::this_thread::sleep_for(50ms);
      }
    }
  });

  while (!exiter.exit()) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    // mode = io::GimbalMode::AUTO_AIM; //这里在没有接入串口的的情况下强制进入自瞄模式，方便调试
    mode = gimbal.mode(); //正式接入
    auto current_mode = mode.load();

    // // 临时调试：每秒打印一次原始模式值，确认 EC 是否发了自瞄请求
    // {
    //   static auto last_mode_log = std::chrono::steady_clock::now();
    //   auto now_mode = std::chrono::steady_clock::now();
    //   if (std::chrono::duration_cast<std::chrono::milliseconds>(now_mode - last_mode_log).count() >= 1000) {
    //     tools::logger()->info("Mode raw: str={}, int={}", gimbal.str(current_mode), (int)current_mode);
    //     last_mode_log = now_mode;
    //   }
    // }

    if (last_mode != current_mode) {
      tools::logger()->info("Switch to {}", gimbal.str(current_mode));
      last_mode = current_mode;
    }

    if (current_mode == io::GimbalMode::AUTO_AIM) {
      auto [img, armors, t] = detector.debug_pop();
      auto euler_deg = gimbal.euler(t - 1ms);
      auto q = euler_deg * kDegToRad;
      display_rx_yaw_deg = euler_deg[2];
      display_rx_pitch_deg = euler_deg[1];

      {
        static auto last_tx_rx_log = std::chrono::steady_clock::now();
        auto now_tx_rx = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now_tx_rx - last_tx_rx_log).count() >= 200) {
          tools::logger()->info(
            "TX/RX -> tx_yaw: {:.2f}deg, tx_pitch: {:.2f}deg, rx_yaw: {:.2f}deg, rx_pitch: {:.2f}deg, control: {}, fire: {}",
            display_yaw_cmd.load() * 57.3,
            display_pitch_cmd.load() * 57.3,
            display_rx_yaw_deg.load(),
            display_rx_pitch_deg.load(),
            display_control.load(),
            display_fire.load());
          last_tx_rx_log = now_tx_rx;
        }
      }

      solver.set_R_gimbal2world(q);

      auto targets = tracker.track(armors, t);
      if (!targets.empty()) {
        target_queue.push(targets.front());
      } else {
        target_queue.push(std::nullopt);
      }

      nlohmann::json plot_data;
      plot_data["time_ms"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - plot_start_time)
          .count();
      plot_data["rx_yaw_deg"] = display_rx_yaw_deg.load();
      plot_data["rx_pitch_deg"] = display_rx_pitch_deg.load();
      plot_data["target_found"] = !targets.empty();
      plot_data["armor_found"] = !armors.empty();
      if (!armors.empty()) {
        const auto & armor = armors.front();
        plot_data["armor_x_px"] = armor.center.x;
        plot_data["armor_y_px"] = armor.center.y;
        plot_data["xyz_g_z"] = armor.xyz_in_gimbal.z();
        plot_data["ypd_pitch_deg"] = armor.ypd_in_world[1] * kRadToDeg;
      } else {
        plot_data["armor_x_px"] = 0.0;
        plot_data["armor_y_px"] = 0.0;
        plot_data["xyz_g_z"] = 0.0;
        plot_data["ypd_pitch_deg"] = 0.0;
      }
      if (ENABLE_VOFA_PLOTTER) vofa_plotter.plot(plot_data);
      if (ENABLE_CSV_LOGGER) csv_logger.write(plot_data);

      debug_img = img;

      auto now = std::chrono::steady_clock::now();
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
      fps_frame_count++;
      if (dt >= 1000) {
        current_fps = fps_frame_count * 1000.0 / dt;
        fps_frame_count = 0;
        last_fps_time = now;
      }

      for (const auto & armor : armors) {
        if (!armor.points.empty()) {
          tools::draw_points(debug_img, armor.points, {255, 255, 0}, 2);
          draw_armor_point_order(debug_img, armor.points);
        }

        std::string armor_info = fmt::format(
          "{:.2f} {} {} {}", armor.confidence,
          auto_aim::COLORS[armor.color],
          auto_aim::ARMOR_NAMES[armor.name],
          auto_aim::ARMOR_TYPES[armor.type]);
        tools::draw_text(debug_img, armor_info, armor.center, {0, 255, 0}, 1.0, 2);
      }

      if (!targets.empty()) {
        auto target = targets.front();

        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(debug_img, image_points, {0, 255, 0}, 2);
        }

        Eigen::Vector4d aim_xyza = planner.debug_xyza;
        auto aim_points =
          solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
        tools::draw_points(debug_img, aim_points, {0, 0, 255}, 2);

        // 显示世界坐标
        Eigen::VectorXd ekf_state = target.ekf_x();
        std::string world_text = fmt::format(
          "World: X={:.2f}m Y={:.2f}m Z={:.2f}m Yaw={:.1f}deg",
          ekf_state[0], ekf_state[2], ekf_state[4], ekf_state[6]);
        tools::draw_text(debug_img, world_text, {10, 180}, {0, 255, 255}, 0.5, 2);

        tools::draw_text(debug_img,
          fmt::format("dist={:.2f}m pitch={:.1f}deg",
          display_dist.load(), display_pitch_raw.load() * 57.3),
          {10, 205}, {255, 255, 0}, 0.5, 2);
      } else {
        tools::draw_text(debug_img, "No Target", {10, 150}, {128, 128, 128}, 0.6, 2);
        tools::draw_text(debug_img,
          fmt::format("dist={:.2f}m pitch={:.1f}deg",
          display_dist.load(), display_pitch_raw.load() * 57.3),
          {10, 205}, {255, 255, 0}, 0.5, 2);
      }

      tools::draw_text(debug_img,
        fmt::format(
          "target={} control={} fire={}",
          display_target_exist.load() ? "yes" : "no",
          display_control.load() ? "on" : "off",
          display_fire.load() ? "on" : "off"),
        {10, 230}, {0, 200, 255}, 0.8, 2);

      tools::draw_text(debug_img,
        fmt::format(
          "TX yaw={:.2f}deg  pitch={:.2f}deg",
          display_yaw_cmd.load() * 57.3,
          display_pitch_cmd.load() * 57.3),
        {10, 265}, {0, 255, 255}, 1.0, 3);

      tools::draw_text(debug_img,
        fmt::format(
          "RX yaw={:.2f}deg  pitch={:.2f}deg",
          display_rx_yaw_deg.load(),
          display_rx_pitch_deg.load()),
        {10, 305}, {255, 255, 0}, 1.0, 3);

      tools::draw_text(debug_img,
        "PTS: 0-1-2-3 should wrap armor, not cross",
        {10, 345}, {255, 255, 255}, 0.8, 2);

      tools::draw_text(debug_img,
        display_fixed_cmd.load() ? "TX MODE: FIXED" : "TX MODE: PLANNER",
        {10, 385}, display_fixed_cmd.load() ? cv::Scalar{0, 0, 255} : cv::Scalar{0, 255, 0}, 1.0, 3);

      int y_offset = 30;
      y_offset += 25;
      std::string fps_text = fmt::format("FPS: {:.1f}", current_fps);
      tools::draw_text(debug_img, fps_text, {10, y_offset}, {255, 255, 255}, 0.7, 2);

      y_offset += 30;
      Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      std::string gimbal_text = fmt::format(
        "Gimbal: Yaw {:.1f}deg Pitch {:.1f}deg",
        gimbal_pos[0] * 57.3, gimbal_pos[1] * 57.3);
      tools::draw_text(debug_img, gimbal_text, {10, y_offset}, {255, 255, 255}, 0.6, 1);

      recorder.record(debug_img, q, t);

      cv::Mat display_img;
      cv::resize(debug_img, display_img, {}, 0.75, 0.75);
      cv::imshow("QYG Sentry Debug", display_img);
      auto key = cv::waitKey(1);
      if (key == 'q' || key == 27) {
        break;
      }
    } else if (current_mode == io::GimbalMode::IDLE) {

      std::this_thread::sleep_for(50ms);
    }
  }

  quit = true;
  if (detect_thread.joinable()) detect_thread.join();
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  rclcpp::shutdown();
  return 0;
}
