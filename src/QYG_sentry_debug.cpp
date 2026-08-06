#include <atomic>
#include <chrono>
#include <fmt/core.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/ros2/aim2nav.hpp"
#include "io/ros2/armor_tf_publisher.hpp"
#include "io/ros2/nav2aim.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_sentry.yaml | 位置参数,yaml配置文件路径 }";

using namespace std::chrono_literals;

namespace
{
constexpr double kRadToDeg = 57.3;
constexpr int kDefaultGimbalPoseTimeOffsetMs = -1;
constexpr bool kObservePlannerOnly = true; // 只观察planner输出的目标状态，不观察tracker的测量状态，避免测量状态过于嘈杂

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
  auto yaml = tools::load(config_path);
  const auto fixed_cmd_vel_angular_z = tools::read<double>(yaml, "fixed_cmd_vel_angular_z");
  tools::logger()->info(
    "[Nav2Aim] fixed cmd_vel angular.z override: {:.4f}", fixed_cmd_vel_angular_z);

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
  auto debug_node = std::make_shared<rclcpp::Node>("qyg_sentry_debug_metrics");
  io::ArmorTfPublisher armor_tf_publisher(debug_node);
  auto meas_xyz_g_pub =
    debug_node->create_publisher<geometry_msgs::msg::Vector3>("/debug/meas_xyz_g", 10);
  auto meas_xyz_w_pub =
    debug_node->create_publisher<geometry_msgs::msg::Vector3>("/debug/meas_xyz_w", 10);
  auto ekf_motion_pub =
    debug_node->create_publisher<geometry_msgs::msg::Vector3>("/debug/ekf_motion", 10);
  auto tx_rx_pitch_pub =
    debug_node->create_publisher<geometry_msgs::msg::Vector3>("/debug/tx_rx_pitch", 10);
  auto target_state_pub =
    debug_node->create_publisher<geometry_msgs::msg::Vector3>("/debug/target_state", 10);
  tools::logger()->info(
    "[ROS2 Debug] publishing /debug/meas_xyz_g, /debug/meas_xyz_w, /debug/ekf_motion, /debug/tx_rx_pitch, /debug/target_state");

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
  std::atomic<bool> display_queue_empty{true};
  std::atomic<bool> display_hold_last_plan{false};
  std::atomic<bool> display_target_sample_valid{false};
  std::atomic<int> gimbal_pose_time_offset_ms{kDefaultGimbalPoseTimeOffsetMs};

  auto last_fps_time = std::chrono::steady_clock::now();
  int fps_frame_count = 0;
  double current_fps = 0.0;
  cv::Mat debug_img;

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
    double last_valid_yaw = 0.0;
    double last_valid_pitch = 0.0;
    double last_valid_dist = 0.0;
    bool has_last_valid_plan = false;

    while (!quit && rclcpp::ok()) {
      auto cmd_vel = nav2aim.get_latest_state();
      const double tx_vx = cmd_vel.linear.x;
      const double tx_vy = cmd_vel.linear.y;
      const double tx_wz = fixed_cmd_vel_angular_z;

      if (mode.load() == io::GimbalMode::AUTO_AIM && !target_queue.empty()) {
        display_queue_empty = false;
        display_hold_last_plan = false;

        auto target = target_queue.pop();
        bool target_valid = target.has_value();
        bool plan_control = false;
        bool plan_fire = false;
        double tx_yaw = last_valid_yaw;
        double tx_pitch = last_valid_pitch;
        double send_yaw = 0.0;
        double send_pitch = 0.0;

        display_target_sample_valid = target_valid;

        if (target_valid) {
          auto gs = gimbal.state();
          auto plan = planner.plan(*target, gs.bullet_speed);
          plan_control = plan.control;
          plan_fire = plan.fire;

          display_dist = std::hypot(target->ekf_x()[0], target->ekf_x()[2]);
          display_pitch_raw = plan_control ? plan.pitch : last_valid_pitch;
          display_target_exist = true;

          if (plan_control) {
            tx_yaw = plan.yaw;
            tx_pitch = plan.pitch;
            last_valid_yaw = tx_yaw;
            last_valid_pitch = tx_pitch;
            last_valid_dist = display_dist.load();
            has_last_valid_plan = true;
          } else {
            display_hold_last_plan = has_last_valid_plan;
          }
        } else {
          display_target_exist = false;
          display_hold_last_plan = has_last_valid_plan;
          display_dist = has_last_valid_plan ? last_valid_dist : 0.0;
          display_pitch_raw = has_last_valid_plan ? last_valid_pitch : 0.0;
        }

        if (!has_last_valid_plan) {
          tx_yaw = 0.0;
          tx_pitch = 0.0;
        }

        bool tx_control = kObservePlannerOnly ? false : (plan_control && target_valid);
        bool tx_fire = kObservePlannerOnly ? false : (plan_fire && target_valid);
        send_yaw = kObservePlannerOnly ? 0.0 : tx_yaw;
        send_pitch = kObservePlannerOnly ? 0.0 : tx_pitch;

        display_control = tx_control;
        display_fire = tx_fire;
        display_yaw_cmd = tx_yaw;
        display_pitch_cmd = tx_pitch;

        tools::logger()->info(
          "Send [{}] -> observe_only: {}, target_sample: {}, hold_last: {}, plan_control: {}, tx_control: {}, plan_fire: {}, tx_fire: {}, planner_yaw: {:.4f}, planner_pitch: {:.4f}, send_yaw: {:.4f}, send_pitch: {:.4f}, tx_vx: {:.4f}, tx_vy: {:.4f}, tx_wz: {:.4f}",
          kObservePlannerOnly ? "PLANNER_OBSERVE_ONLY" : (target_valid && plan_control ? "PLANNER_RAW" : "NO_PLAN"),
          kObservePlannerOnly, target_valid, display_hold_last_plan.load(), plan_control, tx_control, plan_fire, tx_fire,
          tx_yaw, tx_pitch, send_yaw, send_pitch, tx_vx, tx_vy, tx_wz);

        gimbal.send(
          tx_control, tx_fire, send_yaw, send_pitch,
          tx_vx, tx_vy, tx_wz);
        std::this_thread::sleep_for(10ms);
      } else if (mode.load() == io::GimbalMode::AUTO_AIM) {
        display_queue_empty = true;
        display_hold_last_plan = has_last_valid_plan;
        display_target_sample_valid = false;
        display_dist = has_last_valid_plan ? last_valid_dist : 0.0;
        display_pitch_raw = has_last_valid_plan ? last_valid_pitch : 0.0;
        display_control = false;
        display_fire = false;
        display_yaw_cmd = has_last_valid_plan ? last_valid_yaw : 0.0;
        display_pitch_cmd = has_last_valid_plan ? last_valid_pitch : 0.0;

        {
          static auto last_queue_empty_log = std::chrono::steady_clock::now();
          auto now_queue_empty_log = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::milliseconds>(now_queue_empty_log - last_queue_empty_log).count() >= 200) {
            tools::logger()->info(
              "Send [QUEUE_EMPTY_HOLD] -> mode: {}, hold_last: {}, control: false, fire: false, planner_yaw: {:.4f}, planner_pitch: {:.4f}, tx_vx: {:.4f}, tx_vy: {:.4f}, tx_wz: {:.4f}",
              gimbal.str(mode.load()), has_last_valid_plan,
              display_yaw_cmd.load(), display_pitch_cmd.load(),
              tx_vx, tx_vy, tx_wz);
            last_queue_empty_log = now_queue_empty_log;
          }
        }

        gimbal.send(
          false, false, 0.0f, 0.0f,
          tx_vx, tx_vy, tx_wz);
        std::this_thread::sleep_for(10ms);
      } else {
        {
          static auto last_stop_log = std::chrono::steady_clock::now();
          auto now_stop_log = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::milliseconds>(now_stop_log - last_stop_log).count() >= 200) {
            tools::logger()->info(
              "Send [STOP] -> mode: {}, target_queue_empty: {}, tx_vx: {:.4f}, tx_vy: {:.4f}, tx_wz: {:.4f}",
              gimbal.str(mode.load()), target_queue.empty(),
              tx_vx, tx_vy, tx_wz);
            last_stop_log = now_stop_log;
          }
        }
        gimbal.send(
          false, false, 0.0f, 0.0f,
          tx_vx, tx_vy, tx_wz);
        display_queue_empty = true;
        display_hold_last_plan = false;
        display_target_sample_valid = false;
        display_target_exist = false;
        display_control = false;
        display_fire = false;
        display_yaw_cmd = 0.0;
        display_pitch_cmd = 0.0;
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
      const auto pose_time_offset_ms = gimbal_pose_time_offset_ms.load();
      auto euler_deg = gimbal.euler(t + std::chrono::milliseconds(pose_time_offset_ms));
      auto q = euler_deg * kDegToRad;
      display_rx_yaw_deg = euler_deg[2];
      display_rx_pitch_deg = euler_deg[1];
      // 读取编码器原始值和底盘 IMU（用于坐标系对比验证）
      // 注意：串口协议中 vyaw/vpitch/vroll 已经是度数，不要重复转换
      auto gs = gimbal.state();
      double enc_yaw_deg = gs.vyaw;
      double enc_pitch_deg = gs.vpitch;
      double enc_roll_deg = gs.vroll;
      double chassis_imu_yaw_deg = gs.imu_yaw;
      double chassis_imu_pitch_deg = gs.imu_pitch;

      // 串扰检测：当单轴运动时检查其他轴是否出现耦合
      {
        static double prev_enc_yaw = 0.0, prev_enc_pitch = 0.0;
        static auto last_cross_log = std::chrono::steady_clock::now();
        double dy = enc_yaw_deg - prev_enc_yaw;
        double dp = enc_pitch_deg - prev_enc_pitch;
        double roll_mag = std::abs(enc_roll_deg);
        auto now_cross = std::chrono::steady_clock::now();
        bool yaw_moving = std::abs(dy) > 0.5;
        bool pitch_moving = std::abs(dp) > 0.5;
        bool single_axis = yaw_moving != pitch_moving;  // 仅一个轴在动
        if (single_axis && roll_mag > 1.0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now_cross - last_cross_log).count() >= 500) {
          tools::logger()->warn(
            "[CrossCoupling] single-axis motion detected roll={:.2f}deg! "
            "d_yaw={:+.2f} d_pitch={:+.2f} | "
            "ENC yaw={:.2f} pitch={:.2f} roll={:.2f} | "
            "RX yaw={:.2f} pitch={:.2f}",
            enc_roll_deg, dy, dp,
            enc_yaw_deg, enc_pitch_deg, enc_roll_deg,
            display_rx_yaw_deg.load(), display_rx_pitch_deg.load());
          last_cross_log = now_cross;
        }
        prev_enc_yaw = enc_yaw_deg;
        prev_enc_pitch = enc_pitch_deg;
      }

      {
        static auto last_tx_rx_log = std::chrono::steady_clock::now();
        auto now_tx_rx = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now_tx_rx - last_tx_rx_log).count() >= 200) {
          double rx_yaw = display_rx_yaw_deg.load();
          double rx_pitch = display_rx_pitch_deg.load();
          double yaw_diff = enc_yaw_deg - rx_yaw;
          double pitch_diff = enc_pitch_deg - rx_pitch;
          tools::logger()->info(
            "TX/RX -> tx_yaw={:.2f}deg tx_pitch={:.2f}deg | "
            "RX(interp) yaw={:.2f}deg pitch={:.2f}deg | "
            "ENC(raw) yaw={:.2f}deg pitch={:.2f}deg roll={:.2f}deg | "
            "ENC-RX d_yaw={:+.3f}deg d_pitch={:+.3f}deg | "
            "CHASSIS_IMU yaw={:.2f}deg pitch={:.2f}deg | "
            "control={} fire={}",
            display_yaw_cmd.load() * 57.3,
            display_pitch_cmd.load() * 57.3,
            rx_yaw, rx_pitch,
            enc_yaw_deg, enc_pitch_deg, enc_roll_deg,
            yaw_diff, pitch_diff,
            chassis_imu_yaw_deg, chassis_imu_pitch_deg,
            display_control.load(),
            display_fire.load());
          last_tx_rx_log = now_tx_rx;
        }
      }

      solver.set_R_gimbal2world(q);

      auto targets = tracker.track(armors, t);
      int armor_tf_index = 0;
      const auto armor_tf_stamp = debug_node->now();
      for (const auto & armor : armors) {
        armor_tf_publisher.publish(armor, armor_tf_stamp, armor_tf_index++);
      }

      if (!targets.empty()) {
        target_queue.push(targets.front());
      } else {
        target_queue.push(std::nullopt);
      }

      int debug_target_last_id = -1;
      bool debug_target_jumped = false;
      Eigen::Vector3d meas_xyz_g = Eigen::Vector3d::Zero();
      Eigen::Vector3d meas_xyz_w = Eigen::Vector3d::Zero();
      double meas_ypd_pitch_deg = 0.0;
      double ekf_speed_norm = 0.0;
      double ekf_abs_vz = 0.0;
      double ekf_abs_w = 0.0;
      Eigen::VectorXd ekf_state;

      if (!armors.empty()) {
        const auto & armor = armors.front();
        meas_xyz_g = armor.xyz_in_gimbal;
        meas_xyz_w = armor.xyz_in_world;
        meas_ypd_pitch_deg = armor.ypd_in_world[1] * kRadToDeg;
      }

      if (!targets.empty()) {
        const auto & target = targets.front();
        debug_target_last_id = target.last_id;
        debug_target_jumped = target.jumped;

        ekf_state = target.ekf_x();
        ekf_speed_norm = std::hypot(ekf_state[1], ekf_state[3], ekf_state[5]);
        ekf_abs_vz = std::abs(ekf_state[5]);
        ekf_abs_w = std::abs(ekf_state[7]);
      }

      geometry_msgs::msg::Vector3 meas_xyz_g_msg;
      meas_xyz_g_msg.x = meas_xyz_g.x();
      meas_xyz_g_msg.y = meas_xyz_g.y();
      meas_xyz_g_msg.z = meas_xyz_g.z();
      meas_xyz_g_pub->publish(meas_xyz_g_msg);

      geometry_msgs::msg::Vector3 meas_xyz_w_msg;
      meas_xyz_w_msg.x = meas_xyz_w.x();
      meas_xyz_w_msg.y = meas_xyz_w.y();
      meas_xyz_w_msg.z = meas_xyz_w.z();
      meas_xyz_w_pub->publish(meas_xyz_w_msg);

      geometry_msgs::msg::Vector3 ekf_motion_msg;
      ekf_motion_msg.x = ekf_speed_norm;
      ekf_motion_msg.y = ekf_abs_vz;
      ekf_motion_msg.z = ekf_abs_w;
      ekf_motion_pub->publish(ekf_motion_msg);

      geometry_msgs::msg::Vector3 tx_rx_pitch_msg;
      tx_rx_pitch_msg.x = display_pitch_cmd.load() * kRadToDeg;
      tx_rx_pitch_msg.y = display_rx_pitch_deg.load();
      tx_rx_pitch_msg.z = tx_rx_pitch_msg.x - tx_rx_pitch_msg.y;
      tx_rx_pitch_pub->publish(tx_rx_pitch_msg);

      geometry_msgs::msg::Vector3 target_state_msg;
      target_state_msg.x = display_target_sample_valid.load() ? 1.0 : 0.0;
      target_state_msg.y = display_queue_empty.load() ? 1.0 : 0.0;
      target_state_msg.z = display_hold_last_plan.load() ? 1.0 : 0.0;
      target_state_pub->publish(target_state_msg);

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

        tools::draw_text(debug_img,
          fmt::format(
            "EKF pos=({:.2f},{:.2f},{:.2f}) yaw={:.1f}deg",
            ekf_state[0], ekf_state[2], ekf_state[4], ekf_state[6] * kRadToDeg),
          {10, 180}, {0, 255, 255}, 0.5, 2);
      } else {
        tools::draw_text(debug_img, "No Target", {10, 150}, {128, 128, 128}, 0.6, 2);
      }

      tools::draw_text(debug_img,
        fmt::format(
          "G xyz=({:.3f},{:.3f},{:.3f})  pitch={:.2f}deg",
          meas_xyz_g.x(), meas_xyz_g.y(), meas_xyz_g.z(), meas_ypd_pitch_deg),
        {10, 205}, {255, 255, 0}, 0.55, 2);

      tools::draw_text(debug_img,
        fmt::format(
          "W xyz=({:.3f},{:.3f},{:.3f})",
          meas_xyz_w.x(), meas_xyz_w.y(), meas_xyz_w.z()),
        {10, 230}, {0, 255, 255}, 0.55, 2);

      tools::draw_text(debug_img,
        fmt::format(
          "EKF speed={:.4f}  |vz|={:.4f}  |w|={:.4f}",
          ekf_speed_norm, ekf_abs_vz, ekf_abs_w),
        {10, 255}, {0, 255, 0}, 0.55, 2);

      tools::draw_text(debug_img,
        fmt::format(
          "target={} sample={} q_empty={} hold={} control={} fire={}",
          display_target_exist.load() ? "yes" : "no",
          display_target_sample_valid.load() ? "yes" : "no",
          display_queue_empty.load() ? "yes" : "no",
          display_hold_last_plan.load() ? "yes" : "no",
          display_control.load() ? "on" : "off",
          display_fire.load() ? "on" : "off"),
        {10, 280}, {0, 200, 255}, 0.8, 2);

      tools::draw_text(debug_img,
        fmt::format(
          "tracker={} last_id={} jumped={}",
          tracker.state(), debug_target_last_id,
          debug_target_jumped ? "yes" : "no"),
        {10, 305}, {0, 200, 255}, 0.7, 2);

      tools::draw_text(debug_img,
        fmt::format(
          "TX yaw={:.2f}deg  pitch={:.2f}deg",
          display_yaw_cmd.load() * 57.3,
          display_pitch_cmd.load() * 57.3),
        {10, 335}, {0, 255, 255}, 1.0, 3);

      tools::draw_text(debug_img,
        fmt::format(
          "RX yaw={:.2f}deg  pitch={:.2f}deg",
          display_rx_yaw_deg.load(),
          display_rx_pitch_deg.load()),
        {10, 375}, {255, 255, 0}, 1.0, 3);

      tools::draw_text(debug_img,
        fmt::format(
          "ENC yaw={:.2f}deg  pitch={:.2f}deg  roll={:.2f}deg",
          enc_yaw_deg, enc_pitch_deg, enc_roll_deg),
        {10, 400}, {100, 255, 100}, 0.7, 2);

      tools::draw_text(debug_img,
        fmt::format(
          "CHASSIS imu_yaw={:.2f}deg  imu_pitch={:.2f}deg",
          chassis_imu_yaw_deg, chassis_imu_pitch_deg),
        {10, 425}, {180, 180, 255}, 0.7, 2);

      tools::draw_text(debug_img,
        "PTS: 0-1-2-3 should wrap armor, not cross",
        {10, 455}, {255, 255, 255}, 0.8, 2);

      tools::draw_text(debug_img,
        fmt::format(
          "POSE offset={}ms  keys: ,/. adjust  / reset",
          gimbal_pose_time_offset_ms.load()),
        {10, 485}, {255, 255, 255}, 0.65, 2);

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
      } else if (key == ',') {
        gimbal_pose_time_offset_ms--;
        tools::logger()->info(
          "[PoseTimeOffset] gimbal pose time offset: {} ms",
          gimbal_pose_time_offset_ms.load());
      } else if (key == '.') {
        gimbal_pose_time_offset_ms++;
        tools::logger()->info(
          "[PoseTimeOffset] gimbal pose time offset: {} ms",
          gimbal_pose_time_offset_ms.load());
      } else if (key == '/') {
        gimbal_pose_time_offset_ms = kDefaultGimbalPoseTimeOffsetMs;
        tools::logger()->info(
          "[PoseTimeOffset] reset gimbal pose time offset: {} ms",
          gimbal_pose_time_offset_ms.load());
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
