#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <list>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/gimbal_orientation.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/debug_box_logger.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/planner_command_adapter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
// #include "tasks/auto_buff/buff_aimer.hpp"
// #include "tasks/auto_buff/buff_detector.hpp"
// #include "tasks/auto_buff/buff_solver.hpp"
// #include "tasks/auto_buff/buff_target.hpp"
// #include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

#ifdef QYG_ROS2_DEBUG_TELEMETRY
#include <rclcpp/rclcpp.hpp>

#include "io/ros2/qyg_debug_telemetry.hpp"
#endif

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{control          | false | 向下位机发送瞄准命令；默认关闭以隔离EKF和姿态补偿}"
  "{plotjuggler      | false | 通过ROS2发布第一轮姿态补偿诊断话题}"
  "{feedback f       |       | 限频打印 Planner、串口 TX 和 RX，仅用于通信调试}"
  "{@config-path   | configs/QYG_hero.yaml | 位置参数,yaml配置文件路径 }";

using namespace std::chrono_literals;

struct AimTask
{
  std::optional<auto_aim::Target> target;
  std::chrono::steady_clock::time_point timestamp;
  Eigen::Vector3d gimbal_ypr_at_image = Eigen::Vector3d::Zero();
};

struct PlannedAimSnapshot
{
  bool valid = false;
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
  auto_aim::ArmorName name{};
  auto_aim::ArmorType armor_type{};
  bool fire_advice = false;  // 规划器给出的开火建议,仅供调试显示,不下发
  std::chrono::steady_clock::time_point computed_at{};
};

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  const bool control_enabled = cli.get<bool>("control");
  const bool plotjuggler_enabled = cli.get<bool>("plotjuggler");
  const bool packet_debug_enabled = cli.has("feedback");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  const auto config_yaml = tools::load(config_path);
  const double imu_feedback_timeout_ms = io::load_imu_feedback_timeout_ms(config_yaml);
  const int feedback_log_interval_ms =
    config_yaml["feedback_log_interval_ms"]
      ? std::max(1, config_yaml["feedback_log_interval_ms"].as<int>())
      : 200;

  tools::Exiter exiter;
  tools::Recorder recorder;
  tools::Plotter plotter;

#ifdef QYG_ROS2_DEBUG_TELEMETRY
  std::shared_ptr<io::QygDebugTelemetryPublisher> ros2_debug_publisher;
  if (plotjuggler_enabled) {
    rclcpp::init(0, nullptr);
    ros2_debug_publisher = std::make_shared<io::QygDebugTelemetryPublisher>();
  }
#else
  if (plotjuggler_enabled) {
    tools::logger()->warn(
      "QYG_hero_debug was built without ROS2 telemetry support; ROS2 telemetry is disabled");
  }
#endif

  io::Gimbal gimbal(config_path);
  gimbal.set_packet_debug_enabled(packet_debug_enabled);
  io::Camera camera(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  auto_aim::DebugBoxLogger box_logger("QYG_hero_debug_boxes");

  tools::ThreadSafeQueue<AimTask, true> target_queue(1);
  target_queue.push({std::nullopt, std::chrono::steady_clock::now(), Eigen::Vector3d::Zero()});
  std::mutex planned_aim_mutex;
  PlannedAimSnapshot planned_aim_snapshot;

  // auto_buff::Buff_Detector buff_detector(config_path);
  // auto_buff::Solver buff_solver(config_path);
  // auto_buff::SmallTarget buff_small_target;
  // auto_buff::BigTarget buff_big_target;
  // auto_buff::Aimer buff_aimer(config_path);

  std::atomic<bool> quit = false;

  std::atomic<io::Mode> mode{io::Mode::idle};
  auto last_mode{io::Mode::idle};
  // Debug模式相关变量
  auto last_fps_time = std::chrono::steady_clock::now();
  int fps_frame_count = 0;
  int total_frame_count = 0;
  double current_fps = 0.0;
  bool imu_feedback_warning_active = false;
  cv::Mat debug_img;  // 用于显示的图像
  std::mutex preview_mutex;
  cv::Mat latest_preview;

  cv::namedWindow("QYG Hero Debug", cv::WINDOW_NORMAL);
  cv::Mat startup_img(540, 960, CV_8UC3, cv::Scalar::all(0));
  tools::draw_text(
    startup_img, "Mode: IDLE | Waiting for camera and controller", {30, 50},
    {255, 255, 255}, 0.8, 2);
  cv::imshow("QYG Hero Debug", startup_img);
  cv::waitKey(1);

  // 采集线程始终更新预览；只有装甲板自瞄模式才提交检测任务。
  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!quit && !exiter.exit()) {
      camera.read(img, t);
      {
        std::lock_guard<std::mutex> lock(preview_mutex);
        latest_preview = img.clone();
      }
      // 检测线程常驻:不受电控挡位影响,始终异步检测;是否"跟随+开火"仍由
      // 主循环根据电控 auto_aim 挡位决定。结果队列有界,满了自动丢旧帧。
      detector.push(img, t);  // 异步检测
    }
  });

  // 瞄准线程：按子弹飞行时间前向预测目标，运行 TinyMPC，但保持禁火。
  auto plan_thread = std::thread([&]() {
    if (!control_enabled) return;
    bool was_auto_aim = false;
    auto last_planner_feedback_log = std::chrono::steady_clock::time_point{};
    while (!quit) {
      if (mode.load() != io::Mode::auto_aim) {
        if (was_auto_aim) gimbal.send(io::Command{false, false, 0, 0, 0});
        was_auto_aim = false;
        std::this_thread::sleep_for(1ms);
        continue;
      }
      was_auto_aim = true;
      if (target_queue.empty()) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

        const auto task = target_queue.pop();
        io::Command command{false, false, 0, 0, 0};
        if (task.target) {
          const auto plan = planner.plan(*task.target, gimbal.state().bullet_speed);
          command = auto_aim::command_from_plan(plan, task.target->name);
          const bool fire_advice = command.shoot;  // command_from_plan 已把 plan.fire 映射到 shoot
          command.shoot = false;  // 仅瞄准:fire 只作为建议用于调试显示,不下发真实开火
          command.bypass_pitch_limit = true;
#ifdef QYG_ROS2_DEBUG_TELEMETRY
          if (ros2_debug_publisher) {
            ros2_debug_publisher->publish_planner_angles(
              Eigen::Vector2d(plan.target_yaw, plan.target_pitch),
              Eigen::Vector2d(command.yaw, command.pitch));
          }
#endif

        const Eigen::VectorXd x = task.target->ekf_x();
        command.horizon_distance = std::hypot(x[0], x[2]);

        {
          std::lock_guard<std::mutex> lock(planned_aim_mutex);
          planned_aim_snapshot.valid = command.control;
          planned_aim_snapshot.xyza = planner.debug_xyza;
          planned_aim_snapshot.name = task.target->name;
          planned_aim_snapshot.armor_type = task.target->armor_type;
          planned_aim_snapshot.fire_advice = fire_advice;
          planned_aim_snapshot.computed_at = std::chrono::steady_clock::now();
        }

        const auto feedback_log_now = std::chrono::steady_clock::now();
        if (
          packet_debug_enabled &&
          feedback_log_now - last_planner_feedback_log >=
            std::chrono::milliseconds{feedback_log_interval_ms}) {
          last_planner_feedback_log = feedback_log_now;
          tools::logger()->debug(
            "MPC with flight-time prediction - fly_time={:.3f}s armor_id={} control={} fire={} "
            "target_yaw={:.2f}deg target_pitch={:.2f}deg tx_yaw={:.2f}deg "
            "tx_pitch={:.2f}deg tx_yaw_vel={:.2f}deg/s tx_pitch_vel={:.2f}deg/s",
            planner.debug_fly_time, planner.debug_armor_id, command.control, fire_advice,
            plan.target_yaw * 57.3, plan.target_pitch * 57.3, command.yaw * 57.3,
            command.pitch * 57.3, command.yaw_vel * 57.3, command.pitch_vel * 57.3);
        }
      } else {
        std::lock_guard<std::mutex> lock(planned_aim_mutex);
        planned_aim_snapshot = {};
      }

      gimbal.send(command);
    }
  });

  while (!exiter.exit()) {
    // 仍以电控 auto_aim 挡位作为"跟随+开火建议"的总开关;检测线程不受影响、常驻运行。
    const auto gimbal_state = gimbal.state();
    mode = gimbal_state.mode;
    nlohmann::json data;
    auto current_mode = mode.load();  // 缓存模式值，避免重复调用load()
    const char * own_color_str = gimbal_state.own_color == io::OwnColor::red
                                   ? "red"
                                   : gimbal_state.own_color == io::OwnColor::blue ? "blue"
                                                                                  : "unknown";
    const double bullet_speed = gimbal_state.bullet_speed;

    if (last_mode != current_mode) {
      if (last_mode == io::Mode::auto_aim) target_queue.clear();
      tools::logger()->info("Switch to {}", io::MODES[current_mode]);
      last_mode = current_mode;
    }

    /// 自瞄
    if (current_mode == io::Mode::auto_aim) {
      // 从检测队列获取结果（异步检测已完成）
      auto [img, armors, t] = detector.debug_pop();
      const auto feedback = gimbal.gimbal_feedback_for_image_at(t);
      const auto feedback_check_time = std::chrono::steady_clock::now();
      const bool feedback_live =
        feedback &&
        io::feedback_stream_is_live(*feedback, feedback_check_time, imu_feedback_timeout_ms);
      if (!feedback_live) {
        if (!imu_feedback_warning_active) {
          if (feedback) {
            const double feedback_age_ms =
              std::chrono::duration<double, std::milli>(
                feedback_check_time - feedback->newest_feedback_timestamp)
                .count();
            tools::logger()->warn(
              "[QYG_hero_debug] IMU feedback timed out (newest {:.0f} ms ago, timeout {:.0f} ms); "
              "tracking and control paused",
              feedback_age_ms, imu_feedback_timeout_ms);
          } else {
            tools::logger()->warn(
              "[QYG_hero_debug] IMU feedback unavailable; tracking and control paused");
          }
          tracker.reset();
          if (control_enabled) {
            target_queue.clear();
            target_queue.push({std::nullopt, t, Eigen::Vector3d::Zero()});
          }
        }
        imu_feedback_warning_active = true;
        debug_img = img;
        tools::draw_text(
          debug_img, "Mode: AUTO_AIM | IMU feedback unavailable", {10, 30}, {0, 0, 255}, 0.7,
          2);
        cv::Mat display_img;
        cv::resize(debug_img, display_img, {}, 0.5, 0.5);
        cv::imshow("QYG Hero Debug", display_img);
        const auto key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        continue;
      }
      if (imu_feedback_warning_active) {
        tools::logger()->info(
          "[QYG_hero_debug] IMU feedback recovered; tracker restarted in detecting state");
        imu_feedback_warning_active = false;
      }
      solver.set_R_gimbal2world(*feedback);

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      data["gimbal_yaw"] = ypr[0] * 57.3;
      data["gimbal_pitch"] = ypr[1] * 57.3;

      const auto enemy_color = gimbal.enemy_color_string();
      if (enemy_color) {
        tracker.set_enemy_color(*enemy_color);
      }
      auto targets = tracker.track(armors, t);
      const bool TuoLuo = tracker.TuoLuo();
      const auto diagnostic_target = tracker.diagnostic_target();
      const auto tracker_state = tracker.state();
      data["tracker_state"] = tracker_state == "lost"        ? 0.0
                              : tracker_state == "detecting" ? 1.0
                              : tracker_state == "tracking"  ? 2.0
                              : tracker_state == "temp_lost" ? 3.0
                                                             : 4.0;
      data["tracker_detecting"] = tracker_state == "detecting" ? 1.0 : 0.0;
      data["tracker_tracking"] = tracker_state == "tracking" ? 1.0 : 0.0;
      data["tracker_temp_lost"] = tracker_state == "temp_lost" ? 1.0 : 0.0;
      data["TuoLuo"] = TuoLuo ? 1.0 : 0.0;
      if (control_enabled) {
        if (!targets.empty())
          target_queue.push({targets.front(), t, ypr});
        else
          target_queue.push({std::nullopt, t, ypr});
      }

      // 使用 Tracker 实际提交给 EKF 的观测，避免多候选时诊断曲线引用另一块装甲板。
      const auto measured_xyz = tracker.latest_armor_xyz_in_world();
      if (diagnostic_target) {
        const Eigen::VectorXd state = diagnostic_target->ekf_x();
        if (state.size() >= 8) {
          data["ekf_center_vx_mps"] = state[1];
          data["ekf_center_vy_mps"] = state[3];
          data["ekf_center_vz_mps"] = state[5];
          data["ekf_rotation_phase_rad"] = state[6];
          data["ekf_rotation_speed_radps"] = state[7];
        }
        const auto & ekf_data = diagnostic_target->ekf().data;
        for (const auto & [key, value] : ekf_data) data["ekf_" + key] = value;
        for (const auto * key :
             {"prediction_only", "temp_lost_count", "max_temp_lost_count", "observation_accepted",
              "match_no_candidate", "match_rejected"}) {
          if (const auto it = ekf_data.find(key); it != ekf_data.end()) data[key] = it->second;
        }
        if (ekf_data.count("recent_nis_failures")) {
          data["ekf_nis_fail_rate"] = ekf_data.at("recent_nis_failures");
        }
      }
      if (measured_xyz) {
        const Eigen::Vector3d measured_ypd = tools::xyz2ypd(*measured_xyz);
        data["measurement_world_x_m"] = (*measured_xyz)[0];
        data["measurement_world_y_m"] = (*measured_xyz)[1];
        data["measurement_world_z_m"] = (*measured_xyz)[2];
        data["measurement_world_yaw_deg"] = measured_ypd[0] * 57.3;
        data["measurement_world_pitch_deg"] = measured_ypd[1] * 57.3;
        data["measurement_world_distance_m"] = measured_ypd[2];
      }
      // Debug模式：绘制识别画面和信息
      // 调试绘制使用独立副本，保留 img 作为摄像头原始画面供录制。
      debug_img = img.clone();

      // 计算FPS
      auto now = std::chrono::steady_clock::now();
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
      fps_frame_count++;
      if (dt >= 1000) {  // 每秒更新一次FPS
        current_fps = fps_frame_count * 1000.0 / dt;
        fps_frame_count = 0;
        last_fps_time = now;
      }

      // 绘制检测到的装甲板
      int detect_box_index = 0;
      for (const auto & armor : armors) {
        // 绘制装甲板四个角点
        if (!armor.points.empty()) {
          tools::draw_points(debug_img, armor.points, {255, 0, 0}, 2);
        }

        // 绘制装甲板信息
        std::string armor_info = fmt::format(
          "{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color],
          auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type]);
        box_logger.log_box(
          "blue", "detection", total_frame_count, t, detect_box_index++, armor.points, armor_info);
        tools::draw_text(debug_img, armor_info, armor.center, {255, 0, 0}, 0.5, 1);
      }

      // 绘制目标跟踪信息
      if (diagnostic_target) {
        const auto & target = *diagnostic_target;

        // 普通板绿色、当前板黄色、候选板紫色。
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        int ekf_box_index = 0;
        for (std::size_t armor_id = 0; armor_id < armor_xyza_list.size(); ++armor_id) {
          const Eigen::Vector4d & xyza = armor_xyza_list[armor_id];
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          const bool is_current = static_cast<int>(armor_id) == target.last_id;
          const bool is_candidate =
            static_cast<int>(armor_id) == target.switch_candidate_id();
          const cv::Scalar armor_color = is_current
                                           ? cv::Scalar{0, 255, 255}
                                           : is_candidate ? cv::Scalar{255, 0, 255}
                                                          : cv::Scalar{0, 255, 0};
          tools::draw_points(debug_img, image_points, armor_color, is_current ? 3 : 2);
          if (!image_points.empty()) {
            cv::Point2f anchor{0.0F, 0.0F};
            for (const auto & point : image_points) anchor += point;
            anchor *= 1.0F / static_cast<float>(image_points.size());
            const cv::Point label_origin{
              static_cast<int>(anchor.x), static_cast<int>(anchor.y) - 8};
            tools::draw_text(
              debug_img, fmt::format("ID {}", armor_id), label_origin, armor_color, 0.7, 2);
            if (is_current) {
              tools::draw_text(
                debug_img, "CURRENT", {label_origin.x, label_origin.y + 20}, armor_color, 0.6, 2);
            } else if (is_candidate) {
              tools::draw_text(
                debug_img, "CANDIDATE", {label_origin.x, label_origin.y + 20}, armor_color, 0.6,
                2);
            }
          }
          box_logger.log_box(
            "green", "ekf_reproject", total_frame_count, t, ekf_box_index++, image_points,
            fmt::format(
              "{} {} xyza=({:.3f},{:.3f},{:.3f},{:.3f})", auto_aim::ARMOR_NAMES[target.name],
              auto_aim::ARMOR_TYPES[target.armor_type], xyza[0], xyza[1], xyza[2], xyza[3]));
        }
        const std::string candidate_text = target.switch_candidate_id() >= 0
                                             ? std::to_string(target.switch_candidate_id())
                                             : "-";
        tools::draw_text(
          debug_img,
          fmt::format(
            "Armor ID current={} candidate={} ({}/{})", target.last_id, candidate_text,
            target.switch_candidate_count(), target.switch_confirm_frames()),
          {10, 150}, {255, 255, 0}, 0.65, 2);

        PlannedAimSnapshot planned_aim;
        {
          std::lock_guard<std::mutex> lock(planned_aim_mutex);
          planned_aim = planned_aim_snapshot;
        }
        const auto planned_aim_age = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - planned_aim.computed_at)
                                       .count();
        if (
          planned_aim.valid && planned_aim_age >= 0.0 && planned_aim_age <= 250.0 &&
          planned_aim.name == target.name && planned_aim.armor_type == target.armor_type) {
          const auto planner_image_points = solver.reproject_armor(
            planned_aim.xyza.head<3>(), planned_aim.xyza[3], target.armor_type, target.name);
          tools::draw_points(debug_img, planner_image_points, {0, 0, 255}, 3);
          box_logger.log_box(
            "red", "planner_aim", total_frame_count, t, 0, planner_image_points,
            fmt::format(
              "{} {} xyza=({:.3f},{:.3f},{:.3f},{:.3f})", auto_aim::ARMOR_NAMES[target.name],
              auto_aim::ARMOR_TYPES[target.armor_type], planned_aim.xyza[0], planned_aim.xyza[1],
              planned_aim.xyza[2], planned_aim.xyza[3]));
          // 开火建议仅用于调试显示(不真实下发真实开火命令)
          tools::draw_text(
            debug_img, planned_aim.fire_advice ? "Fire advice: YES" : "Fire advice: NO",
            {10, 190}, planned_aim.fire_advice ? cv::Scalar{0, 0, 255} : cv::Scalar{0, 255, 0}, 0.6,
            1);
        }

        // 显示目标信息
        // Eigen::VectorXd x = target.ekf_x();
        // double distance = std::sqrt(x[0] * x[0] + x[2] * x[2] + x[4] * x[4]);
        // std::string target_info = fmt::format(
        //   "Target: {} | Dist: {:.2f}m | Yaw: {:.1f}deg | W: {:.2f}rad/s",
        //   auto_aim::ARMOR_NAMES[target.name], distance, x[6] * 57.3, x[7]);
        // tools::draw_text(debug_img, target_info, {10, 150}, {255, 255, 0},
        // 0.6, 2);
      } else {
        // 没有目标时显示提示
        tools::draw_text(debug_img, "No Target", {10, 150}, {128, 128, 128}, 0.6, 2);
      }

      // 显示模式、FPS和装甲板数量
      int y_offset = 30;
      tools::draw_text(
        debug_img,
        fmt::format(
          "Mode: AUTO_AIM | Own color: {} | Bullet: {:.1f} m/s", own_color_str, bullet_speed),
        {10, y_offset}, {255, 255, 255}, 0.65, 2);
      y_offset += 30;
      const std::string tuoluo_text = fmt::format("TuoLuo: {}", TuoLuo ? 1 : 0);
      tools::draw_text(
        debug_img, tuoluo_text, {10, y_offset},
        TuoLuo ? cv::Scalar{0, 255, 0} : cv::Scalar{255, 255, 255}, 0.7, 2);
      // std::string mode_text = fmt::format("Mode: {}",
      // io::MODES[current_mode]); tools::draw_text(debug_img, mode_text, {10,
      // y_offset}, {255, 255, 255}, 0.7, 2);

      // y_offset += 30;
      // std::string tracker_state_text = fmt::format("Tracker: {}",
      // tracker.state()); tools::draw_text(debug_img, tracker_state_text, {10,
      // y_offset}, {255, 255, 255}, 0.6, 1);

      y_offset += 30;
      std::string fps_text = fmt::format("FPS: {:.1f}", current_fps);
      tools::draw_text(debug_img, fps_text, {10, y_offset}, {255, 255, 255}, 0.7, 2);

      // y_offset += 30;
      // std::string armor_count_text = fmt::format("Armors: {}",
      // armors.size()); tools::draw_text(debug_img, armor_count_text, {10,
      // y_offset}, {255, 255, 255}, 0.7, 2);

      // 显示云台和命令信息
      y_offset += 30;
      Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      std::string gimbal_text = fmt::format(
        "Gimbal: Yaw {:.1f}deg Pitch {:.1f}deg", gimbal_pos[0] * 57.3, -gimbal_pos[1] * 57.3);
      tools::draw_text(debug_img, gimbal_text, {10, y_offset}, {255, 255, 255}, 0.6, 1);

      // y_offset += 25;
      // std::string bullet_text = fmt::format("Bullet Speed: {:.1f} m/s",
      // gimbal.bullet_speed); tools::draw_text(debug_img, bullet_text, {10,
      // y_offset}, {255, 255, 255}, 0.6, 1);

      // 保存绘制了检测框和调试信息的画面，同时记录对应的 IMU 姿态。
      // EKF 的目标旋转相位/角速度单独写入同一 session 下的 ekf.txt，
      // 与录制帧保持相同的时间基准；没有有效目标时写 nan nan。
      std::optional<Eigen::Vector2d> ekf_rotation;
      if (diagnostic_target) {
        const Eigen::VectorXd state = diagnostic_target->ekf_x();
        if (state.size() > 7 && std::isfinite(state[6]) && std::isfinite(state[7])) {
          ekf_rotation = Eigen::Vector2d{state[6], state[7]};
        }
      }
      recorder.record(
        img, debug_img, feedback->raw_q, t, ekf_rotation,
        std::optional<double>{feedback->feedback_yaw_deg}, bullet_speed, enemy_color);

      // 显示图像（缩小尺寸以提高性能）
      cv::Mat display_img;
      cv::resize(debug_img, display_img, {}, 0.5, 0.5);
      cv::imshow("QYG Hero Debug", display_img);
      total_frame_count++;
      auto key = cv::waitKey(1);
      if (key == 'q' || key == 27) {  // 'q' 或 ESC 退出
        break;
      }
    }

    // /// 打符
    // else if (mode.load() == io::Mode::small_buff || mode.load() ==
    // io::Mode::big_buff) {
    //   buff_solver.set_R_gimbal2world(q);

    //   auto power_runes = buff_detector.detect(img);

    //   buff_solver.solve(power_runes);

    //   auto_aim::Plan buff_plan;
    //   if (mode.load() == io::Mode::small_buff) {
    //     buff_small_target.get_target(power_runes, t);
    //     auto target_copy = buff_small_target;
    //     buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
    //   } else if (mode.load() == io::Mode::big_buff) {
    //     buff_big_target.get_target(power_runes, t);
    //     auto target_copy = buff_big_target;
    //     buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
    //   }
    //   gimbal.send(io::command(plan.control,
    //   plan.fire,plan.yaw,plan.pitch,plan.horizon_distance));
    // }

    else {
      // 非自瞄(idle/buff)也消费检测线程的产出,并叠加装甲板框作为预览:
      // 让检测线程始终被消费、切到 auto_aim 时不会处理积压的旧帧。
      cv::Mat det_img;
      std::list<auto_aim::Armor> det_armors;
      std::chrono::steady_clock::time_point det_t;
      const bool got_detection = detector.debug_pop_for(det_img, det_armors, det_t, 40ms);

      cv::Mat preview;
      if (got_detection) {
        preview = det_img;
        for (const auto & armor : det_armors) {
          if (!armor.points.empty()) {
            tools::draw_points(preview, armor.points, {255, 0, 0}, 2);
          }
          const std::string armor_info = fmt::format(
            "{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color],
            auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type]);
          tools::draw_text(preview, armor_info, armor.center, {255, 0, 0}, 0.5, 1);
        }
      } else {
        std::lock_guard<std::mutex> lock(preview_mutex);
        if (!latest_preview.empty()) preview = latest_preview.clone();
        if (preview.empty()) preview = cv::Mat(540, 960, CV_8UC3, cv::Scalar::all(0));
      }

      const char * mode_text = current_mode == io::Mode::small_buff ? "SMALL_BUFF"
                               : current_mode == io::Mode::big_buff ? "BIG_BUFF"
                                                                   : "IDLE";
      const cv::Scalar mode_color = current_mode == io::Mode::idle
                                      ? cv::Scalar{255, 255, 255}
                                      : cv::Scalar{0, 255, 255};
      tools::draw_text(
        preview,
        fmt::format(
          "Mode: {} | Own color: {} | Bullet: {:.1f} m/s", mode_text, own_color_str,
          bullet_speed),
        {10, 30}, mode_color, 0.7, 2);
      if (current_mode == io::Mode::small_buff || current_mode == io::Mode::big_buff) {
        tools::draw_text(
          preview, "Task visualization is not connected", {10, 65}, {0, 255, 255}, 0.65, 2);
      }

      cv::Mat display_img;
      cv::resize(preview, display_img, {}, 0.5, 0.5);
      cv::imshow("QYG Hero Debug", display_img);
      const auto key = cv::waitKey(1);
      if (key == 'q' || key == 27) break;
      std::this_thread::sleep_for(10ms);
    }
    // ROS2 PlotJuggler 模式只发布选定的 tx/rx/error 与 EKF 初始化曲线，
    // 避免 UDP JSON 再产生一套未筛选的曲线。
    if (!plotjuggler_enabled) plotter.plot(data);
  }

  quit = true;
  if (detect_thread.joinable()) detect_thread.join();
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(io::Command{false, false, 0, 0, 0});

#ifdef QYG_ROS2_DEBUG_TELEMETRY
  ros2_debug_publisher.reset();
  if (plotjuggler_enabled && rclcpp::ok()) rclcpp::shutdown();
#endif

  return 0;
}
