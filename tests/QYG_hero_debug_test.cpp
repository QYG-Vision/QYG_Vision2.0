// 离线回放 QYG_hero_debug 的核心流程：视频帧 + imu.txt -> 检测、PnP、Tracker/EKF、Planner。
// 修改下面三个常量即可更换测试数据，不需要改动后面的处理逻辑。
#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/planner_command_adapter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "io/gimbal_orientation.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

// ======== 日后更换录制数据时，优先修改这里 ========
static const std::string VIDEO_PATH = "records/2026-09-07_21-58-22/raw.avi";
static const std::string IMU_PATH = "records/2026-09-07_21-58-22/imu.txt";
// Online QYG_hero_debug stores the yaw feedback fused with raw_q here.
// Older recordings may not have this sidecar; they fall back to raw_q replay.
static const std::string GIMBAL_FEEDBACK_PATH =
  "records/2026-09-07_21-58-22/gimbal_feedback.txt";
static const std::string RUNTIME_PATH = "records/2026-09-07_21-58-22/runtime.txt";
static const std::string CONFIG_PATH = "configs/QYG_hero.yaml";
// 0 表示从头播放；大于 0 时限制最后一帧，便于快速定位问题。
static constexpr int START_FRAME = 0;
static constexpr int END_FRAME = 0;
// 是否保存离线推理效果视频。false 时只显示，不创建 effect 文件夹和视频。
static constexpr bool SAVE_EFFECT_VIDEO = true;
static const std::string EFFECT_DIR = "effect";

// imu.txt 由 Recorder 写出：相对起始时间（秒） qw qx qy qz。
struct ImuSample {
  double t_sec;
  Eigen::Quaterniond q;
};

int main()
{
  cv::VideoCapture video(VIDEO_PATH);
  if (!video.isOpened()) {
    tools::logger()->error("无法打开视频: {}", VIDEO_PATH);
    return 1;
  }
  std::ifstream imu_file(IMU_PATH);
  if (!imu_file) {
    tools::logger()->error("无法打开 IMU 文件: {}", IMU_PATH);
    return 1;
  }
  std::ifstream feedback_file(GIMBAL_FEEDBACK_PATH);
  const bool has_feedback_sidecar = static_cast<bool>(feedback_file);
  if (!has_feedback_sidecar) {
    tools::logger()->warn(
      "未找到姿态反馈文件 {}，将退化为仅回放 imu.txt 中的 raw_q", GIMBAL_FEEDBACK_PATH);
  }
  std::ifstream runtime_file(RUNTIME_PATH);
  const bool has_runtime_sidecar = static_cast<bool>(runtime_file);

  auto_aim::YOLO yolo(CONFIG_PATH, true);
  auto_aim::Solver solver(CONFIG_PATH);
  auto_aim::Tracker tracker(CONFIG_PATH, solver);
  auto_aim::Planner planner(CONFIG_PATH);
  tools::Plotter plotter;
  tools::Exiter exiter;

  // 跳到起始帧，同时消费对应数量的 IMU 行，保证视频和 IMU 对齐。
  video.set(cv::CAP_PROP_POS_FRAMES, START_FRAME);
  for (int i = 0; i < START_FRAME; ++i) {
    ImuSample unused;
    double qw, qx, qy, qz;
    if (!(imu_file >> unused.t_sec >> qw >> qx >> qy >> qz)) {
      tools::logger()->error("IMU 行数不足，无法跳到起始帧 {}", START_FRAME);
      return 1;
    }
    if (has_feedback_sidecar) {
      double feedback_t, feedback_qw, feedback_qx, feedback_qy, feedback_qz, feedback_yaw_deg;
      if (!(feedback_file >> feedback_t >> feedback_qw >> feedback_qx >> feedback_qy >>
            feedback_qz >> feedback_yaw_deg)) {
        tools::logger()->error("姿态反馈行数不足，无法跳到起始帧 {}", START_FRAME);
        return 1;
      }
    }
    if (has_runtime_sidecar) {
      double runtime_t, runtime_bullet_speed;
      std::string runtime_enemy_color;
      if (!(runtime_file >> runtime_t >> runtime_bullet_speed >> runtime_enemy_color)) {
        tools::logger()->error("运行参数行数不足，无法跳到起始帧 {}", START_FRAME);
        return 1;
      }
    }
  }

  const auto t0 = std::chrono::steady_clock::now();
  cv::Mat frame;
  cv::VideoWriter raw_effect_writer;
  cv::VideoWriter debug_effect_writer;
  bool effect_writer_initialized = false;
  for (int frame_index = START_FRAME; !exiter.exit(); ++frame_index) {
    if (END_FRAME > 0 && frame_index > END_FRAME) break;
    if (!video.read(frame) || frame.empty()) break;

    double t_sec, qw, qx, qy, qz;
    if (!(imu_file >> t_sec >> qw >> qx >> qy >> qz)) {
      tools::logger()->warn("第 {} 帧没有对应 IMU，停止回放", frame_index);
      break;
    }
    double feedback_t = t_sec;
    double feedback_qw = qw, feedback_qx = qx, feedback_qy = qy, feedback_qz = qz;
    double feedback_yaw_deg = std::numeric_limits<double>::quiet_NaN();
    if (
      has_feedback_sidecar &&
      !(feedback_file >> feedback_t >> feedback_qw >> feedback_qx >> feedback_qy >> feedback_qz >>
        feedback_yaw_deg)) {
      tools::logger()->warn("第 {} 帧没有对应姿态反馈，停止回放", frame_index);
      break;
    }
    double bullet_speed = 23.0;
    std::string recorded_enemy_color;
    if (has_runtime_sidecar) {
      double runtime_t;
      if (!(runtime_file >> runtime_t >> bullet_speed >> recorded_enemy_color)) {
        tools::logger()->warn("第 {} 帧没有对应运行参数，停止回放", frame_index);
        break;
      }
      if (std::isfinite(bullet_speed) && bullet_speed > 0.0) {
        // Keep the recorded value.
      } else {
        bullet_speed = 23.0;
      }
    }
    const auto timestamp = t0 + std::chrono::microseconds{static_cast<long long>(t_sec * 1e6)};
    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (q.norm() < 1e-9) {
      tools::logger()->warn("第 {} 帧四元数无效，跳过", frame_index);
      continue;
    }
    q.normalize();
    Eigen::Quaterniond feedback_q(feedback_qw, feedback_qx, feedback_qy, feedback_qz);
    if (feedback_q.norm() < 1e-9) feedback_q = q;
    else feedback_q.normalize();

    // 优先复现在线实际使用的 fused feedback（raw_q + feedback yaw）；
    // 老录制没有 sidecar 时退化为原来的 raw_q 路径。
    if (has_feedback_sidecar && std::isfinite(feedback_yaw_deg)) {
      io::GimbalFeedbackSample feedback{
        feedback_q, feedback_yaw_deg, timestamp, timestamp};
      solver.set_R_gimbal2world(feedback);
    } else {
      solver.set_R_gimbal2world(q);
    }
    auto armors = yolo.detect(frame, frame_index);
    if (recorded_enemy_color == "red" || recorded_enemy_color == "blue") {
      tracker.set_enemy_color(recorded_enemy_color);
    }
    auto targets = tracker.track(armors, timestamp);

    io::Command command{};
    auto_aim::Plan plan{false};
    if (!targets.empty()) {
      // 离线回放不向真实云台发送指令，只计算 Planner 输出观察效果。
      plan = planner.plan(targets.front(), bullet_speed);
      command = auto_aim::command_from_plan(plan, targets.front().name);
    }

    cv::Mat display = frame.clone();  // 原始 frame 不修改，只在副本上绘制调试信息。
    for (const auto & armor : armors) {
      tools::draw_points(display, armor.points, {255, 0, 0}, 2);
      tools::draw_text(
        display, fmt::format("{:.2f} {}", armor.confidence, auto_aim::ARMOR_NAMES[armor.name]),
        armor.center, {255, 0, 0}, 0.5, 1);
    }

    nlohmann::json data;
    data["frame"] = frame_index;
    data["time_sec"] = t_sec;
    data["armor_count"] = armors.size();
    data["tracker_state"] = tracker.state();
    data["command_yaw_deg"] = command.yaw * 180.0 / CV_PI;
    data["command_pitch_deg"] = command.pitch * 180.0 / CV_PI;

    if (!targets.empty()) {
      auto target = targets.front();
      const auto x = target.ekf_x();
      // 记录完整 11 维 EKF 状态，便于后续离线分析。
      data["ekf_x"] = x[0]; data["ekf_vx"] = x[1];
      data["ekf_y"] = x[2]; data["ekf_vy"] = x[3];
      data["ekf_z"] = x[4]; data["ekf_vz"] = x[5];
      data["ekf_yaw_deg"] = x[6] * 180.0 / CV_PI; data["ekf_w_radps"] = x[7];
      data["ekf_r"] = x[8]; data["ekf_l"] = x[9]; data["ekf_h"] = x[10];

      const auto & diagnostics = target.ekf().data;
      for (const char * key : {"nis", "nis_fail", "recent_nis_failures",
                               "observation_accepted", "prediction_only", "match_rejected"}) {
        if (auto it = diagnostics.find(key); it != diagnostics.end()) data[key] = it->second;
      }

      // 画 EKF 预测出的各装甲板（绿色）和 Planner 目标（红色）。
      for (const auto & xyza : target.armor_xyza_list()) {
        tools::draw_points(
          display, solver.reproject_armor(xyza.head<3>(), xyza[3], target.armor_type, target.name),
          {0, 255, 0}, 2);
      }
      if (planner.debug_xyza.allFinite()) {
        tools::draw_points(
          display, solver.reproject_armor(
                      planner.debug_xyza.head<3>(), planner.debug_xyza[3], target.armor_type,
                      target.name),
          {0, 0, 255}, 3);
      }
    }

    plotter.plot(data);
    tools::draw_text(
      display, fmt::format("frame={} tracker={} armors={} EKF={}", frame_index, tracker.state(),
                          armors.size(), targets.empty() ? "none" : "valid"),
      {10, 30}, {255, 255, 255}, 0.7, 2);
    cv::Mat preview;
    cv::resize(display, preview, {}, 0.5, 0.5);
    cv::imshow("QYG hero offline debug", preview);

    // 保存两个视频：raw_effect.avi 为原始画面，debug_effect.avi 为叠加推理结果的画面。
    // Writer 在第一帧到达后初始化，以确保分辨率与实际视频一致。
    if (SAVE_EFFECT_VIDEO) {
      if (!effect_writer_initialized) {
        std::filesystem::create_directories(EFFECT_DIR);
        const double fps = video.get(cv::CAP_PROP_FPS) > 1.0 ? video.get(cv::CAP_PROP_FPS) : 30.0;
        const auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        raw_effect_writer.open(EFFECT_DIR + "/raw_effect.avi", fourcc, fps, frame.size());
        debug_effect_writer.open(EFFECT_DIR + "/debug_effect.avi", fourcc, fps, display.size());
        if (!raw_effect_writer.isOpened() || !debug_effect_writer.isOpened()) {
          tools::logger()->error("无法创建效果视频，已关闭视频保存");
          raw_effect_writer.release();
          debug_effect_writer.release();
        } else {
          effect_writer_initialized = true;
          tools::logger()->info("效果视频保存到 {}", EFFECT_DIR);
        }
      }
      if (effect_writer_initialized) {
        raw_effect_writer.write(frame);
        debug_effect_writer.write(display);
      }
    }

    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
  }
  raw_effect_writer.release();
  debug_effect_writer.release();
  return 0;
}
