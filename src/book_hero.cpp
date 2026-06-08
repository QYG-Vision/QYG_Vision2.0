#include <fmt/chrono.h>
#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/debug_overlay.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_hero.yaml | 位置参数,yaml配置文件路径 }"
  "{tradition t    | false                 | 是否使用传统视觉识别}"
  "{fps            | 60.0                  | 绘制后调试视频保存帧率}"
  "{output o       | records/QYG_hero_debug_QLF.avi | 绘制后调试视频保存路径}"
  "{display d      | true                  | 是否显示实时调试窗口}";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  // 获取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto use_tradition = cli.get<bool>("tradition");
  auto fps = cli.get<double>("fps");
  auto output_path = cli.get<std::string>("output");
  auto display = cli.get<bool>("display");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  {
    auto now_stamp = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    std::filesystem::path output_path_obj(output_path);
    auto parent = output_path_obj.parent_path();
    auto stem = output_path_obj.stem().string();
    auto ext = output_path_obj.extension().string();
    if (ext.empty()) ext = ".avi";
    auto timestamp_name = fmt::format("{}_{}{}", stem, now_stamp, ext);
    if (!parent.empty()) std::filesystem::create_directories(parent);
    output_path = parent.empty() ? timestamp_name : (parent / timestamp_name).string();
  }

  tools::Exiter exiter;
  tools::Recorder recorder(fps, "QYG_hero_debug_QLF_raw");
  tools::Plotter plotter;
  cv::VideoWriter debug_writer;
  const auto video_segment_duration = std::chrono::minutes(3);
  auto video_segment_start = std::chrono::steady_clock::now();
  int video_segment_index = 0;
  const auto base_output_path = output_path;
  auto make_segment_path = [&]() {
    std::filesystem::path path_obj(base_output_path);
    auto parent = path_obj.parent_path();
    auto stem = path_obj.stem().string();
    auto ext = path_obj.extension().string();
    if (ext.empty()) ext = ".avi";
    auto name = fmt::format("{}_part{:03d}{}", stem, video_segment_index, ext);
    return parent.empty() ? name : (parent / name).string();
  };
  output_path = make_segment_path();

  tools::logger()->info("QYG_hero_debug_QLF 启动");
  tools::logger()->info("配置文件: {}", config_path);
  tools::logger()->info("绘制后视频: {}", output_path);
  tools::logger()->info("识别模式: {}", use_tradition ? "traditional" : "yolo");
  tools::logger()->info("显示窗口: {}", display ? "on" : "off");

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::Detector traditional_detector(config_path, true);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  tracker.set_enemy_color(cboard.enemy_color_string());
  tools::logger()->info("敌方颜色过滤: {}", cboard.enemy_color_string());

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  int idle_counter = 0;

  auto last_fps_time = std::chrono::steady_clock::now();
  int fps_frame_count = 0;
  int total_frame_count = 0;
  double current_fps = 0.0;
  double last_detect_ms = 0.0;

  while (!exiter.exit()) {
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    if (mode == io::Mode::auto_aim) {
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);
      if (img.empty()) {
        tools::logger()->warn("QLF 拿到空帧，跳过本轮。");
        std::this_thread::sleep_for(50ms);
        continue;
      }

      auto q = cboard.imu_at(timestamp);
      recorder.record(img, q, timestamp);
      solver.set_R_gimbal2world(q);

      auto detect_start = std::chrono::steady_clock::now();
      std::list<auto_aim::Armor> armors;
      if (use_tradition)
        armors = traditional_detector.detect(img, total_frame_count);
      else
        armors = yolo.detect(img, total_frame_count);
      auto detect_end = std::chrono::steady_clock::now();
      last_detect_ms = tools::delta_time(detect_end, detect_start) * 1000.0;

      auto targets = tracker.track(armors, timestamp);

      io::Command command{false, false, 0, 0, 0};
      std::optional<Eigen::Vector4d> aim_xyza;
      auto_aim::Plan plan{false};
      if (!targets.empty()) {
        auto target = targets.front();
        plan = planner.plan(target, cboard.bullet_speed);
        // command = {plan.control, plan.fire, plan.yaw, plan.pitch, 0};
        command = {1, plan.fire, plan.yaw, plan.pitch, 0,0,1};
        if (plan.control) aim_xyza = planner.debug_xyza;
      }
      cboard.send(command);

      auto_aim::DetectionOverlayData overlay_data;
      overlay_data.valid = true;
      overlay_data.frame_count = total_frame_count;
      overlay_data.control = command.control;
      overlay_data.fire = command.shoot;
      overlay_data.yaw_deg = command.yaw * 57.3;
      overlay_data.pitch_deg = command.pitch * 57.3;
      overlay_data.fps = current_fps;
      overlay_data.detect_ms = last_detect_ms;
      overlay_data.armor_count = static_cast<int>(armors.size());
      overlay_data.target_count = static_cast<int>(targets.size());
      auto_aim::set_detection_overlay_data(overlay_data);

      cv::Mat debug_img = img.clone();

      if (!armors.empty()) {
        const auto & armor = armors.front();
        if (!armor.points.empty()) {
          tools::draw_points(debug_img, armor.points, {255, 0, 0}, 2);
        }

        auto name_id = static_cast<std::size_t>(armor.name);
        auto color_id = static_cast<std::size_t>(armor.color);
        auto type_id = static_cast<std::size_t>(armor.type);
        std::string armor_info = fmt::format(
          "{:.2f} {} {} {}", armor.confidence,
          color_id < auto_aim::COLORS.size() ? auto_aim::COLORS[color_id] : "unknown",
          name_id < auto_aim::ARMOR_NAMES.size() ? auto_aim::ARMOR_NAMES[name_id] : "unknown",
          type_id < auto_aim::ARMOR_TYPES.size() ? auto_aim::ARMOR_TYPES[type_id] : "unknown");
        tools::draw_text(debug_img, armor_info, armor.center, {255, 0, 0}, 0.5, 1);
      }

      if (!targets.empty()) {
        auto target = targets.front();
        int ekf_box_count = 0;
        for (const auto & xyza : target.armor_xyza_list()) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(debug_img, image_points, {0, 255, 0}, 2);
          if (++ekf_box_count >= 4) break;
        }

        if (aim_xyza.has_value()) {
          auto aim_points =
            solver.reproject_armor(aim_xyza->head(3), (*aim_xyza)[3], target.armor_type, target.name);
          tools::draw_points(debug_img, aim_points, {0, 0, 255}, 2);
        }
      } else {
        tools::draw_text(debug_img, "No Target", {10, 150}, {128, 128, 128}, 0.6, 2);
      }

      fps_frame_count++;
      total_frame_count++;
      auto now = std::chrono::steady_clock::now();
      auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
      if (dt_ms >= 1000) {
        current_fps = fps_frame_count * 1000.0 / dt_ms;
        tools::logger()->info(
          "运行中 | frame:{} | fps:{:.1f} | armors:{} | targets:{} | detect:{:.1f}ms | fire:{}",
          total_frame_count, current_fps, armors.size(), targets.size(), last_detect_ms, command.shoot);
        fps_frame_count = 0;
        last_fps_time = now;
      }

      tools::draw_text(
        debug_img,
        fmt::format(
          "FPS:{:.1f} Detect:{:.1f}ms Armors:{} Targets:{} Mode:QLF-MPC Fire:{}",
          current_fps, last_detect_ms, armors.size(), targets.size(), command.shoot),
        {10, 30}, {255, 255, 255}, 0.7, 2);

      Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      tools::draw_text(
        debug_img,
        fmt::format(
          "Gimbal: Yaw {:.1f}deg Pitch {:.1f}deg",
          gimbal_pos[0] * 57.3, -gimbal_pos[1] * 57.3),
        {10, 60}, {255, 255, 255}, 0.6, 1);

      nlohmann::json plot_data;
      plot_data["cmd_yaw"] = command.yaw * 57.3;
      plot_data["cmd_pitch"] = command.pitch * 57.3;
      plot_data["actual_yaw"] = gimbal_pos[0] * 57.3;
      plot_data["actual_pitch"] = -gimbal_pos[1] * 57.3;
      plot_data["yaw_error"] = (command.yaw - gimbal_pos[0]) * 57.3;
      plot_data["pitch_error"] = (command.pitch + gimbal_pos[1]) * 57.3;
      plotter.plot(plot_data);

      auto segment_now = std::chrono::steady_clock::now();
      if (debug_writer.isOpened() && segment_now - video_segment_start >= video_segment_duration) {
        debug_writer.release();
        ++video_segment_index;
        output_path = make_segment_path();
        video_segment_start = segment_now;
      }

      if (!debug_writer.isOpened()) {
        auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        debug_writer.open(output_path, fourcc, fps, debug_img.size());
        if (!debug_writer.isOpened()) {
          tools::logger()->error("无法打开QLF调试视频保存路径: {}", output_path);
          break;
        }
        tools::logger()->info("QLF绘制后调试视频保存到: {}", output_path);
      }
      debug_writer.write(debug_img);

      if (display) {
        cv::Mat raw_display_img;
        cv::Mat debug_display_img;
        cv::resize(img, raw_display_img, {}, 0.5, 0.5);
        cv::resize(debug_img, debug_display_img, {}, 0.5, 0.5);
        cv::imshow("QYG Hero QLF Raw", raw_display_img);
        cv::imshow("QYG Hero QLF Debug", debug_display_img);
      }

      auto key = cv::waitKey(1);
      if (key == 'q' || key == 27) break;
    }

    else if (mode == io::Mode::idle) {
      if (++idle_counter >= 10) {
        cboard.send({false, false, 0, 0, 0});
        idle_counter = 0;
      }
      std::this_thread::sleep_for(50ms);
    }

    else {
      std::this_thread::sleep_for(50ms);
    }
  }

  cboard.send({false, false, 0, 0, 0});
  debug_writer.release();
  cv::destroyAllWindows();
  tools::logger()->info("QYG_hero_debug_QLF 已停止，共处理 {} 帧。", total_frame_count);

  return 0;
}
