#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <list>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace
{
const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明}"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标}"
  "{end-index e    | 0                 | 视频结束帧下标，0 表示到文件结束}"
  "{output-dir o   | logs/ekf_visual_experience | Rerun/离线数据输出目录}"
  "{image-stride   | 1                 | 每 N 帧保存一张叠加调试图，0 表示不保存图片}"
  "{delay-ms       | 30                | 每帧等待时间，方便肉眼观察}"
  "{synthetic      |                   | 使用模拟 EKF 轨迹，不读取视频/模型，适合缺少 demo.avi 时体验}"
  "{no-gui         |                   | 不打开 OpenCV 窗口，适合无显示环境}"
  "{@input-path    | assets/demo/demo  | avi 和 txt 文件的公共路径}";

struct FrameDebug
{
  int frame = 0;
  double t = 0.0;
  bool has_armor = false;
  bool has_target = false;
  bool has_aim = false;
  double gimbal_yaw = 0.0;
  double cmd_yaw = 0.0;
  double armor_yaw = 0.0;
  Eigen::Vector3d armor_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d aim_xyz = Eigen::Vector3d::Zero();
  nlohmann::json scalars;
};

void write_json_line(
  std::ofstream & jsonl, const FrameDebug & debug, const std::string & image_path,
  const std::vector<Eigen::Vector3d> & predicted_armors)
{
  nlohmann::json j;
  j["frame"] = debug.frame;
  j["t"] = debug.t;
  j["has_armor"] = debug.has_armor;
  j["has_target"] = debug.has_target;
  j["has_aim"] = debug.has_aim;
  j["gimbal_yaw"] = debug.gimbal_yaw;
  j["cmd_yaw"] = debug.cmd_yaw;
  j["armor_yaw"] = debug.armor_yaw;
  j["armor_xyz"] = {debug.armor_xyz.x(), debug.armor_xyz.y(), debug.armor_xyz.z()};
  j["target_xyz"] = {debug.target_xyz.x(), debug.target_xyz.y(), debug.target_xyz.z()};
  j["aim_xyz"] = {debug.aim_xyz.x(), debug.aim_xyz.y(), debug.aim_xyz.z()};
  j["predicted_armors"] = nlohmann::json::array();
  for (const auto & p : predicted_armors) {
    j["predicted_armors"].push_back({p.x(), p.y(), p.z()});
  }
  j["image_path"] = image_path;
  j["scalars"] = debug.scalars;
  jsonl << j.dump() << "\n";
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto input_path = cli.get<std::string>(0);
  const auto config_path = cli.get<std::string>("config-path");
  const auto start_index = cli.get<int>("start-index");
  const auto end_index = cli.get<int>("end-index");
  const auto output_dir = std::filesystem::path(cli.get<std::string>("output-dir"));
  const auto image_stride = cli.get<int>("image-stride");
  const auto delay_ms = cli.get<int>("delay-ms");
  const bool synthetic = cli.has("synthetic");
  const bool no_gui = cli.has("no-gui");

  const auto image_dir = output_dir / "images";
  std::filesystem::create_directories(image_dir);
  std::ofstream jsonl(output_dir / "frames.jsonl");
  if (!jsonl.is_open()) {
    tools::logger()->error("Failed to open {}", (output_dir / "frames.jsonl").string());
    return 1;
  }

  tools::Exiter exiter;

  if (synthetic) {
    int saved_images = 0;
    const int synthetic_end = end_index > 0 ? end_index : 240;
    for (int frame_count = start_index; frame_count <= synthetic_end && !exiter.exit(); ++frame_count) {
      const double t = frame_count / 30.0;

      FrameDebug debug;
      debug.frame = frame_count;
      debug.t = t;
      debug.has_armor = true;
      debug.has_target = true;
      debug.has_aim = true;
      debug.gimbal_yaw = 8.0 * std::sin(t);
      debug.cmd_yaw = debug.gimbal_yaw + 1.5 * std::sin(t * 2.0);
      debug.armor_yaw = debug.gimbal_yaw - 2.0 * std::cos(t * 1.7);
      debug.armor_xyz = {2.5 + 0.2 * std::sin(t), 0.4 * std::cos(t), 0.35};
      debug.target_xyz = {2.6 + 0.2 * std::sin(t + 0.12), 0.4 * std::cos(t + 0.12), 0.38};
      debug.aim_xyz = debug.target_xyz + Eigen::Vector3d(0.0, 0.02 * std::sin(t * 3.0), 0.05);
      debug.scalars["armor_num"] = 1;
      debug.scalars["cmd_pitch"] = -6.0 + 0.5 * std::sin(t);
      debug.scalars["shoot"] = std::abs(debug.cmd_yaw - debug.gimbal_yaw) < 1.0;
      debug.scalars["residual_yaw"] = debug.armor_yaw - debug.gimbal_yaw;
      debug.scalars["residual_pitch"] = 0.4 * std::sin(t * 2.1);
      debug.scalars["residual_distance"] = 0.08 * std::cos(t * 1.3);
      debug.scalars["nis"] = 1.0 + 0.4 * std::sin(t * 1.9);
      debug.scalars["nees"] = 1.2 + 0.5 * std::cos(t * 1.4);
      debug.scalars["w"] = 0.8 * std::sin(t * 0.7);
      debug.scalars["r"] = 0.22;

      std::vector<Eigen::Vector3d> predicted_armors = {
        debug.target_xyz + Eigen::Vector3d(0.0, 0.18, 0.0),
        debug.target_xyz + Eigen::Vector3d(0.0, -0.18, 0.0),
      };

      cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(24, 24, 24));
      const auto obs_px =
        cv::Point(640 + static_cast<int>(debug.armor_xyz.y() * 450), 360 - static_cast<int>(debug.armor_xyz.z() * 450));
      const auto tgt_px =
        cv::Point(640 + static_cast<int>(debug.target_xyz.y() * 450), 360 - static_cast<int>(debug.target_xyz.z() * 450));
      const auto aim_px =
        cv::Point(640 + static_cast<int>(debug.aim_xyz.y() * 450), 360 - static_cast<int>(debug.aim_xyz.z() * 450));
      cv::circle(img, obs_px, 7, cv::Scalar(0, 128, 255), -1);
      cv::circle(img, tgt_px, 7, cv::Scalar(0, 255, 0), -1);
      cv::circle(img, aim_px, 7, cv::Scalar(0, 0, 255), -1);
      tools::draw_text(img, fmt::format("synthetic t={:.2f}s", t), {10, 40}, {255, 255, 255});

      std::string image_path;
      if (image_stride > 0 && frame_count % image_stride == 0) {
        image_path = (image_dir / fmt::format("frame_{:06d}.jpg", frame_count)).string();
        cv::imwrite(image_path, img);
        ++saved_images;
      }
      write_json_line(jsonl, debug, image_path, predicted_armors);

      if (!no_gui) {
        cv::Mat disp;
        cv::resize(img, disp, {}, 0.7, 0.7);
        cv::imshow("ekf_visual_experience_rerun", disp);
        if (cv::waitKey(delay_ms) == 'q') break;
      } else if (delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
      }
    }
    tools::logger()->info("[ekf_visual_experience_rerun] wrote {}", (output_dir / "frames.jsonl").string());
    return 0;
  }

  const auto avi_path = input_path + ".avi";
  const auto txt_path = input_path + ".txt";

  cv::VideoCapture video(avi_path);
  if (!video.isOpened()) {
    tools::logger()->error("Failed to open video: {}", avi_path);
    return 1;
  }

  std::ifstream text(txt_path);
  if (!text.is_open()) {
    tools::logger()->error("Failed to open txt: {}", txt_path);
    return 1;
  }

  // Pipeline (same as auto_aim_test offline flow)
  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  io::Command last_command;

  int saved_images = 0;
  const auto t0 = std::chrono::steady_clock::now();

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; ++i) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); ++frame_count) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    if (!text.good()) break;

    const auto timestamp = t0 + std::chrono::microseconds(static_cast<int>(t * 1e6));

    solver.set_R_gimbal2world(Eigen::Quaterniond(w, x, y, z));
    auto armors = yolo.detect(img, frame_count);
    auto targets = tracker.track(armors, timestamp);
    auto command = aimer.aim(targets, timestamp, 27, false);

    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;
    if (command.control) last_command = command;

    FrameDebug debug;
    debug.frame = frame_count;
    debug.t = t;
    debug.cmd_yaw = command.yaw * 57.3;
    debug.scalars["armor_num"] = armors.size();
    debug.scalars["cmd_pitch"] = command.pitch * 57.3;
    debug.scalars["shoot"] = command.shoot;

    Eigen::Quaterniond q{w, x, y, z};
    debug.gimbal_yaw = tools::eulers(q, 2, 1, 0)[0] * 57.3;

    if (!armors.empty()) {
      const auto & armor = armors.front();
      debug.has_armor = true;
      debug.armor_xyz = armor.xyz_in_world;
      debug.armor_yaw = armor.ypr_in_world[0] * 57.3;
      debug.scalars["armor_x"] = armor.xyz_in_world[0];
      debug.scalars["armor_y"] = armor.xyz_in_world[1];
      debug.scalars["armor_z"] = armor.xyz_in_world[2];
      debug.scalars["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      debug.scalars["armor_center_x"] = armor.center_norm.x;
      debug.scalars["armor_center_y"] = armor.center_norm.y;
    }

    std::vector<Eigen::Vector3d> predicted_armors;
    if (!targets.empty()) {
      const auto target = targets.front();
      debug.has_target = true;
      const auto ekf_x = target.ekf_x();
      debug.target_xyz = Eigen::Vector3d(ekf_x[0], ekf_x[2], ekf_x[4]);
      debug.scalars["x"] = ekf_x[0];
      debug.scalars["vx"] = ekf_x[1];
      debug.scalars["y"] = ekf_x[2];
      debug.scalars["vy"] = ekf_x[3];
      debug.scalars["z"] = ekf_x[4];
      debug.scalars["vz"] = ekf_x[5];
      debug.scalars["a"] = ekf_x[6] * 57.3;
      debug.scalars["w"] = ekf_x[7];
      debug.scalars["r"] = ekf_x[8];
      debug.scalars["l"] = ekf_x[9];
      debug.scalars["h"] = ekf_x[10];
      debug.scalars["last_id"] = target.last_id;

      const auto armor_xyza_list = target.armor_xyza_list();
      for (const auto & xyza : armor_xyza_list) {
        predicted_armors.push_back(xyza.head(3));
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      auto aim_point = aimer.debug_aim_point;
      if (aim_point.valid) {
        debug.has_aim = true;
        debug.aim_xyz = aim_point.xyza.head(3);
        auto image_points = solver.reproject_armor(
          aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 0, 255});
      }

      const auto & ekf_data = target.ekf().data;
      debug.scalars["residual_yaw"] = ekf_data.at("residual_yaw");
      debug.scalars["residual_pitch"] = ekf_data.at("residual_pitch");
      debug.scalars["residual_distance"] = ekf_data.at("residual_distance");
      debug.scalars["residual_angle"] = ekf_data.at("residual_angle");
      debug.scalars["nis"] = ekf_data.at("nis");
      debug.scalars["nees"] = ekf_data.at("nees");
      debug.scalars["nis_fail"] = ekf_data.at("nis_fail");
      debug.scalars["nees_fail"] = ekf_data.at("nees_fail");
      debug.scalars["recent_nis_failures"] = ekf_data.at("recent_nis_failures");
    }

    tools::draw_text(
      img,
      fmt::format(
        "cmd yaw={:.2f} pitch={:.2f} shoot={}", debug.cmd_yaw,
        debug.scalars.value("cmd_pitch", 0.0), command.shoot),
      {10, 60}, {154, 50, 205});
    tools::draw_text(img, fmt::format("gimbal yaw={:.2f}", debug.gimbal_yaw), {10, 90}, {255, 255, 255});

    std::string image_path;
    if (image_stride > 0 && frame_count % image_stride == 0) {
      image_path = (image_dir / fmt::format("frame_{:06d}.jpg", frame_count)).string();
      cv::imwrite(image_path, img);
      ++saved_images;
    }
    write_json_line(jsonl, debug, image_path, predicted_armors);

    if (!no_gui) {
      cv::Mat disp;
      cv::resize(img, disp, {}, 0.5, 0.5);
      cv::imshow("ekf_visual_experience_rerun", disp);
      if (cv::waitKey(delay_ms) == 'q') break;
    } else if (delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
  }

  tools::logger()->info(
    "[ekf_visual_experience_rerun] wrote {} images and {}", saved_images,
    (output_dir / "frames.jsonl").string());
  return 0;
}
