#include <chrono>
#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

#include "io/camera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_sentry.yaml | 配置文件路径（含相机+检测参数）}";

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    rclcpp::shutdown();
    return 0;
  }

  auto config_path = cli.get<std::string>("@config-path");
  auto node = std::make_shared<rclcpp::Node>("camera_uv_detect");
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/camera/uv_detect", 10);

  tools::Exiter exiter;
  tools::logger()->info("[CameraUV] Opening camera and detector...");

  io::Camera camera(config_path);
  auto_aim::multithread::MultiThreadDetector detector(config_path, true);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;

  auto detect_thread = std::thread([&]() {
    while (rclcpp::ok() && !exiter.exit()) {
      camera.read(img, timestamp);
      if (!img.empty()) detector.push(img, timestamp);
    }
  });

  while (rclcpp::ok() && !exiter.exit()) {
    auto [raw_img, armors, t] = detector.debug_pop();
    if (raw_img.empty()) continue;

    cv::Mat display = raw_img.clone();

    for (const auto & armor : armors) {
      // 四个角点 + UV 坐标
      for (size_t i = 0; i < armor.points.size(); ++i) {
        const auto & p = armor.points[i];
        cv::circle(display, p, 4, {0, 255, 255}, -1);
        std::string label = fmt::format("U={:.0f} V={:.0f}", p.x, p.y);
        tools::draw_text(display, label,
          {static_cast<int>(p.x) + 10, static_cast<int>(p.y) - 10},
          {0, 255, 255}, 0.5, 1);
      }
      // 连线
      if (armor.points.size() == 4) {
        for (int i = 0; i < 4; ++i)
          cv::line(display, armor.points[i], armor.points[(i+1)%4], {255,255,0}, 2);
      }
      // 物理 3D 模型尺寸
      auto phys = fmt::format("{:.0f}x{:.0f}mm",
        (armor.type == auto_aim::ArmorType::big ? 230.0 : 145.0), 53.0);
      tools::draw_text(display, phys,
        {armor.box.x, armor.box.y + armor.box.height + 10}, {200,200,200}, 0.5, 1);
      // 装甲板信息
      auto info = fmt::format("{} {} {} conf={:.2f}",
        auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type], armor.confidence);
      tools::draw_text(display, info, armor.center, {0, 255, 0}, 0.8, 2);
    }

    if (armors.empty())
      tools::draw_text(display, "No armor detected", {10, 30}, {128,128,128}, 0.8, 2);

    // 帧率
    frame_count++;
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_fps_time).count();
    if (dt >= 1000) {
      tools::logger()->info("[CameraUV] {:.1f} fps, {} armors",
        frame_count * 1000.0 / dt, armors.size());
      frame_count = 0; last_fps_time = std::chrono::steady_clock::now();
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = "camera_optical_frame";
    msg.height = display.rows;
    msg.width = display.cols;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(display.step);
    msg.data.assign(display.datastart, display.dataend);
    pub->publish(msg);
  }

  if (detect_thread.joinable()) detect_thread.join();
  rclcpp::shutdown();
  return 0;
}
