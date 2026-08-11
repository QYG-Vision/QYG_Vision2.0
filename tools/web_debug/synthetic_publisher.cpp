#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>

#include "web_debug.hpp"

int main(int argc, char ** argv)
{
  double seconds = 0.0;
  if (argc == 3 && std::string(argv[1]) == "--seconds") {
    seconds = std::stod(argv[2]);
  } else if (argc != 1) {
    std::cerr << "Usage: qyg_web_debug_synthetic [--seconds N]\n";
    return 2;
  }

  using Clock = std::chrono::steady_clock;
  const auto begin = Clock::now();
  tools::web_debug::WebDebugPublisher publisher;
  tools::web_debug::WebDebugRenderer renderer;
  std::uint64_t frame = 0;
  while (seconds <= 0.0 || std::chrono::duration<double>(Clock::now() - begin).count() < seconds) {
    const double t = std::chrono::duration<double>(Clock::now() - begin).count();
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(5, 13, 22));
    const cv::Point center(320 + static_cast<int>(120 * std::sin(t)), 240);
    const cv::Rect box(center.x - 45, center.y - 25, 90, 50);
    cv::rectangle(image, box, cv::Scalar(0, 150, 220), 2);

    tools::web_debug::WebDebugContext context;
    context.system.source = "qyg_web_debug_synthetic";
    context.system.mode = "AUTO_AIM";
    context.system.fps = 60.0;
    context.system.frame_id = frame++;
    context.detector.armor_count = 1;
    context.tracker.state = "tracking";
    context.tracker.has_target = true;
    context.target = tools::web_debug::TargetSnapshot{
      "synthetic", "small", 3.0, 0.2, 0.8, 0.1, 0.0, 0.0, t, 1.2, 0.28, 1};
    context.planner.valid = true;
    context.planner.observation_frame_id = context.system.frame_id;
    context.planner.age_ms = 2.0;
    context.planner.control = true;
    context.planner.target_yaw_rad = 0.1 * std::sin(t);
    context.planner.command_yaw_rad = 0.09 * std::sin(t);
    context.gimbal.yaw_rad = 0.08 * std::sin(t);
    context.gimbal.bullet_speed_mps = 22.0;
    context.timing.frame_age_ms = 8.0;

    tools::web_debug::OverlaySnapshot overlays;
    overlays.detections.push_back(
      {{{static_cast<float>(box.x), static_cast<float>(box.y)},
        {static_cast<float>(box.x + box.width), static_cast<float>(box.y)},
        {static_cast<float>(box.x + box.width), static_cast<float>(box.y + box.height)},
        {static_cast<float>(box.x), static_cast<float>(box.y + box.height)}},
       "synthetic", 0.99});
    const auto debug_image = renderer.render(image, context, overlays);
    if (!publisher.publish(debug_image, context, t)) {
      std::cerr << "synthetic publisher: publish failed\n";
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  return 0;
}
