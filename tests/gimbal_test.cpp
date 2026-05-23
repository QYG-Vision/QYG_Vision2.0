#include "io/gimbal/gimbal.hpp"
#include "io/camera.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{f              | | 是否开火}"
  "{@config-path   | configs/QYG_sentry.yaml | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto test_fire = cli.get<bool>("f");
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  
  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  cv::Mat frame;

  auto t0 = std::chrono::steady_clock::now();
  auto last_mode = gimbal.mode();
  // uint16_t last_bullet_count = 0;  // field removed

  auto last_rx_log = std::chrono::steady_clock::now();

  float yaw_deg = 5.0f;
  float pitch_deg = -15.0f;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();
    auto mode = gimbal.mode();

    if (mode != last_mode) {
      tools::logger()->info("Gimbal mode changed: {}", gimbal.str(mode));
      last_mode = mode;
    }

    auto t = std::chrono::steady_clock::now();
    camera.read(frame, t);
    auto euler = gimbal.euler(t);
    auto ypr = Eigen::Vector3d(euler(2), euler(1), euler(0));

    if (t - last_rx_log >= std::chrono::milliseconds(200)) {
      tools::logger()->info(
        "Gimbal rx: mode={} yaw={:.1f}deg pitch={:.1f}deg roll={:.1f}deg",
        gimbal.str(mode), ypr(0), ypr(1), ypr(2));
      last_rx_log = t;
    }

    // gimbal.send(true, test_fire && fire, 2, 4);
    // float linear_x = 0.1f;
    // float linear_y = 0.5f;
    // float angular_z = 0.5f;
    gimbal.send(true, true, yaw_deg * CV_PI / 180.0f, pitch_deg * CV_PI / 180.0f,0.0f,0.0f);
  
    tools::logger()->info(
      "Gimbal tx: control=1 yaw={:.1f}deg pitch={:.1f}deg",
      yaw_deg, pitch_deg);

    nlohmann::json data;
    data["q_yaw"] = ypr[0];
    data["q_pitch"] = ypr[1];
    data["yaw"] = ypr(0);
    data["pitch"] = ypr(1);
    data["t"] = tools::delta_time(t, t0);
    plotter.plot(data);

    //在摄像头上显示 TX / RX 信息
    cv::Mat disp = frame.clone();
    tools::draw_text(disp,
      fmt::format("TX: yaw={:.1f}deg pitch={:.1f}deg", yaw_deg, pitch_deg),
      {10, 30}, {0, 255, 0}, 0.7, 2);
    tools::draw_text(disp,
      fmt::format("RX: yaw={:.1f}deg pitch={:.1f}deg roll={:.1f}deg", ypr(0), ypr(1), ypr(2)),
      {10, 60}, {0, 255, 255}, 0.7, 2);
    cv::resize(disp, disp, {}, 0.5, 0.5);
    cv::imshow("Gimbal Test", disp);
    if (cv::waitKey(1) == 'q') break;

    std::this_thread::sleep_for(50ms);

  }

  gimbal.send(false, false, 0.0f, 0.0f, 0.0f, 0.0f);
  tools::logger()->info("Gimbal tx: control=0 yaw={:.1f}deg pitch={:.1f}deg", yaw_deg, pitch_deg);
  return 0;
}