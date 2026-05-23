#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"
#include "tools/exiter.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>



const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{f              | | 是否开火}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto test_fire = cli.get<bool>("f");
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }
  io::Gimbal gimbal(config_path);
  tools::Exiter exiter;

  while(!exiter.exit())
  {

    auto t = std::chrono::steady_clock::now();
    auto state = gimbal.state();
    auto euler = gimbal.euler(t);
    auto mode = gimbal.mode();
    tools::logger()->info(
      "Gimbal rx: mode={} yaw={:.3f} pitch={:.3f} bullet_speed={:.3f} euler=[{:.4f},{:.4f},{:.4f}]",
      gimbal.str(mode), state.yaw, state.pitch, state.bullet_speed, euler(0), euler(1), euler(2));
  }

  return 0;
}