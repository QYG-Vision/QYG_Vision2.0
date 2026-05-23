#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
// #include "tasks/auto_buff/buff_aimer.hpp"
// #include "tasks/auto_buff/buff_detector.hpp"
// #include "tasks/auto_buff/buff_solver.hpp"
// #include "tasks/auto_buff/buff_target.hpp"
// #include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/recorder.hpp"
#include "io/ros2/aim2nav.hpp"
#include "io/ros2/nav2aim.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_sentry.yaml | 位置参数,yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help")) {
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

  auto_aim::multithread::MultiThreadDetector detector(config_path,true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  // auto_buff::Buff_Detector buff_detector(config_path);
  // auto_buff::Solver buff_solver(config_path);
  // auto_buff::SmallTarget buff_small_target;
  // auto_buff::BigTarget buff_big_target;
  // auto_buff::Aimer buff_aimer(config_path);

  std::atomic<bool> quit = false;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};

  // 检测线程：异步进行图像采集和检测
  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!quit && !exiter.exit()) {
      if (mode.load() == io::GimbalMode::AUTO_AIM) {
        camera.read(img, t);
        detector.push(img, t);  // 异步检测
      } else
        std::this_thread::sleep_for(10ms);
    }
  });

  // 规划线程：MPC规划和控制
  auto plan_thread = std::thread([&]() {
    while (!quit && rclcpp::ok()) {
      auto cmd_vel = nav2aim.get_latest_state();
      if (mode.load() == io::GimbalMode::AUTO_AIM && !target_queue.empty()) {
        auto target = target_queue.pop();  // pop()会阻塞等待，但empty()检查可以避免在非自瞄模式下等待
        auto gs = gimbal.state();
        auto plan = planner.plan(target, gs.bullet_speed);

        gimbal.send(
          plan.control, plan.fire, plan.yaw, plan.pitch,
          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

        std::this_thread::sleep_for(10ms);
      } else {
        gimbal.send(
          false, false, 0.0f, 0.0f,
          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        std::this_thread::sleep_for(50ms);
      }
    }
  });
  while (!exiter.exit()) {
    mode = gimbal.mode();
    auto current_mode = mode.load();  // 缓存模式值，避免重复调用load()

    if (last_mode != current_mode) {
      tools::logger()->info("Switch to {}", gimbal.str(current_mode));
      last_mode = current_mode;
    }

    /// 自瞄
    if (current_mode == io::GimbalMode::AUTO_AIM) {
      // 从检测队列获取结果（异步检测已完成）
      auto [img, armors, t] = detector.debug_pop();
      auto q = gimbal.euler(t - 1ms);
      
      recorder.record(img, q, t);
      solver.set_R_gimbal2world(q);

      auto targets = tracker.track(armors, t);
      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    }

    // /// 打符
    // else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
    //   buff_solver.set_R_gimbal2world(q);

    //   auto power_runes = buff_detector.detect(img);

    //   buff_solver.solve(power_runes);

    //   auto_aim::Plan buff_plan;
    //   if (mode.load() == io::GimbalMode::SMALL_BUFF) {
    //     buff_small_target.get_target(power_runes, t);
    //     auto target_copy = buff_small_target;
    //     buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
    //   } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
    //     buff_big_target.get_target(power_runes, t);
    //     auto target_copy = buff_big_target;
    //     buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
    //   }
    //   gimbal.send(_
    //     buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
    //     buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);
    // }
    
    else if (current_mode == io::GimbalMode::IDLE) {
      std::this_thread::sleep_for(50ms);  // 降低CPU占用
    }
  }

  quit = true;
  if (detect_thread.joinable()) detect_thread.join();
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0);

  rclcpp::shutdown();
  return 0;
}
