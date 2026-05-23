#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
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

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_hero.yaml | 位置参数,yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
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

  std::atomic<io::Mode> mode{io::Mode::idle};
  auto last_mode{io::Mode::idle};
  int idle_counter = 0;  // idle模式下的计数器，用于降低发送频率

  // 检测线程：异步进行图像采集和检测
  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!quit && !exiter.exit()) {
      if (mode.load() == io::Mode::auto_aim) {
        camera.read(img, t);
        detector.push(img, t);  // 异步检测
      } else
        std::this_thread::sleep_for(10ms);
    }
  });

  // 规划线程：MPC规划和控制
  auto plan_thread = std::thread([&]() {
    while (!quit) {
      if (mode.load() == io::Mode::auto_aim && !target_queue.empty()) {
        auto target = target_queue.pop();  // pop()会阻塞等待，但empty()检查可以避免在非自瞄模式下等待
        auto plan = planner.plan(target, cboard.bullet_speed);

        // 使用初始化列表构造Command
        io::Command command{
          plan.control, plan.fire, plan.yaw, plan.pitch, 0  // horizon_distance设为0
        };
        cboard.send(command);

        std::this_thread::sleep_for(10ms);
      } else
        std::this_thread::sleep_for(50ms);  // 减少等待时间，提高响应速度
    }
  });

  while (!exiter.exit()) {
    mode = cboard.mode;
    auto current_mode = mode.load();  // 缓存模式值，避免重复调用load()

    if (last_mode != current_mode) {
      tools::logger()->info("Switch to {}", io::MODES[current_mode]);
      last_mode = current_mode;
    }

    /// 自瞄
    if (current_mode == io::Mode::auto_aim) {
      // 从检测队列获取结果（异步检测已完成）
      auto [img, armors, t] = detector.debug_pop();
      auto q = cboard.imu_at(t - 1ms);
      
      recorder.record(img, q, t);
      solver.set_R_gimbal2world(q);

      auto targets = tracker.track(armors, t);
      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    }

    // /// 打符
    // else if (mode.load() == io::Mode::small_buff || mode.load() == io::Mode::big_buff) {
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
    //   cboard.send(io::command(plan.control, plan.fire,plan.yaw,plan.pitch,plan.horizon_distance));
    // }
    
    else if (current_mode == io::Mode::idle) {
      // idle模式下降低发送频率（每10次循环发送一次，约500ms）
      if (++idle_counter >= 10) {
        io::Command command{false, false, 0, 0, 0};
        cboard.send(command);
        idle_counter = 0;
      }
      std::this_thread::sleep_for(50ms);  // 降低CPU占用
    }

  }

  quit = true;
  if (detect_thread.joinable()) detect_thread.join();
  if (plan_thread.joinable()) plan_thread.join();
  io::Command command{false, false, 0, 0, 0};
  cboard.send(command);

  return 0;
}