#include <cassert>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <unistd.h>

#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/web_debug_adapter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/web_debug/web_debug.hpp"

int main()
{
  {
    tools::web_debug::WebDebugContext context;
    context.system.mode = "AUTO_AIM";
    context.system.fps = 60.0;
    context.gimbal.yaw_rad = M_PI / 2.0;

    const auto json = context.to_status_json();

    assert(json.at("schema_version") == 1);
    assert(json.at("system").at("mode") == "AUTO_AIM");
    assert(std::abs(json.at("gimbal").at("yaw_deg").get<double>() - 90.0) < 1e-9);
    assert(json.at("target").is_null());
    assert(json.at("planner").at("valid") == false);
    assert(json.at("planner").at("command_yaw_deg").is_null());

    context.planner.valid = true;
    context.planner.observation_frame_id = 42;
    context.planner.age_ms = 12.5;
    context.planner.command_yaw_rad = M_PI / 4.0;
    const auto valid_planner_json = context.to_status_json().at("planner");
    assert(valid_planner_json.at("observation_frame_id") == 42);
    assert(valid_planner_json.at("age_ms") == 12.5);
    assert(std::abs(valid_planner_json.at("command_yaw_deg").get<double>() - 45.0) < 1e-9);
  }

  {
    tools::web_debug::CurveHistory history(600);
    tools::web_debug::WebDebugContext context;
    for (int i = 0; i < 601; ++i) {
      context.system.fps = static_cast<double>(i);
      context.detector.armor_count = static_cast<std::size_t>(i % 4);
      if (i == 600) context.target.reset();
      history.append(context, i * 0.05);
    }

    const auto json = history.to_json();
    assert(json.at("schema_version") == 1);
    assert(json.at("time").size() == 600);
    assert(json.at("fps").size() == json.at("time").size());
    assert(json.at("target_x").size() == json.at("time").size());
    assert(json.at("time").front() == 0.05);
    assert(json.at("fps").back() == 600.0);
    assert(json.at("target_x").back().is_null());
    assert(json.at("command_yaw").back().is_null());
  }

  {
    auto header = tools::web_debug::make_frame_header(8, 1024, 123456);
    assert(std::memcmp(header.magic, "QYGF", 4) == 0);
    assert(header.header_size == sizeof(tools::web_debug::FrameHeader));
    assert(tools::web_debug::valid_frame_header(header, 4096));

    header.sequence = 9;
    assert(!tools::web_debug::valid_frame_header(header, 4096));
    header.sequence = 10;
    header.jpeg_size = 5000;
    assert(!tools::web_debug::valid_frame_header(header, 4096));
    header.jpeg_size = 10;
    header.capacity = 5000;
    assert(!tools::web_debug::valid_frame_header(header, 4096));
  }

  {
    const auto root = std::filesystem::path("/tmp") /
      ("qyg_web_debug_test_" + std::to_string(static_cast<long long>(getpid())));
    std::filesystem::create_directories(root);
    tools::web_debug::PublisherOptions options;
    options.frame_path = (root / "frame").string();
    options.data_path = (root / "data.json").string();
    options.log_path = (root / "log.json").string();
    options.frame_size = 64 * 1024;
    options.frame_interval = std::chrono::hours(1);
    options.json_interval = std::chrono::hours(1);
    tools::web_debug::WebDebugPublisher publisher(options);

    cv::Mat image(32, 32, CV_8UC3, cv::Scalar(10, 20, 30));
    tools::web_debug::WebDebugContext context;
    context.system.producer_online = true;
    assert(publisher.publish(image, context, 1.0));

    std::ifstream frame_file(options.frame_path, std::ios::binary);
    tools::web_debug::FrameHeader frame_header{};
    frame_file.read(reinterpret_cast<char *>(&frame_header), sizeof(frame_header));
    assert(tools::web_debug::valid_frame_header(frame_header, options.frame_size));
    std::vector<unsigned char> jpeg(frame_header.jpeg_size);
    frame_file.read(reinterpret_cast<char *>(jpeg.data()), jpeg.size());
    assert(jpeg.size() > 2 && jpeg[0] == 0xff && jpeg[1] == 0xd8);

    std::ifstream log_file(options.log_path);
    const auto log = nlohmann::json::parse(
      std::string(std::istreambuf_iterator<char>(log_file), std::istreambuf_iterator<char>()));
    assert(log.at("schema_version") == 1);

    std::ifstream data_file(options.data_path);
    const auto data = nlohmann::json::parse(
      std::string(std::istreambuf_iterator<char>(data_file), std::istreambuf_iterator<char>()));
    assert(data.at("time").size() == 1);
    assert(publisher.publish(image, context, 2.0));
    std::ifstream unchanged_frame_file(options.frame_path, std::ios::binary);
    tools::web_debug::FrameHeader unchanged_header{};
    unchanged_frame_file.read(reinterpret_cast<char *>(&unchanged_header), sizeof(unchanged_header));
    assert(unchanged_header.sequence == frame_header.sequence);
    assert(!publisher.publish(cv::Mat(), context, 2.0));
    std::filesystem::remove_all(root);
  }
  {
    const auto root = std::filesystem::path("/tmp") /
      ("qyg_web_debug_status_test_" + std::to_string(static_cast<long long>(getpid())));
    std::filesystem::create_directories(root);
    tools::web_debug::PublisherOptions options;
    options.frame_path = (root / "frame").string();
    options.data_path = (root / "data.json").string();
    options.log_path = (root / "log.json").string();
    options.frame_size = 64 * 1024;
    options.json_interval = std::chrono::milliseconds(0);
    tools::web_debug::WebDebugContext context;
    context.system.mode = "IDLE";
    {
      tools::web_debug::AsyncWebDebugPublisher publisher(options);
      assert(publisher.submit_status(context, 1.0));
    }
    assert(std::filesystem::exists(options.log_path));
    assert(std::filesystem::exists(options.data_path));
    std::ifstream log_file(options.log_path);
    const auto log = nlohmann::json::parse(
      std::string(std::istreambuf_iterator<char>(log_file), std::istreambuf_iterator<char>()));
    assert(log.at("system").at("mode") == "IDLE");
    std::filesystem::remove_all(root);
  }
  {
    const auto root = std::filesystem::path("/tmp") /
      ("qyg_web_debug_async_test_" + std::to_string(static_cast<long long>(getpid())));
    std::filesystem::create_directories(root);
    tools::web_debug::PublisherOptions options;
    options.frame_path = (root / "frame").string();
    options.data_path = (root / "data.json").string();
    options.log_path = (root / "log.json").string();
    options.frame_size = 64 * 1024;
    options.frame_interval = std::chrono::milliseconds(0);
    options.json_interval = std::chrono::milliseconds(0);
    std::atomic<bool> factory_called{false};
    {
      tools::web_debug::AsyncWebDebugPublisher publisher(options);
      cv::Mat image(32, 32, CV_8UC3, cv::Scalar(1, 2, 3));
      tools::web_debug::OverlaySnapshot overlays;
      assert(publisher.submit(image, {}, overlays, 0.0));
      for (std::uint64_t frame_id = 1; frame_id <= 50; ++frame_id) {
        tools::web_debug::WebDebugContext context;
        context.system.frame_id = frame_id;
        assert(publisher.submit(
          image, context,
          [&factory_called, overlays] {
            factory_called = true;
            return overlays;
          },
          frame_id * 0.05));
      }
    }
    assert(factory_called);
    std::ifstream log_file(root / "log.json");
    const auto log = nlohmann::json::parse(
      std::string(std::istreambuf_iterator<char>(log_file), std::istreambuf_iterator<char>()));
    assert(log.at("system").at("frame_id") == 50);
    std::filesystem::remove_all(root);
  }
  {
    const auto root = std::filesystem::path("/tmp") /
      ("qyg_web_debug_image_snapshot_test_" +
       std::to_string(static_cast<long long>(getpid())));
    std::filesystem::create_directories(root);
    tools::web_debug::PublisherOptions options;
    options.frame_path = (root / "frame").string();
    options.data_path = (root / "data.json").string();
    options.log_path = (root / "log.json").string();
    options.frame_size = 64 * 1024;
    options.frame_interval = std::chrono::milliseconds(0);
    options.json_interval = std::chrono::milliseconds(0);

    std::promise<void> factory_entered;
    std::promise<void> allow_factory_return;
    auto entered = factory_entered.get_future();
    auto release = allow_factory_return.get_future().share();
    {
      tools::web_debug::AsyncWebDebugPublisher publisher(options);
      cv::Mat image(64, 64, CV_8UC3, cv::Scalar(0, 0, 255));
      assert(publisher.submit(
        image, {},
        [&factory_entered, release] {
          factory_entered.set_value();
          release.wait();
          return tools::web_debug::OverlaySnapshot{};
        },
        1.0));
      entered.wait();
      image.setTo(cv::Scalar(0, 255, 0));
      allow_factory_return.set_value();
    }

    std::ifstream frame_file(options.frame_path, std::ios::binary);
    tools::web_debug::FrameHeader header{};
    frame_file.read(reinterpret_cast<char *>(&header), sizeof(header));
    assert(tools::web_debug::valid_frame_header(header, options.frame_size));
    std::vector<unsigned char> jpeg(header.jpeg_size);
    frame_file.read(reinterpret_cast<char *>(jpeg.data()), jpeg.size());
    const auto decoded = cv::imdecode(jpeg, cv::IMREAD_COLOR);
    assert(!decoded.empty());
    const auto pixel = decoded.at<cv::Vec3b>(5, 5);
    assert(pixel[2] > pixel[1]);
    std::filesystem::remove_all(root);
  }
  {
    const auto root = std::filesystem::path("/tmp") /
      ("qyg_web_debug_failure_test_" + std::to_string(static_cast<long long>(getpid())));
    std::filesystem::create_directories(root);
    tools::web_debug::PublisherOptions options;
    options.frame_path = (root / "missing" / "frame").string();
    options.data_path = (root / "data.json").string();
    options.log_path = (root / "log.json").string();
    tools::web_debug::WebDebugPublisher publisher(options);
    cv::Mat image(16, 16, CV_8UC3, cv::Scalar(0, 0, 0));
    tools::web_debug::WebDebugContext context;

    assert(!publisher.publish(image, context, 1.0));
    assert(std::filesystem::exists(options.data_path));
    assert(std::filesystem::exists(options.log_path));
    std::filesystem::remove_all(root);
  }
  {
    cv::Mat source = cv::Mat::zeros(100, 100, CV_8UC3);
    tools::web_debug::OverlaySnapshot overlays;
    overlays.detections.push_back({{{10, 10}, {30, 10}, {30, 30}, {10, 30}}, "red one", 0.9});
    overlays.projected_armors.push_back(
      {{{70, 70}, {90, 70}, {90, 90}, {70, 90}}, true, 0});
    tools::web_debug::WebDebugContext context;

    tools::web_debug::WebDebugRenderer renderer;
    const auto rendered = renderer.render(source, context, overlays);
    const auto source_center = source.at<cv::Vec3b>(50, 50);
    const auto cross_center = rendered.at<cv::Vec3b>(50, 50);
    const auto detection_corner = rendered.at<cv::Vec3b>(10, 10);
    const auto selected_corner = rendered.at<cv::Vec3b>(70, 70);
    assert(source_center == cv::Vec3b(0, 0, 0));
    assert(cross_center[0] > 200 && cross_center[1] > 200 && cross_center[2] > 200);
    assert(detection_corner[1] > 200 && detection_corner[2] > 200);
    assert(selected_corner[2] > 200);
  }
  {
    tools::ThreadSafeQueue<int> queue(2);
    assert(queue.size() == 0);
    queue.push(7);
    assert(queue.size() == 1);
  }
  {
    auto_aim::Solver solver("configs/sentry.yaml");
    const auto_aim::Solver & read_only_solver = solver;
    const Eigen::Vector3d point_in_front(1.1307102, 0.0231314, 0.0942460);
    const auto pixels = read_only_solver.project_world_points({point_in_front});
    assert(pixels.size() == 1);
    assert(pixels.front().has_value());
    assert(std::abs(pixels.front()->x - 717.2624) < 0.2);
    assert(std::abs(pixels.front()->y - 582.6854) < 0.2);
  }
  {
    auto_aim::Target target(3.0, 2.5, 0.3, 0.1);
    target.name = auto_aim::ArmorName::outpost;
    target.armor_type = auto_aim::ArmorType::small;
    target.last_id = 2;
    const auto snapshot = auto_aim::make_web_debug_target(target);
    assert(snapshot.name == "outpost");
    assert(snapshot.type == "small");
    assert(snapshot.x_m == 3.0);
    assert(snapshot.yaw_speed_radps == 2.5);
    assert(snapshot.radius_m == 0.3);
    assert(snapshot.last_id == 2);
  }
  return 0;
}
