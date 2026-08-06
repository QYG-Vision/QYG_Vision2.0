#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include <yaml-cpp/yaml.h>

const std::string keys =
  "{help h usage ? |                      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_sentry.yaml | 配置文件路径（含相机+检测参数）}";

// 装甲板模型点（物体系：X=0平面，Y=宽度，Z=灯条方向）
constexpr double LIGHTBAR_LEN = 53e-3;
const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0,  230e-3 / 2,  LIGHTBAR_LEN / 2},
  {0, -230e-3 / 2,  LIGHTBAR_LEN / 2},
  {0, -230e-3 / 2, -LIGHTBAR_LEN / 2},
  {0,  230e-3 / 2, -LIGHTBAR_LEN / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0,  145e-3 / 2,  LIGHTBAR_LEN / 2},
  {0, -145e-3 / 2,  LIGHTBAR_LEN / 2},
  {0, -145e-3 / 2, -LIGHTBAR_LEN / 2},
  {0,  145e-3 / 2, -LIGHTBAR_LEN / 2}};

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
  auto node = std::make_shared<rclcpp::Node>("camera_pnp_solve");
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/camera/pnp_solve", 10);
  auto camera_info_pub =
    node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/camera/armor_markers", 10);

  tools::Exiter exiter;
  tools::logger()->info("[CameraPnP] Opening camera, detector and solver...");

  io::Camera camera(config_path);
  auto_aim::multithread::MultiThreadDetector detector(config_path, true);

  // 直接从 YAML 读取相机内参（不依赖 Solver）
  auto yaml = YAML::LoadFile(config_path);
  auto cm_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto dc_data = yaml["distort_coeffs"].as<std::vector<double>>();
  cv::Mat camera_matrix(3, 3, CV_64F);
  cv::Mat distort_coeffs(1, 5, CV_64F);
  for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++)
    camera_matrix.at<double>(r, c) = cm_data[r * 3 + c];
  for (int c = 0; c < 5; c++) distort_coeffs.at<double>(c) = dc_data[c];

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.distortion_model = "plumb_bob";
  camera_info.d = dc_data;
  for (size_t i = 0; i < camera_info.k.size(); ++i) camera_info.k[i] = cm_data[i];
  camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info.p = {
    cm_data[0], cm_data[1], cm_data[2], 0.0,
    cm_data[3], cm_data[4], cm_data[5], 0.0,
    cm_data[6], cm_data[7], cm_data[8], 0.0};

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_fps_time = std::chrono::steady_clock::now();
  auto last_pnp_warning_time = std::chrono::steady_clock::time_point{};
  int frame_count = 0;
  int previous_armor_marker_count = 0;

  auto warn_pnp_failure = [&](const std::string & reason) {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_pnp_warning_time).count() >= 1000)
    {
      tools::logger()->warn("[CameraPnP] PnP skipped: {}", reason);
      last_pnp_warning_time = now;
    }
  };

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
    std::vector<const auto_aim::Armor *> solved_armors;
    solved_armors.reserve(armors.size());

    for (auto & armor : armors) {
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

      if (armor.points.size() != 4) {
        warn_pnp_failure(fmt::format("expected 4 image points, got {}", armor.points.size()));
        tools::draw_text(display, "PnP skipped: invalid points",
          {armor.box.x, armor.box.y - 15}, {0, 0, 255}, 0.55, 1);
        continue;
      }

      // PnP 求解（IPPE，直接操作，不依赖 Solver）
      try {
        const auto & object_points =
          (armor.type == auto_aim::ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

        std::vector<cv::Mat> rvecs, tvecs;
        const int solution_count = cv::solvePnPGeneric(
          object_points, armor.points, camera_matrix, distort_coeffs,
          rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
        if (solution_count <= 0 || rvecs.empty() || tvecs.empty() ||
          rvecs.size() != tvecs.size())
        {
          warn_pnp_failure("IPPE returned no valid pose candidates");
          tools::draw_text(display, "PnP failed",
            {armor.box.x, armor.box.y - 15}, {0, 0, 255}, 0.55, 1);
          continue;
        }

        // 按重投影误差选最优
        int best_idx = -1;
        double best_err = 1e10;
        for (size_t i = 0; i < rvecs.size(); ++i) {
          if (rvecs[i].empty() || tvecs[i].empty() || tvecs[i].total() != 3) continue;

          std::vector<cv::Point2f> proj;
          cv::projectPoints(object_points, rvecs[i], tvecs[i],
            camera_matrix, distort_coeffs, proj);
          if (proj.size() != armor.points.size()) continue;

          double err = 0;
          for (int j = 0; j < 4; j++) err += cv::norm(armor.points[j] - proj[j]);
          if (!std::isfinite(err)) continue;
          if (err < best_err) { best_err = err; best_idx = static_cast<int>(i); }
        }
        if (best_idx < 0) {
          warn_pnp_failure("all IPPE pose candidates were invalid");
          tools::draw_text(display, "PnP failed",
            {armor.box.x, armor.box.y - 15}, {0, 0, 255}, 0.55, 1);
          continue;
        }

        cv::Mat tvec = tvecs[best_idx];
        cv::Mat rmat;
        cv::Rodrigues(rvecs[best_idx], rmat);
        cv::cv2eigen(tvec, armor.xyz_in_camera);
        cv::cv2eigen(rmat, armor.R_armor2camera);

        // 修正 PnP 背面解：翻转 X(法向) 和 Z(灯条) 保持右手系
        armor.R_armor2camera.col(0) *= -1;
        armor.R_armor2camera.col(2) *= -1;

        if (!armor.xyz_in_camera.allFinite() || !armor.R_armor2camera.allFinite()) {
          warn_pnp_failure("selected IPPE pose contains non-finite values");
          tools::draw_text(display, "PnP failed",
            {armor.box.x, armor.box.y - 15}, {0, 0, 255}, 0.55, 1);
          continue;
        }
      } catch (const cv::Exception & e) {
        warn_pnp_failure(fmt::format("OpenCV exception: {}", e.what()));
        tools::draw_text(display, "PnP exception",
          {armor.box.x, armor.box.y - 15}, {0, 0, 255}, 0.55, 1);
        continue;
      }
      solved_armors.push_back(&armor);

      // 装甲板中心投影回图像（红X标记）
      {
        std::vector<cv::Point3f> center_cam = {{
          static_cast<float>(armor.xyz_in_camera.x()),
          static_cast<float>(armor.xyz_in_camera.y()),
          static_cast<float>(armor.xyz_in_camera.z())}};
        std::vector<cv::Point2f> px;
        cv::projectPoints(center_cam, cv::Mat::zeros(3, 1, CV_64F),
          cv::Mat::zeros(3, 1, CV_64F), camera_matrix, distort_coeffs, px);
        if (!px.empty()) {
          cv::drawMarker(display, px[0], {0, 0, 255}, cv::MARKER_CROSS, 20, 2);
          tools::draw_text(display, "center",
            {static_cast<int>(px[0].x) + 10, static_cast<int>(px[0].y) - 10},
            {0, 0, 255}, 0.5, 1);
        }
      }

      auto xyz = fmt::format("X={:.3f} Y={:.3f} Z={:.3f}m",
        armor.xyz_in_camera.x(),
        armor.xyz_in_camera.y(),
        armor.xyz_in_camera.z());
      tools::draw_text(display, xyz,
        {armor.box.x, armor.box.y - 15}, {0, 255, 255}, 0.55, 1);

      // 姿态 (欧拉角, deg)
      Eigen::Vector3d ypr = tools::eulers(armor.R_armor2camera, 2, 1, 0);
      auto rpy = fmt::format("Y={:.1f} P={:.1f} R={:.1f}deg",
        ypr[0]*57.3, ypr[1]*57.3, ypr[2]*57.3);
      tools::draw_text(display, rpy,
        {armor.box.x, armor.box.y - 30}, {255, 200, 0}, 0.55, 1);

      // 装甲板信息
      auto phys = fmt::format("model: {:.0f}x{:.0f}mm",
        (armor.type == auto_aim::ArmorType::big ? 230.0 : 145.0), 53.0);
      tools::draw_text(display, phys,
        {armor.box.x, armor.box.y + armor.box.height + 10}, {200,200,200}, 0.5, 1);

      auto info = fmt::format("{} {} {} conf={:.2f}",
        auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
        auto_aim::ARMOR_TYPES[armor.type], armor.confidence);
      tools::draw_text(display, info, armor.center, {0, 255, 0}, 0.8, 2);
    }

    // 发布 3D Marker
    {
      visualization_msgs::msg::MarkerArray marker_array;
      int id = 0;
      for (const auto * armor_ptr : solved_armors) {
        const auto & armor = *armor_ptr;
        auto now = node->now();
        auto make_marker = [&](int type, double r, double g, double b, double a = 1.0) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = "camera_optical_frame";
          m.header.stamp = now;
          m.ns = "armor";
          m.id = id++;
          m.type = type;
          m.action = visualization_msgs::msg::Marker::ADD;
          // camera_optical_frame 遵循 ROS 光学坐标系：X右、Y下、Z前。
          m.pose.position.x = armor.xyz_in_camera.x();
          m.pose.position.y = armor.xyz_in_camera.y();
          m.pose.position.z = armor.xyz_in_camera.z();
          m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
          return m;
        };

        // 🟡 球：装甲板中心
        auto sphere = make_marker(visualization_msgs::msg::Marker::SPHERE, 1.0, 1.0, 0.0, 0.6);
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.03;
        marker_array.markers.push_back(sphere);

        // 🔴 装甲板 X（法向），已经位于 camera_optical_frame。
        auto arrow_x = make_marker(visualization_msgs::msg::Marker::ARROW, 1.0, 0.0, 0.0, 0.8);
        arrow_x.scale.x = 0.08; arrow_x.scale.y = 0.01; arrow_x.scale.z = 0.01;
        {
          Eigen::Vector3d ax_optical = armor.R_armor2camera.col(0).normalized();
          double dot = Eigen::Vector3d::UnitX().dot(ax_optical);
          Eigen::Quaterniond qx;
          if (dot > 0.9999) { qx.setIdentity(); }
          else if (dot < -0.9999) { qx = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())); }
          else { qx.setFromTwoVectors(Eigen::Vector3d::UnitX(), ax_optical); }
          arrow_x.pose.orientation.w = qx.w(); arrow_x.pose.orientation.x = qx.x();
          arrow_x.pose.orientation.y = qx.y(); arrow_x.pose.orientation.z = qx.z();
        }
        marker_array.markers.push_back(arrow_x);

        // 🟢 装甲板 Y（横宽），已经位于 camera_optical_frame。
        auto arrow_y = make_marker(visualization_msgs::msg::Marker::ARROW, 0.0, 1.0, 0.0, 0.8);
        arrow_y.scale.x = 0.08; arrow_y.scale.y = 0.01; arrow_y.scale.z = 0.01;
        {
          Eigen::Vector3d ay_optical = armor.R_armor2camera.col(1).normalized();
          double dot = Eigen::Vector3d::UnitX().dot(ay_optical);
          Eigen::Quaterniond qy;
          if (dot > 0.9999) { qy.setIdentity(); }
          else if (dot < -0.9999) { qy = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())); }
          else { qy.setFromTwoVectors(Eigen::Vector3d::UnitX(), ay_optical); }
          arrow_y.pose.orientation.w = qy.w(); arrow_y.pose.orientation.x = qy.x();
          arrow_y.pose.orientation.y = qy.y(); arrow_y.pose.orientation.z = qy.z();
        }
        marker_array.markers.push_back(arrow_y);

        // 🔵 装甲板 Z（灯条/竖直），已经位于 camera_optical_frame。
        auto arrow_z = make_marker(visualization_msgs::msg::Marker::ARROW, 0.0, 0.0, 1.0, 0.8);
        arrow_z.scale.x = 0.08; arrow_z.scale.y = 0.01; arrow_z.scale.z = 0.01;
        {
          Eigen::Vector3d az_optical = armor.R_armor2camera.col(2).normalized();
          double dot = Eigen::Vector3d::UnitX().dot(az_optical);
          Eigen::Quaterniond qz;
          if (dot > 0.9999) { qz.setIdentity(); }
          else if (dot < -0.9999) { qz = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())); }
          else { qz.setFromTwoVectors(Eigen::Vector3d::UnitX(), az_optical); }
          arrow_z.pose.orientation.w = qz.w(); arrow_z.pose.orientation.x = qz.x();
          arrow_z.pose.orientation.y = qz.y(); arrow_z.pose.orientation.z = qz.z();
        }
        marker_array.markers.push_back(arrow_z);

        // 📝 文字：ypd_cam（相机系下的 Yaw Pitch Distance）
        auto text = make_marker(visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.0, 1.0, 1.0);
        text.scale.z = 0.04;
        {
          const auto & p = armor.xyz_in_camera;
          double dist = p.norm();
          double yaw_cam = std::atan2(p.x(), p.z()) * 57.3;   // OpenCV: +X=右, 右为正yaw
          double pitch_cam = std::atan2(-p.y(), std::hypot(p.x(), p.z())) * 57.3;
          Eigen::Vector3d ypr_c = tools::eulers(armor.R_armor2camera, 2, 1, 0);
          text.text = fmt::format("pos P:{:.1f} D:{:.2f}m\natt YPR:{:.1f} {:.1f} {:.1f}",
              pitch_cam, dist, ypr_c[0]*57.3, ypr_c[1]*57.3, ypr_c[2]*57.3);
        }
        marker_array.markers.push_back(text);
      }

      // 当前目标数量减少时，显式删除上一帧遗留的 armor Marker。
      const int current_armor_marker_count = id;
      for (int stale_id = current_armor_marker_count;
        stale_id < previous_armor_marker_count; ++stale_id)
      {
        visualization_msgs::msg::Marker stale_marker;
        stale_marker.header.frame_id = "camera_optical_frame";
        stale_marker.header.stamp = node->now();
        stale_marker.ns = "armor";
        stale_marker.id = stale_id;
        stale_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(stale_marker);
      }
      previous_armor_marker_count = current_armor_marker_count;

      // 🎯 相机系自定义坐标轴（固定在原点，与装甲板姿态区分）
      {
        auto now = node->now();
        auto make_axes_marker = [&](int id, double r, double g, double b) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = "camera_optical_frame";
          m.header.stamp = now;
          m.ns = "camera_axes";
          m.id = id;
          m.type = visualization_msgs::msg::Marker::ARROW;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = 0; m.pose.position.y = 0; m.pose.position.z = 0;
          m.scale.x = 0.30; m.scale.y = 0.02; m.scale.z = 0.02;
          m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9;
          return m;
        };

        // 🔴 optical X（右）：Marker 箭头默认沿 +X，无需旋转。
        auto cam_x = make_axes_marker(100, 1.0, 0.2, 0.2);
        cam_x.pose.orientation.w = 1;
        marker_array.markers.push_back(cam_x);

        // 🟢 optical Y（下）：将 Marker 的 +X 旋转到 +Y。
        auto cam_y = make_axes_marker(101, 0.2, 1.0, 0.2);
        Eigen::Quaterniond qy(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
        cam_y.pose.orientation.w = qy.w(); cam_y.pose.orientation.x = qy.x();
        cam_y.pose.orientation.y = qy.y(); cam_y.pose.orientation.z = qy.z();
        marker_array.markers.push_back(cam_y);

        // 🔵 optical Z（前）：将 Marker 的 +X 旋转到 +Z。
        auto cam_z = make_axes_marker(102, 0.2, 0.2, 1.0);
        Eigen::Quaterniond qz(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));
        cam_z.pose.orientation.w = qz.w(); cam_z.pose.orientation.x = qz.x();
        cam_z.pose.orientation.y = qz.y(); cam_z.pose.orientation.z = qz.z();
        marker_array.markers.push_back(cam_z);
      }

      marker_pub->publish(marker_array);
    }

    if (armors.empty())
      tools::draw_text(display, "No armor detected", {10, 30}, {128,128,128}, 0.8, 2);

    // 帧率
    frame_count++;
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_fps_time).count();
    if (dt >= 1000) {
      tools::logger()->info("[CameraPnP] {:.1f} fps, {} armors",
        frame_count * 1000.0 / dt, armors.size());
      frame_count = 0; last_fps_time = std::chrono::steady_clock::now();
    }

    const auto frame_stamp = node->now();
    sensor_msgs::msg::Image msg;
    msg.header.stamp = frame_stamp;
    msg.header.frame_id = "camera_optical_frame";
    msg.height = display.rows;
    msg.width = display.cols;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(display.step);
    msg.data.assign(display.datastart, display.dataend);
    pub->publish(msg);

    camera_info.header = msg.header;
    camera_info.height = msg.height;
    camera_info.width = msg.width;
    camera_info_pub->publish(camera_info);
  }

  if (detect_thread.joinable()) detect_thread.join();
  rclcpp::shutdown();
  return 0;
}
