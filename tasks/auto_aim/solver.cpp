#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
//constexpr double LIGHTBAR_LENGTH = 56e-3;    // m
constexpr double LIGHTBAR_LENGTH = 53e-3;    // m, temporary tablet/phone target size
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
//constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m
constexpr double SMALL_ARMOR_WIDTH = 145e-3;  // m, temporary tablet/phone target size
constexpr bool FORCE_IPPE_DEBUG = true;  // temporary: observe both IPPE candidates every frame

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  // Installation extrinsic: fixed rotation between gimbal frame and IMU body frame.
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  // IMU每帧给出姿态四元数q，这里先转成R_imubody2imuabs（同一旋转的矩阵表示）。
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  // 再用固定安装外参R_gimbal2imubody做轴对齐，得到当前帧R_gimbal2world。
  // 直观例子：敌人静止、云台右转30度/秒时，若不做这步，观测会在云台系里“向左漂”；
  // 做完这步再转到world系后，这部分由云台自转引入的假运动会被抵消。
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

void Solver::set_R_gimbal2world(const Eigen::Vector3d & euler)
{
  Eigen::Vector3d ypr(euler(2), euler(1), euler(0));
  Eigen::Matrix3d R_imubody2imuabs = tools::rotation_matrix(ypr);
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

//solvePnP（获得姿态）主要负责通过 PnP（Perspective-n-Point）算法计算装甲板在不同坐标系（相机、云台、世界）下的 位置（Translation） 和 姿态（Rotation）。
void Solver::solve(Armor & armor) const
{
  static auto last_debug_log = std::chrono::steady_clock::time_point{};
  auto now_debug_log = std::chrono::steady_clock::now();
  bool log_debug =
    std::chrono::duration_cast<std::chrono::milliseconds>(now_debug_log - last_debug_log).count() >=
    200;
  if (log_debug) {
    last_debug_log = now_debug_log;
    if (armor.points.size() == 4) {
      tools::logger()->info(
        "[ArmorPts] p0=({:.0f},{:.0f}) p1=({:.0f},{:.0f}) p2=({:.0f},{:.0f}) p3=({:.0f},{:.0f})",
        armor.points[0].x, armor.points[0].y,
        armor.points[1].x, armor.points[1].y,
        armor.points[2].x, armor.points[2].y,
        armor.points[3].x, armor.points[3].y);
    }
  }

  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Mat rvec, tvec;
  bool use_iterative = false;
  int best_idx = -1;

  // ===== 主路径: 用上一帧的解作为初始猜测，ITERATIVE 迭代优化 =====
  // 原理: Levenberg-Marquardt 从上一帧解的邻域出发，自然收敛到同一个局部最优
  //       根本不会产生第二个歧义解 → 从源头消灭跳变
  if (!FORCE_IPPE_DEBUG && has_prev_solution_) {
    rvec = cv::Mat(prev_rvec_);
    tvec = cv::Mat(prev_tvec_);
    cv::solvePnP(
      object_points, armor.points, camera_matrix_, distort_coeffs_,
      rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);

    // 验证迭代结果：如果初始猜测太陈旧导致收敛到垃圾解，重投影误差会很大
    std::vector<cv::Point2f> proj_pts; 
    //计算重投影误差proj_pts，进行pnp结果判断
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, proj_pts);
    double total_reproj_err = 0;
    for (int i = 0; i < 4; i++) total_reproj_err += cv::norm(armor.points[i] - proj_pts[i]);

    if (total_reproj_err < 15.0) {
      use_iterative = true;  // 迭代结果可信
    }
    // 否则 fallback 到下面的 IPPE 路径
  }

  // ===== 回退路径: 首帧 / 目标丢失重新捕获 / 迭代法发散 =====
  // 此时没有可靠的上一帧解，只能用 IPPE + IMU 先验做一次性筛选
  if (!use_iterative) {
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat reprojectionErrors;
    cv::solvePnPGeneric(
      object_points, armor.points, camera_matrix_, distort_coeffs_, rvecs, tvecs, false,
      cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), reprojectionErrors);

    best_idx = 0;
    if (rvecs.size() > 1) {
      double min_cost = 1e10;
      for (size_t i = 0; i < rvecs.size(); ++i) {
        Eigen::Vector3d xyz_in_camera_i;
        cv::cv2eigen(tvecs[i], xyz_in_camera_i);
        Eigen::Vector3d xyz_in_gimbal_i = R_camera2gimbal_ * xyz_in_camera_i + t_camera2gimbal_;
        Eigen::Vector3d xyz_in_world_i = R_gimbal2world_ * xyz_in_gimbal_i;
        Eigen::Vector3d ypd_in_world_i = tools::xyz2ypd(xyz_in_world_i);

        cv::Mat rmat_i;
        cv::Rodrigues(rvecs[i], rmat_i);
        Eigen::Matrix3d R_a2c_i;
        cv::cv2eigen(rmat_i, R_a2c_i);
        Eigen::Matrix3d R_a2w_i = R_gimbal2world_ * R_camera2gimbal_ * R_a2c_i;

        // 法向量约束
        Eigen::Vector3d armor_to_cam = -R_gimbal2world_ * xyz_in_gimbal_i;
        armor_to_cam.normalize();
        Eigen::Vector3d armor_normal = R_a2w_i.col(0);
        double normal_cost = 1.0 - armor_normal.dot(armor_to_cam);

        // Pitch先验约束，防止p4p二值性结果
        Eigen::Vector3d ypr = tools::eulers(R_a2w_i, 2, 1, 0);
        double expected_pitch = 15.0 * CV_PI / 180.0;
        if (armor.name == ArmorName::outpost) expected_pitch = -15.0 * CV_PI / 180.0;
        double pitch_cost = 0.0;
        auto is_balance = (armor.type == ArmorType::big) &&
                        (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                         armor.name == ArmorName::five);
        if (!is_balance) {
          pitch_cost = std::abs(std::abs(ypr[1]) - std::abs(expected_pitch));
        }

        double reproj_error = 0.0;
        if (!reprojectionErrors.empty()) {
          if (reprojectionErrors.depth() == CV_64F) {
            reproj_error = reprojectionErrors.at<double>(static_cast<int>(i), 0);
          } else if (reprojectionErrors.depth() == CV_32F) {
            reproj_error = reprojectionErrors.at<float>(static_cast<int>(i), 0);
          }
        }
        double cost = reproj_error + 10.0 * normal_cost + 20.0 * pitch_cost;

        if (log_debug) {
          tools::logger()->info(
            "[IPPE] i={} reproj={:.2f} normal={:.3f} pitch_cost={:.3f} ypr_w=({:.1f},{:.1f},{:.1f}) xyz_g=({:.2f},{:.2f},{:.2f}) ypd_w=({:.1f},{:.1f},{:.2f}) cost={:.2f}",
            i, reproj_error, normal_cost, pitch_cost,
            ypr[0] * 57.3, ypr[1] * 57.3, ypr[2] * 57.3,
            xyz_in_gimbal_i.x(), xyz_in_gimbal_i.y(), xyz_in_gimbal_i.z(),
            ypd_in_world_i[0] * 57.3, ypd_in_world_i[1] * 57.3, ypd_in_world_i[2], cost);
        }

        if (cost < min_cost) {
          min_cost = cost;
          best_idx = i;
        }
      }
    }
    if (log_debug) {
      tools::logger()->info("[IPPE] best_idx={}", best_idx);
    }
    rvec = rvecs[best_idx];
    tvec = tvecs[best_idx];
  }

  // 更新时序先验，供下一帧 ITERATIVE 使用
  prev_rvec_ = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
  prev_tvec_ = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  has_prev_solution_ = true;

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_camera = xyz_in_camera;
  // 坐标链：camera -> gimbal（旋转+平移）-> world（旋转）。
  // 生动理解：像“稳像”先扣掉机体转动。敌人静止、云台右转30度/秒时，
  // 在gimbal系看目标会持续左移；在world系该漂移被扣掉，目标坐标近似静止。
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  
  armor.R_armor2camera = R_armor2camera;
  armor.R_armor2gimbal = R_armor2gimbal;

  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  armor.yaw_raw = armor.ypr_in_world[0];

  if (log_debug) {
    tools::logger()->info(
      "[ArmorPose] pnp={} xyz_g=({:.2f},{:.2f},{:.2f}) xyz_w=({:.2f},{:.2f},{:.2f}) ypr_w=({:.1f},{:.1f},{:.1f}) ypd_w=({:.1f},{:.1f},{:.2f})",
      use_iterative ? "ITERATIVE" : "IPPE",
      armor.xyz_in_gimbal.x(), armor.xyz_in_gimbal.y(), armor.xyz_in_gimbal.z(),
      armor.xyz_in_world.x(), armor.xyz_in_world.y(), armor.xyz_in_world.z(),
      armor.ypr_in_world[0] * 57.3,
      armor.ypr_in_world[1] * 57.3,
      armor.ypr_in_world[2] * 57.3,
      armor.ypd_in_world[0] * 57.3,
      armor.ypd_in_world[1] * 57.3,
      armor.ypd_in_world[2]);
  }

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  optimize_yaw(armor);
}
//将世界装甲板坐标映射到二维图像坐标（reproject armor）
std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // solve
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  // 与主链路一致：camera -> gimbal（旋转+平移）-> world（旋转）。
  // 这样这里的重投影误差评估也基于“扣掉云台自转后”的world观测。
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

void Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}

// 相机坐标直接投影到像素（不经过 world/gimbal 链，用于无云台测试验证）
std::vector<cv::Point2f> Solver::camera2pixel(const std::vector<cv::Point3f> &cameraPoints) const
{
  std::vector<cv::Point3f> valid;
  for (const auto &p : cameraPoints) {
    if (p.z > 0) valid.push_back(p);
  }
  if (valid.empty()) return {};
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(valid, cv::Mat::zeros(3,1,CV_64F), cv::Mat::zeros(3,1,CV_64F),
                    camera_matrix_, distort_coeffs_, pixels);
  return pixels;
}
}  // namespace auto_aim
