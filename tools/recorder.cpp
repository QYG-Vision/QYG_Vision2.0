#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

namespace tools
{
Recorder::Recorder(double fps, const std::string & name_suffix)
: init_(false), fps_(fps), queue_(1), stop_thread_(false)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;

  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  if (!name_suffix.empty()) file_name = fmt::format("{}_{}", file_name, name_suffix);
  session_dir_ = fmt::format("records/{}", file_name);
  text_path_ = fmt::format("{}/imu.txt", session_dir_);
  ekf_path_ = fmt::format("{}/ekf.txt", session_dir_);
  feedback_path_ = fmt::format("{}/gimbal_feedback.txt", session_dir_);
  runtime_path_ = fmt::format("{}/runtime.txt", session_dir_);
  raw_video_path_ = fmt::format("{}/raw.avi", session_dir_);
  debug_video_path_ = fmt::format("{}/debug.avi", session_dir_);

  std::filesystem::create_directories(session_dir_);
}

Recorder::~Recorder()
{
  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({
    cv::Mat::zeros(0, 0, 0), cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0},
    std::chrono::steady_clock::now()});
  if (saving_thread_.joinable()) saving_thread_.join();  // 等待视频保存线程结束

  if (!init_) return;
  text_writer_.close();
  ekf_writer_.close();
  feedback_writer_.close();
  runtime_writer_.close();
  raw_video_writer_.release();
  debug_video_writer_.release();
}

void Recorder::save_to_file()
{
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);  // 从队列中取出帧数据
    if (frame.raw_img.empty() || frame.debug_img.empty()) {
      tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }
    // 写入视频文件
    raw_video_writer_.write(frame.raw_img);
    debug_video_writer_.write(frame.debug_img);

    // 写入文本文件（输出顺序为wxyz）
    Eigen::Vector4d xyzw = frame.q.coeffs();
    auto since_begin = tools::delta_time(frame.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);

    // One line is emitted for every recorded video frame.  A missing target
    // is represented by NaN so the sidecar remains frame-aligned.
    if (frame.ekf_rotation) {
      ekf_writer_ << fmt::format(
        "{} {} {}\n", since_begin, (*frame.ekf_rotation)[0], (*frame.ekf_rotation)[1]);
    } else {
      ekf_writer_ << fmt::format("{} nan nan\n", since_begin);
    }

    // This is the exact yaw feedback used by the online fused orientation.
    // Keep raw_q in imu.txt for backward compatibility and persist the yaw
    // override separately so offline replay can call the same Solver API.
    if (frame.feedback_yaw_deg) {
      Eigen::Vector4d feedback_xyzw = frame.q.coeffs();
      feedback_writer_ << fmt::format(
        "{} {} {} {} {} {}\n", since_begin, feedback_xyzw[3], feedback_xyzw[0],
        feedback_xyzw[1], feedback_xyzw[2], *frame.feedback_yaw_deg);
    } else {
      feedback_writer_ << fmt::format("{} nan nan nan nan nan\n", since_begin);
    }

    runtime_writer_ << fmt::format(
      "{} {} {}\n", since_begin,
      frame.bullet_speed ? fmt::format("{}", *frame.bullet_speed) : "nan",
      frame.enemy_color ? *frame.enemy_color : "unknown");
  }
}

void Recorder::record(
  const cv::Mat & raw_img, const cv::Mat & debug_img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  if (raw_img.empty() || debug_img.empty()) return;
  if (!init_) init(raw_img, debug_img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({raw_img, debug_img, q, timestamp, std::nullopt, std::nullopt});
}

void Recorder::record(
  const cv::Mat & raw_img, const cv::Mat & debug_img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp,
  const std::optional<Eigen::Vector2d> & ekf_rotation)
{
  if (raw_img.empty() || debug_img.empty()) return;
  if (!init_) init(raw_img, debug_img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({raw_img, debug_img, q, timestamp, ekf_rotation, std::nullopt});
}

void Recorder::record(
  const cv::Mat & raw_img, const cv::Mat & debug_img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp,
  const std::optional<Eigen::Vector2d> & ekf_rotation,
  const std::optional<double> & feedback_yaw_deg)
{
  if (raw_img.empty() || debug_img.empty()) return;
  if (!init_) init(raw_img, debug_img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({raw_img, debug_img, q, timestamp, ekf_rotation, feedback_yaw_deg, std::nullopt,
               std::nullopt});
}

void Recorder::record(
  const cv::Mat & raw_img, const cv::Mat & debug_img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp,
  const std::optional<Eigen::Vector2d> & ekf_rotation,
  const std::optional<double> & feedback_yaw_deg, double bullet_speed,
  const std::optional<std::string> & enemy_color)
{
  if (raw_img.empty() || debug_img.empty()) return;
  if (!init_) init(raw_img, debug_img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({raw_img, debug_img, q, timestamp, ekf_rotation, feedback_yaw_deg, bullet_speed,
               enemy_color});
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  record(img, img, q, timestamp);
}

void Recorder::init(const cv::Mat & raw_img, const cv::Mat & debug_img)
{
  text_writer_.open(text_path_);
  ekf_writer_.open(ekf_path_);
  feedback_writer_.open(feedback_path_);
  runtime_writer_.open(runtime_path_);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  raw_video_writer_ = cv::VideoWriter(raw_video_path_, fourcc, fps_, raw_img.size());
  debug_video_writer_ = cv::VideoWriter(debug_video_path_, fourcc, fps_, debug_img.size());
  saving_thread_ = std::thread(&Recorder::save_to_file, this);  // 启动保存线程
  init_ = true;
}

}  // namespace tools
