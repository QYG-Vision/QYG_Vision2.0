#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <optional>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/thread_safe_queue.hpp"
namespace tools
{
class Recorder
{
public:
  Recorder(double fps = 30, const std::string & name_suffix = "");
  ~Recorder();
  // Record a raw camera frame, an annotated/debug frame, and the matching IMU pose.
  void record(
    const cv::Mat & raw_img, const cv::Mat & debug_img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);
  // Record the target rotation phase (rad) and angular velocity (rad/s) for
  // the same frame. Values are written to the sidecar ekf.txt file.
  void record(
    const cv::Mat & raw_img, const cv::Mat & debug_img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp,
    const std::optional<Eigen::Vector2d> & ekf_rotation);
  // Also persist the yaw feedback that was fused online with the raw IMU pose.
  void record(
    const cv::Mat & raw_img, const cv::Mat & debug_img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp,
    const std::optional<Eigen::Vector2d> & ekf_rotation,
    const std::optional<double> & feedback_yaw_deg);
  // Persist the runtime inputs consumed by Tracker/Planner for deterministic replay.
  void record(
    const cv::Mat & raw_img, const cv::Mat & debug_img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp,
    const std::optional<Eigen::Vector2d> & ekf_rotation,
    const std::optional<double> & feedback_yaw_deg, double bullet_speed,
    const std::optional<std::string> & enemy_color);
  // Backward-compatible overload: writes the same image to both video streams.
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

private:
  struct FrameData
  {
    cv::Mat raw_img;
    cv::Mat debug_img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
    std::optional<Eigen::Vector2d> ekf_rotation;
    std::optional<double> feedback_yaw_deg;
    std::optional<double> bullet_speed;
    std::optional<std::string> enemy_color;
  };
  bool init_;
  std::atomic<bool> stop_thread_;
  double fps_;
  std::string session_dir_;
  std::string text_path_;
  std::string ekf_path_;
  std::string feedback_path_;
  std::string runtime_path_;
  std::string raw_video_path_;
  std::string debug_video_path_;
  std::ofstream text_writer_;
  std::ofstream ekf_writer_;
  std::ofstream feedback_writer_;
  std::ofstream runtime_writer_;
  cv::VideoWriter raw_video_writer_;
  cv::VideoWriter debug_video_writer_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_time_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;  // 负责保存帧数据的线程
  void init(const cv::Mat & raw_img, const cv::Mat & debug_img);
  void save_to_file();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
