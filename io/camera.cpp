#include "camera.hpp"

#include <optional>
#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
bool CameraBase::read_for(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
  std::chrono::milliseconds timeout)
{
  (void)timeout;
  read(img, timestamp);
  return true;
}

Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    tools::logger()->info(
      "[Camera] config:{} camera:{} exposure_ms:{:.3f} gamma:{:.3f}", config_path, camera_name,
      exposure_ms, gamma);
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    const auto gamma =
      yaml["gamma"] ? std::optional<double>{yaml["gamma"].as<double>()} : std::nullopt;
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    tools::logger()->info(
      "[Camera] config:{} camera:{} exposure_ms:{:.3f} gain:{:.3f} gamma:{}", config_path,
      camera_name, exposure_ms, gain, gamma ? std::to_string(*gamma) : std::string("disabled"));
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, gamma);
  }

  else if (camera_name == "hikrobot_gige") {
    auto gain = tools::read<double>(yaml, "gain");
    const auto gamma =
      yaml["gamma"] ? std::optional<double>{yaml["gamma"].as<double>()} : std::nullopt;
    tools::logger()->info(
      "[Camera] config:{} camera:{} exposure_ms:{:.3f} gain:{:.3f} gamma:{}", config_path,
      camera_name, exposure_ms, gain, gamma ? std::to_string(*gamma) : std::string("disabled"));
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, gamma);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

bool Camera::read_for(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
  std::chrono::milliseconds timeout)
{
  return camera_->read_for(img, timestamp, timeout);
}

}  // namespace io
