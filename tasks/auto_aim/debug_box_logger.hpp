#ifndef AUTO_AIM__DEBUG_BOX_LOGGER_HPP
#define AUTO_AIM__DEBUG_BOX_LOGGER_HPP

#include <chrono>
#include <fstream>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <optional>
#include <string>
#include <vector>

namespace auto_aim
{

class DebugBoxLogger
{
public:
  explicit DebugBoxLogger(const std::string & name);

  const std::string & path() const;

  void log_box(
    const std::string & color, const std::string & source, int frame,
    std::chrono::steady_clock::time_point timestamp, int index,
    const std::vector<cv::Point2f> & points, const std::string & label = "",
    bool valid = true);

private:
  std::ofstream file_;
  std::mutex mutex_;
  std::string path_;
  std::chrono::system_clock::time_point system_start_;
  std::chrono::steady_clock::time_point steady_start_;
  std::optional<std::chrono::steady_clock::time_point> first_frame_timestamp_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DEBUG_BOX_LOGGER_HPP
