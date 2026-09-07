#include "debug_box_logger.hpp"

#include <fmt/chrono.h>
#include <fmt/core.h>
#include <nlohmann/json.hpp>

#include <filesystem>

namespace auto_aim
{

DebugBoxLogger::DebugBoxLogger(const std::string & name)
{
  system_start_ = std::chrono::system_clock::now();
  steady_start_ = std::chrono::steady_clock::now();

  std::filesystem::create_directories("logs");
  auto stamp = fmt::format("{:%Y-%m-%d_%H-%M-%S}", system_start_);
  path_ = fmt::format("logs/{}_{}.jsonl", name, stamp);
  file_.open(path_, std::ios::out | std::ios::app);

  if (file_) {
    nlohmann::json meta;
    meta["type"] = "meta";
    meta["created_at"] = stamp;
    meta["created_unix_us"] =
      std::chrono::duration_cast<std::chrono::microseconds>(system_start_.time_since_epoch())
        .count();
    meta["path"] = path_;
    file_ << meta.dump() << '\n';
    file_.flush();
  }
}

const std::string & DebugBoxLogger::path() const { return path_; }

void DebugBoxLogger::log_box(
  const std::string & color, const std::string & source, int frame,
  std::chrono::steady_clock::time_point timestamp, int index,
  const std::vector<cv::Point2f> & points, const std::string & label, bool valid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!file_) return;
  if (!first_frame_timestamp_.has_value()) first_frame_timestamp_ = timestamp;

  auto system_now = std::chrono::system_clock::now();
  auto steady_now = std::chrono::steady_clock::now();

  nlohmann::json point_list = nlohmann::json::array();
  for (const auto & point : points) {
    point_list.push_back({{"x", point.x}, {"y", point.y}});
  }

  nlohmann::json line;
  line["type"] = "box";
  line["frame"] = frame;
  line["wall_time"] = fmt::format("{:%Y-%m-%d %H:%M:%S}", system_now);
  line["wall_unix_us"] =
    std::chrono::duration_cast<std::chrono::microseconds>(system_now.time_since_epoch()).count();
  line["logger_elapsed_us"] =
    std::chrono::duration_cast<std::chrono::microseconds>(steady_now - steady_start_).count();
  line["steady_us"] =
    std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count();
  line["frame_elapsed_us"] =
    std::chrono::duration_cast<std::chrono::microseconds>(
      timestamp - first_frame_timestamp_.value())
      .count();
  line["color"] = color;
  line["source"] = source;
  line["index"] = index;
  line["valid"] = valid;
  line["label"] = label;
  line["points"] = point_list;

  file_ << line.dump() << '\n';
  file_.flush();
}

}  // namespace auto_aim
