#include "web_debug.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

namespace tools::web_debug
{
double rad_to_deg(double radians) { return radians * 180.0 / M_PI; }

FrameHeader make_frame_header(
  std::uint64_t stable_sequence, std::uint32_t jpeg_size, std::uint64_t monotonic_ns)
{
  FrameHeader header{};
  std::memcpy(header.magic, "QYGF", 4);
  header.version = FRAME_PROTOCOL_VERSION;
  header.header_size = sizeof(FrameHeader);
  header.sequence = stable_sequence;
  header.monotonic_ns = monotonic_ns;
  header.jpeg_size = jpeg_size;
  header.capacity = jpeg_size;
  return header;
}

bool valid_frame_header(const FrameHeader & header, std::size_t mapped_size)
{
  if (std::memcmp(header.magic, "QYGF", 4) != 0 ||
      header.version != FRAME_PROTOCOL_VERSION || header.header_size != sizeof(FrameHeader) ||
      header.sequence == 0 || (header.sequence & 1U) != 0 || header.jpeg_size == 0 ||
      header.jpeg_size > header.capacity) {
    return false;
  }
  return header.header_size <= mapped_size &&
         header.capacity <= mapped_size - header.header_size &&
         header.jpeg_size <= mapped_size - header.header_size;
}

nlohmann::json WebDebugContext::to_status_json() const
{
  const auto angle_or_null = [this](double radians) -> nlohmann::json {
    return planner.valid ? nlohmann::json(rad_to_deg(radians)) : nlohmann::json(nullptr);
  };
  nlohmann::json json{
    {"schema_version", SCHEMA_VERSION},
    {"source", system.source},
    {"system",
     {{"mode", system.mode},
      {"producer_online", system.producer_online},
      {"fps", system.fps},
      {"frame_id", system.frame_id}}},
    {"detector",
     {{"armor_count", detector.armor_count}, {"queue_depth", detector.queue_depth}}},
    {"tracker", {{"state", tracker.state}, {"has_target", tracker.has_target}}},
    {"planner",
     {{"valid", planner.valid},
      {"observation_frame_id", planner.observation_frame_id},
      {"age_ms", planner.valid ? nlohmann::json(planner.age_ms) : nlohmann::json(nullptr)},
      {"control", planner.valid ? nlohmann::json(planner.control) : nlohmann::json(nullptr)},
      {"fire", planner.valid ? nlohmann::json(planner.fire) : nlohmann::json(nullptr)},
      {"target_yaw_deg", angle_or_null(planner.target_yaw_rad)},
      {"target_pitch_deg", angle_or_null(planner.target_pitch_rad)},
      {"command_yaw_deg", angle_or_null(planner.command_yaw_rad)},
      {"command_pitch_deg", angle_or_null(planner.command_pitch_rad)}}},
    {"gimbal",
     {{"yaw_deg", rad_to_deg(gimbal.yaw_rad)},
      {"pitch_deg", rad_to_deg(gimbal.pitch_rad)},
      {"bullet_speed_mps", gimbal.bullet_speed_mps}}},
    {"timing",
     {{"frame_age_ms", timing.frame_age_ms},
      {"detector_wait_ms", timing.detector_wait_ms},
      {"tracker_ms", timing.tracker_ms},
      {"planner_ms", timing.planner_ms},
      {"publisher_ms", timing.publisher_ms}}}};

  if (target) {
    json["target"] = {
      {"name", target->name},
      {"type", target->type},
      {"x_m", target->x_m},
      {"y_m", target->y_m},
      {"z_m", target->z_m},
      {"vx_mps", target->vx_mps},
      {"vy_mps", target->vy_mps},
      {"vz_mps", target->vz_mps},
      {"yaw_deg", rad_to_deg(target->yaw_rad)},
      {"yaw_speed_radps", target->yaw_speed_radps},
      {"radius_m", target->radius_m},
      {"last_id", target->last_id}};
  } else {
    json["target"] = nullptr;
  }

  return json;
}

CurveHistory::CurveHistory(std::size_t capacity) : capacity_(capacity) {}

void CurveHistory::append(const WebDebugContext & context, double time_seconds)
{
  if (capacity_ == 0) return;
  entries_.push_back({time_seconds, context});
  while (entries_.size() > capacity_) entries_.pop_front();
}

std::size_t CurveHistory::size() const { return entries_.size(); }

nlohmann::json CurveHistory::to_json() const
{
  nlohmann::json json{
    {"schema_version", SCHEMA_VERSION},
    {"time", nlohmann::json::array()},
    {"fps", nlohmann::json::array()},
    {"frame_age_ms", nlohmann::json::array()},
    {"detector_wait_ms", nlohmann::json::array()},
    {"tracker_ms", nlohmann::json::array()},
    {"planner_ms", nlohmann::json::array()},
    {"publisher_ms", nlohmann::json::array()},
    {"armor_count", nlohmann::json::array()},
    {"detector_queue_depth", nlohmann::json::array()},
    {"target_x", nlohmann::json::array()},
    {"target_y", nlohmann::json::array()},
    {"target_z", nlohmann::json::array()},
    {"target_vx", nlohmann::json::array()},
    {"target_vy", nlohmann::json::array()},
    {"target_vz", nlohmann::json::array()},
    {"target_yaw", nlohmann::json::array()},
    {"target_yaw_speed", nlohmann::json::array()},
    {"target_yaw_cmd", nlohmann::json::array()},
    {"target_pitch_cmd", nlohmann::json::array()},
    {"command_yaw", nlohmann::json::array()},
    {"command_pitch", nlohmann::json::array()},
    {"gimbal_yaw", nlohmann::json::array()},
    {"gimbal_pitch", nlohmann::json::array()},
    {"yaw_error", nlohmann::json::array()},
    {"pitch_error", nlohmann::json::array()},
    {"control", nlohmann::json::array()},
    {"fire", nlohmann::json::array()}};

  for (const auto & entry : entries_) {
    const auto & context = entry.context;
    json["time"].push_back(entry.time_seconds);
    json["fps"].push_back(context.system.fps);
    json["frame_age_ms"].push_back(context.timing.frame_age_ms);
    json["detector_wait_ms"].push_back(context.timing.detector_wait_ms);
    json["tracker_ms"].push_back(context.timing.tracker_ms);
    json["planner_ms"].push_back(context.timing.planner_ms);
    json["publisher_ms"].push_back(context.timing.publisher_ms);
    json["armor_count"].push_back(context.detector.armor_count);
    json["detector_queue_depth"].push_back(context.detector.queue_depth);

    if (context.target) {
      json["target_x"].push_back(context.target->x_m);
      json["target_y"].push_back(context.target->y_m);
      json["target_z"].push_back(context.target->z_m);
      json["target_vx"].push_back(context.target->vx_mps);
      json["target_vy"].push_back(context.target->vy_mps);
      json["target_vz"].push_back(context.target->vz_mps);
      json["target_yaw"].push_back(rad_to_deg(context.target->yaw_rad));
      json["target_yaw_speed"].push_back(context.target->yaw_speed_radps);
    } else {
      for (const char * key : {"target_x", "target_y", "target_z", "target_vx", "target_vy",
                               "target_vz", "target_yaw", "target_yaw_speed"}) {
        json[key].push_back(nullptr);
      }
    }

    if (context.planner.valid) {
      json["target_yaw_cmd"].push_back(rad_to_deg(context.planner.target_yaw_rad));
      json["target_pitch_cmd"].push_back(rad_to_deg(context.planner.target_pitch_rad));
      json["command_yaw"].push_back(rad_to_deg(context.planner.command_yaw_rad));
      json["command_pitch"].push_back(rad_to_deg(context.planner.command_pitch_rad));
      json["yaw_error"].push_back(
        rad_to_deg(context.planner.command_yaw_rad - context.gimbal.yaw_rad));
      json["pitch_error"].push_back(
        rad_to_deg(context.planner.command_pitch_rad - context.gimbal.pitch_rad));
      json["control"].push_back(context.planner.control ? 1 : 0);
      json["fire"].push_back(context.planner.fire ? 1 : 0);
    } else {
      for (const char * key : {"target_yaw_cmd", "target_pitch_cmd", "command_yaw",
                               "command_pitch", "yaw_error", "pitch_error", "control", "fire"}) {
        json[key].push_back(nullptr);
      }
    }
    json["gimbal_yaw"].push_back(rad_to_deg(context.gimbal.yaw_rad));
    json["gimbal_pitch"].push_back(rad_to_deg(context.gimbal.pitch_rad));
  }
  return json;
}

WebDebugPublisher::WebDebugPublisher(PublisherOptions options)
: options_(std::move(options)), history_(600)
{
  open_frame();
}

WebDebugPublisher::~WebDebugPublisher()
{
  if (frame_map_ != nullptr && frame_map_ != MAP_FAILED) {
    munmap(frame_map_, options_.frame_size);
    frame_map_ = nullptr;
  }
  if (frame_fd_ >= 0) {
    close(frame_fd_);
    frame_fd_ = -1;
  }
}

bool WebDebugPublisher::open_frame()
{
  if (options_.frame_size <= sizeof(FrameHeader)) return false;
  frame_fd_ = open(options_.frame_path.c_str(), O_RDWR | O_CREAT, 0644);
  if (frame_fd_ < 0) return false;
  if (ftruncate(frame_fd_, static_cast<off_t>(options_.frame_size)) != 0) return false;
  frame_map_ = mmap(nullptr, options_.frame_size, PROT_READ | PROT_WRITE, MAP_SHARED, frame_fd_, 0);
  if (frame_map_ == MAP_FAILED) {
    frame_map_ = nullptr;
    return false;
  }
  return true;
}

bool WebDebugPublisher::ready() const noexcept
{
  return frame_map_ != nullptr && frame_fd_ >= 0;
}

bool WebDebugPublisher::write_json_atomic(
  const std::string & path, const nlohmann::json & json) const noexcept
{
  try {
    const std::string temporary = path + ".tmp." + std::to_string(getpid());
    {
      std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
      if (!output) return false;
      output << json.dump();
      output.flush();
      if (!output) return false;
    }
    return rename(temporary.c_str(), path.c_str()) == 0;
  } catch (...) {
    return false;
  }
}

bool WebDebugPublisher::publish(
  const cv::Mat & image, const WebDebugContext & context, double time_seconds) noexcept
{
  if (image.empty()) return false;
  try {
    const auto now = std::chrono::steady_clock::now();
    const bool frame_due = last_frame_publish_.time_since_epoch().count() == 0 ||
                           now - last_frame_publish_ >= options_.frame_interval;
    const bool json_due = last_json_publish_.time_since_epoch().count() == 0 ||
                          now - last_json_publish_ >= options_.json_interval;
    if (!frame_due && !json_due) return true;

    bool frame_ok = true;
    bool json_ok = true;
    std::vector<unsigned char> jpeg;
    if (frame_due) {
      frame_ok = ready() && cv::imencode(
                              ".jpg", image, jpeg,
                              {cv::IMWRITE_JPEG_QUALITY,
                               std::clamp(options_.jpeg_quality, 1, 100)}) &&
                 jpeg.size() <= options_.frame_size - sizeof(FrameHeader);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_due && frame_ok) {
      auto * bytes = static_cast<unsigned char *>(frame_map_);
      auto * header = reinterpret_cast<FrameHeader *>(bytes);
      const std::uint64_t stable_sequence = sequence_ + 2;
      __atomic_store_n(&header->sequence, stable_sequence - 1, __ATOMIC_SEQ_CST);
      std::memcpy(bytes + sizeof(FrameHeader), jpeg.data(), jpeg.size());
      auto next_header = make_frame_header(
        stable_sequence, static_cast<std::uint32_t>(jpeg.size()),
        static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()));
      next_header.capacity = static_cast<std::uint32_t>(options_.frame_size - sizeof(FrameHeader));
      constexpr auto sequence_offset = offsetof(FrameHeader, sequence);
      constexpr auto after_sequence = sequence_offset + sizeof(std::uint64_t);
      std::memcpy(bytes, &next_header, sequence_offset);
      std::memcpy(
        bytes + after_sequence,
        reinterpret_cast<const unsigned char *>(&next_header) + after_sequence,
        sizeof(FrameHeader) - after_sequence);
      __atomic_thread_fence(__ATOMIC_RELEASE);
      __atomic_store_n(&header->sequence, stable_sequence, __ATOMIC_RELEASE);
      sequence_ = stable_sequence;
    }
    if (frame_due) last_frame_publish_ = now;
    if (json_due) {
      history_.append(context, time_seconds);
      const bool log_ok = write_json_atomic(options_.log_path, context.to_status_json());
      const bool data_ok = write_json_atomic(options_.data_path, history_.to_json());
      json_ok = log_ok && data_ok;
      last_json_publish_ = now;
    }
    return frame_ok && json_ok;
  } catch (...) {
    return false;
  }
}

bool WebDebugPublisher::publish_status(
  const WebDebugContext & context, double time_seconds) noexcept
{
  try {
    const auto now = std::chrono::steady_clock::now();
    if (last_json_publish_.time_since_epoch().count() != 0 &&
        now - last_json_publish_ < options_.json_interval) {
      return true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    history_.append(context, time_seconds);
    const bool log_ok = write_json_atomic(options_.log_path, context.to_status_json());
    const bool data_ok = write_json_atomic(options_.data_path, history_.to_json());
    last_json_publish_ = now;
    return log_ok && data_ok;
  } catch (...) {
    return false;
  }
}

namespace
{
void draw_polygon(cv::Mat & image, const std::vector<cv::Point2f> & points, const cv::Scalar & color)
{
  if (points.size() < 2) return;
  for (std::size_t i = 0; i < points.size(); ++i) {
    cv::line(image, points[i], points[(i + 1) % points.size()], color, 2, cv::LINE_8);
  }
}
}  // namespace

cv::Mat WebDebugRenderer::render(
  const cv::Mat & source, const WebDebugContext & context, const OverlaySnapshot & overlays) const
{
  if (source.empty()) return {};
  cv::Mat image = source.clone();

  for (const auto & detection : overlays.detections) {
    draw_polygon(image, detection.points, cv::Scalar(0, 255, 255));
    if (!detection.points.empty()) {
      std::ostringstream label;
      label << detection.label << ' ' << std::fixed << std::setprecision(2) << detection.confidence;
      cv::putText(
        image, label.str(), detection.points.front() + cv::Point2f(0, -5),
        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
  }

  static const cv::Scalar colors[] = {
    {255, 200, 0}, {255, 0, 255}, {255, 100, 100}, {0, 165, 255}};
  for (const auto & armor : overlays.projected_armors) {
    draw_polygon(
      image, armor.points,
      armor.selected ? cv::Scalar(0, 0, 255) : colors[armor.color_index % std::size(colors)]);
  }

  if (overlays.target_center && overlays.velocity_endpoint) {
    cv::arrowedLine(
      image, *overlays.target_center, *overlays.velocity_endpoint, cv::Scalar(0, 255, 0), 2,
      cv::LINE_AA, 0, 0.2);
  }
  if (overlays.target_center && overlays.yaw_endpoint) {
    cv::arrowedLine(
      image, *overlays.target_center, *overlays.yaw_endpoint, cv::Scalar(255, 255, 0), 2,
      cv::LINE_AA, 0, 0.2);
  }

  const cv::Point center(image.cols / 2, image.rows / 2);
  cv::line(image, center + cv::Point(-10, 0), center + cv::Point(10, 0), cv::Scalar(255, 255, 255), 1);
  cv::line(image, center + cv::Point(0, -10), center + cv::Point(0, 10), cv::Scalar(255, 255, 255), 1);

  std::ostringstream top;
  top << context.system.mode << " | FPS " << std::fixed << std::setprecision(1) << context.system.fps
      << " | Q " << context.detector.queue_depth << " | " << context.tracker.state;
  if (context.target) top << " | ID " << context.target->last_id;
  cv::putText(
    image, top.str(), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1,
    cv::LINE_AA);

  std::ostringstream bottom;
  if (context.planner.valid) {
    bottom << "Target " << std::fixed << std::setprecision(1)
           << rad_to_deg(context.planner.target_yaw_rad) << "/"
           << rad_to_deg(context.planner.target_pitch_rad) << "  Cmd "
           << rad_to_deg(context.planner.command_yaw_rad) << "/"
           << rad_to_deg(context.planner.command_pitch_rad) << "  Gimbal "
           << rad_to_deg(context.gimbal.yaw_rad) << "/" << rad_to_deg(context.gimbal.pitch_rad)
           << "  C/F " << context.planner.control << "/" << context.planner.fire;
  } else {
    bottom << "Planner —  Gimbal " << std::fixed << std::setprecision(1)
           << rad_to_deg(context.gimbal.yaw_rad) << "/" << rad_to_deg(context.gimbal.pitch_rad);
  }
  cv::putText(
    image, bottom.str(), {10, std::max(20, image.rows - 15)}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  return image;
}

AsyncWebDebugPublisher::AsyncWebDebugPublisher(PublisherOptions options)
: publisher_(std::move(options)), worker_(&AsyncWebDebugPublisher::worker_loop, this)
{
}

AsyncWebDebugPublisher::~AsyncWebDebugPublisher()
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stopping_ = true;
  }
  queue_condition_.notify_one();
  if (worker_.joinable()) worker_.join();
}

bool AsyncWebDebugPublisher::submit(
  const cv::Mat & image, WebDebugContext context, OverlaySnapshot overlays,
  double time_seconds) noexcept
{
  return submit(
    image, std::move(context),
    [overlays = std::move(overlays)]() mutable { return std::move(overlays); }, time_seconds);
}

bool AsyncWebDebugPublisher::submit_status(
  WebDebugContext context, double time_seconds) noexcept
{
  try {
    Packet packet{cv::Mat{}, std::move(context), {}, time_seconds};
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) return false;
      if (pending_) dropped_count_.fetch_add(1, std::memory_order_relaxed);
      pending_ = std::move(packet);
    }
    queue_condition_.notify_one();
    return true;
  } catch (...) {
    failure_count_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
}

bool AsyncWebDebugPublisher::submit(
  const cv::Mat & image, WebDebugContext context, OverlayFactory overlay_factory,
  double time_seconds) noexcept
{
  if (image.empty()) return false;
  if (!overlay_factory) return false;
  try {
    Packet packet{image.clone(), std::move(context), std::move(overlay_factory), time_seconds};
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) return false;
      if (pending_) dropped_count_.fetch_add(1, std::memory_order_relaxed);
      pending_ = std::move(packet);
    }
    queue_condition_.notify_one();
    return true;
  } catch (...) {
    failure_count_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
}

bool AsyncWebDebugPublisher::ready() const noexcept { return publisher_.ready(); }

std::uint64_t AsyncWebDebugPublisher::failure_count() const noexcept
{
  return failure_count_.load(std::memory_order_relaxed);
}

std::uint64_t AsyncWebDebugPublisher::dropped_count() const noexcept
{
  return dropped_count_.load(std::memory_order_relaxed);
}

double AsyncWebDebugPublisher::last_publish_ms() const noexcept
{
  return last_publish_ms_.load(std::memory_order_relaxed);
}

void AsyncWebDebugPublisher::worker_loop() noexcept
{
  while (true) {
    std::optional<Packet> packet;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_condition_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
      if (pending_) {
        packet = std::move(pending_);
        pending_.reset();
      } else if (stopping_) {
        return;
      }
    }

    const auto begin = std::chrono::steady_clock::now();
    try {
      bool published = false;
      if (packet->image.empty()) {
        published = publisher_.publish_status(packet->context, packet->time_seconds);
      } else {
        const auto overlays = packet->overlay_factory();
        const cv::Mat debug_image = renderer_.render(packet->image, packet->context, overlays);
        published = !debug_image.empty() &&
                    publisher_.publish(debug_image, packet->context, packet->time_seconds);
      }
      if (!published) {
        failure_count_.fetch_add(1, std::memory_order_relaxed);
      }
    } catch (...) {
      failure_count_.fetch_add(1, std::memory_order_relaxed);
    }
    last_publish_ms_.store(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count(),
      std::memory_order_relaxed);
  }
}

}  // namespace tools::web_debug
