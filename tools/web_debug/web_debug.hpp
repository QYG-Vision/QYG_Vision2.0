#ifndef TOOLS__WEB_DEBUG__WEB_DEBUG_HPP
#define TOOLS__WEB_DEBUG__WEB_DEBUG_HPP

#include <atomic>
#include <cstdint>
#include <deque>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
#include <nlohmann/json.hpp>

namespace tools::web_debug
{
constexpr int SCHEMA_VERSION = 1;
constexpr std::uint16_t FRAME_PROTOCOL_VERSION = 1;

struct FrameHeader
{
  char magic[4];
  std::uint16_t version;
  std::uint16_t header_size;
  std::uint64_t sequence;
  std::uint64_t monotonic_ns;
  std::uint32_t jpeg_size;
  std::uint32_t capacity;
};
static_assert(sizeof(FrameHeader) == 32);

FrameHeader make_frame_header(
  std::uint64_t stable_sequence, std::uint32_t jpeg_size, std::uint64_t monotonic_ns);
bool valid_frame_header(const FrameHeader & header, std::size_t mapped_size);

struct SystemSnapshot
{
  std::string source = "unknown";
  std::string mode = "IDLE";
  bool producer_online = true;
  double fps = 0.0;
  std::uint64_t frame_id = 0;
};

struct DetectorSnapshot
{
  std::size_t armor_count = 0;
  std::size_t queue_depth = 0;
};

struct TrackerSnapshot
{
  std::string state = "lost";
  bool has_target = false;
};

struct TargetSnapshot
{
  std::string name;
  std::string type;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double vz_mps = 0.0;
  double yaw_rad = 0.0;
  double yaw_speed_radps = 0.0;
  double radius_m = 0.0;
  int last_id = -1;
};

struct PlannerSnapshot
{
  bool valid = false;
  std::uint64_t observation_frame_id = 0;
  double age_ms = 0.0;
  bool control = false;
  bool fire = false;
  double target_yaw_rad = 0.0;
  double target_pitch_rad = 0.0;
  double command_yaw_rad = 0.0;
  double command_pitch_rad = 0.0;
};

struct GimbalSnapshot
{
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  double bullet_speed_mps = 0.0;
};

struct TimingSnapshot
{
  double frame_age_ms = 0.0;
  double detector_wait_ms = 0.0;
  double tracker_ms = 0.0;
  double planner_ms = 0.0;
  double publisher_ms = 0.0;
};

struct WebDebugContext
{
  SystemSnapshot system;
  DetectorSnapshot detector;
  TrackerSnapshot tracker;
  std::optional<TargetSnapshot> target;
  PlannerSnapshot planner;
  GimbalSnapshot gimbal;
  TimingSnapshot timing;

  nlohmann::json to_status_json() const;
};

double rad_to_deg(double radians);

class CurveHistory
{
public:
  explicit CurveHistory(std::size_t capacity = 600);

  void append(const WebDebugContext & context, double time_seconds);
  nlohmann::json to_json() const;
  std::size_t size() const;

private:
  struct Entry
  {
    double time_seconds;
    WebDebugContext context;
  };

  std::size_t capacity_;
  std::deque<Entry> entries_;
};

struct PublisherOptions
{
  std::string frame_path = "/dev/shm/qyg_frame";
  std::string data_path = "/dev/shm/qyg_data.json";
  std::string log_path = "/dev/shm/qyg_log.json";
  std::size_t frame_size = 4 * 1024 * 1024;
  int jpeg_quality = 80;
  std::chrono::milliseconds frame_interval{17};
  std::chrono::milliseconds json_interval{50};
};

class WebDebugPublisher
{
public:
  explicit WebDebugPublisher(PublisherOptions options = {});
  ~WebDebugPublisher();

  WebDebugPublisher(const WebDebugPublisher &) = delete;
  WebDebugPublisher & operator=(const WebDebugPublisher &) = delete;

  bool publish(const cv::Mat & image, const WebDebugContext & context, double time_seconds) noexcept;
  bool publish_status(const WebDebugContext & context, double time_seconds) noexcept;
  bool ready() const noexcept;

private:
  PublisherOptions options_;
  int frame_fd_ = -1;
  void * frame_map_ = nullptr;
  std::uint64_t sequence_ = 0;
  std::chrono::steady_clock::time_point last_frame_publish_{};
  std::chrono::steady_clock::time_point last_json_publish_{};
  CurveHistory history_;
  mutable std::mutex mutex_;

  bool open_frame();
  bool write_json_atomic(const std::string & path, const nlohmann::json & json) const noexcept;
};

struct DetectionOverlay
{
  std::vector<cv::Point2f> points;
  std::string label;
  double confidence = 0.0;
};

struct ProjectedArmorOverlay
{
  std::vector<cv::Point2f> points;
  bool selected = false;
  std::size_t color_index = 0;
};

struct OverlaySnapshot
{
  std::vector<DetectionOverlay> detections;
  std::vector<ProjectedArmorOverlay> projected_armors;
  std::optional<cv::Point2f> target_center;
  std::optional<cv::Point2f> velocity_endpoint;
  std::optional<cv::Point2f> yaw_endpoint;
};

using OverlayFactory = std::function<OverlaySnapshot()>;

class WebDebugRenderer
{
public:
  cv::Mat render(
    const cv::Mat & source, const WebDebugContext & context,
    const OverlaySnapshot & overlays) const;
};

class AsyncWebDebugPublisher
{
public:
  explicit AsyncWebDebugPublisher(PublisherOptions options = {});
  ~AsyncWebDebugPublisher();

  AsyncWebDebugPublisher(const AsyncWebDebugPublisher &) = delete;
  AsyncWebDebugPublisher & operator=(const AsyncWebDebugPublisher &) = delete;

  bool submit(
    const cv::Mat & image, WebDebugContext context, OverlaySnapshot overlays,
    double time_seconds) noexcept;
  bool submit(
    const cv::Mat & image, WebDebugContext context, OverlayFactory overlay_factory,
    double time_seconds) noexcept;
  bool submit_status(WebDebugContext context, double time_seconds) noexcept;
  bool ready() const noexcept;
  std::uint64_t failure_count() const noexcept;
  std::uint64_t dropped_count() const noexcept;
  double last_publish_ms() const noexcept;

private:
  struct Packet
  {
    cv::Mat image;
    WebDebugContext context;
    OverlayFactory overlay_factory;
    double time_seconds;
  };

  WebDebugPublisher publisher_;
  WebDebugRenderer renderer_;
  std::thread worker_;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::optional<Packet> pending_;
  bool stopping_ = false;
  std::atomic<std::uint64_t> failure_count_{0};
  std::atomic<std::uint64_t> dropped_count_{0};
  std::atomic<double> last_publish_ms_{0.0};

  void worker_loop() noexcept;
};

}  // namespace tools::web_debug

#endif  // TOOLS__WEB_DEBUG__WEB_DEBUG_HPP
