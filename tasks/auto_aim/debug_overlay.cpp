#include "debug_overlay.hpp"

#include <mutex>

namespace auto_aim
{
namespace
{
std::mutex g_overlay_mutex;
DetectionOverlayData g_overlay_data;
}  // namespace

void set_detection_overlay_data(const DetectionOverlayData & data)
{
  std::lock_guard<std::mutex> lock(g_overlay_mutex);
  g_overlay_data = data;
}

DetectionOverlayData get_detection_overlay_data()
{
  std::lock_guard<std::mutex> lock(g_overlay_mutex);
  return g_overlay_data;
}

}  // namespace auto_aim
