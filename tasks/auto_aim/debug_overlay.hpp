#ifndef AUTO_AIM__DEBUG_OVERLAY_HPP
#define AUTO_AIM__DEBUG_OVERLAY_HPP

namespace auto_aim
{

struct DetectionOverlayData
{
  bool valid = false;
  int frame_count = -1;
  bool control = false;
  bool fire = false;
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
  double fps = 0.0;
  double detect_ms = 0.0;
  int armor_count = 0;
  int target_count = 0;
};

void set_detection_overlay_data(const DetectionOverlayData & data);
DetectionOverlayData get_detection_overlay_data();

}  // namespace auto_aim

#endif  // AUTO_AIM__DEBUG_OVERLAY_HPP
