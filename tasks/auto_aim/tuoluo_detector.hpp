#ifndef AUTO_AIM__TUOLUO_DETECTOR_HPP
#define AUTO_AIM__TUOLUO_DETECTOR_HPP

#include <cmath>
#include <stdexcept>

namespace auto_aim
{
struct TuoLuoConfig
{
  double enter_speed_radps = 2.2;
  double exit_speed_radps = 1.5;
  double enter_duration_s = 0.10;
  double exit_duration_s = 0.30;
  double filter_alpha = 0.20;
};

class TuoLuoDetector
{
public:
  explicit TuoLuoDetector(TuoLuoConfig config = {}) : config_{config}
  {
    if (
      config_.enter_speed_radps <= config_.exit_speed_radps || config_.exit_speed_radps < 0.0 ||
      config_.enter_duration_s <= 0.0 || config_.exit_duration_s <= 0.0 ||
      config_.filter_alpha <= 0.0 || config_.filter_alpha > 1.0) {
      throw std::invalid_argument{"invalid TuoLuo detector configuration"};
    }
  }

  bool update(double angular_speed_radps, double dt_s, bool reliable)
  {
    if (!reliable || !std::isfinite(angular_speed_radps) || !std::isfinite(dt_s) || dt_s <= 0.0) {
      enter_elapsed_s_ = 0.0;
      exit_elapsed_s_ = 0.0;
      return TuoLuo;
    }

    if (!filter_initialized_) {
      filtered_w_radps_ = angular_speed_radps;
      filter_initialized_ = true;
    } else {
      filtered_w_radps_ = (1.0 - config_.filter_alpha) * filtered_w_radps_ +
                          config_.filter_alpha * angular_speed_radps;
    }

    const double abs_w = std::abs(filtered_w_radps_);
    if (!TuoLuo) {
      exit_elapsed_s_ = 0.0;
      enter_elapsed_s_ = abs_w >= config_.enter_speed_radps ? enter_elapsed_s_ + dt_s : 0.0;
      if (enter_elapsed_s_ >= config_.enter_duration_s) {
        TuoLuo = true;
        enter_elapsed_s_ = 0.0;
      }
    } else {
      enter_elapsed_s_ = 0.0;
      exit_elapsed_s_ = abs_w <= config_.exit_speed_radps ? exit_elapsed_s_ + dt_s : 0.0;
      if (exit_elapsed_s_ >= config_.exit_duration_s) {
        TuoLuo = false;
        exit_elapsed_s_ = 0.0;
      }
    }
    return TuoLuo;
  }

  void reset()
  {
    TuoLuo = false;
    filtered_w_radps_ = 0.0;
    enter_elapsed_s_ = 0.0;
    exit_elapsed_s_ = 0.0;
    filter_initialized_ = false;
  }

  double filtered_w_radps() const { return filtered_w_radps_; }

  bool TuoLuo = false;

private:
  TuoLuoConfig config_;
  double filtered_w_radps_ = 0.0;
  double enter_elapsed_s_ = 0.0;
  double exit_elapsed_s_ = 0.0;
  bool filter_initialized_ = false;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TUOLUO_DETECTOR_HPP
