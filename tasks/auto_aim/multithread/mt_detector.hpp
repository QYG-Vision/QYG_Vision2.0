#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <chrono>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <tuple>

#include "tasks/auto_aim/yolos/yolov5.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & config_path, bool debug = false);

  void push(cv::Mat img, std::chrono::steady_clock::time_point t);
  void push(cv::Mat img, std::chrono::steady_clock::time_point t, std::uint64_t capture_generation);

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();  //暂时不支持yolov8

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

  bool debug_pop_for(
    cv::Mat & img, std::list<Armor> & armors, std::chrono::steady_clock::time_point & timestamp,
    std::chrono::milliseconds timeout);
  bool debug_pop_for(
    cv::Mat & img, std::list<Armor> & armors, std::chrono::steady_clock::time_point & timestamp,
    std::uint64_t & capture_generation, std::chrono::milliseconds timeout);

private:
  using PendingResult =
    std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest, std::uint64_t>;
  using DebugResult =
    std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point, std::uint64_t>;

  DebugResult postprocess_debug_result(PendingResult result);

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  std::string device_;
  YOLO yolo_;

  tools::ThreadSafeQueue<PendingResult, true> queue_{
    16, [] { tools::logger()->debug("[MultiThreadDetector] queue is full!"); }};
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP
