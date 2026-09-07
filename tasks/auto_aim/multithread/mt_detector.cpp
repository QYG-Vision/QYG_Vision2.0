#include "mt_detector.hpp"

#include <utility>

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();

  auto model = core_.read_model(model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})  // TODO
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));

  tools::logger()->info("[MultiThreadDetector] initialized !");
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  push(std::move(img), t, 0);
}

void MultiThreadDetector::push(
  cv::Mat img, std::chrono::steady_clock::time_point t, std::uint64_t capture_generation)
{
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, input(roi), {w, h});

  auto input_port = compiled_model_.input();
  auto infer_request = compiled_model_.create_infer_request();
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  infer_request.set_input_tensor(input_tensor);
  // Keep inference inside this worker thread so OpenVINO never reads from
  // the local input buffer after it has gone out of scope.
  infer_request.infer();
  queue_.push({img.clone(), t, std::move(infer_request), capture_generation});
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto [img, t, infer_request, capture_generation] = queue_.pop();
  static_cast<void>(capture_generation);
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI

  return {std::move(armors), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  auto [img, armors, timestamp, capture_generation] = postprocess_debug_result(queue_.pop());
  static_cast<void>(capture_generation);
  return {std::move(img), std::move(armors), timestamp};
}

bool MultiThreadDetector::debug_pop_for(
  cv::Mat & img, std::list<Armor> & armors, std::chrono::steady_clock::time_point & timestamp,
  std::chrono::milliseconds timeout)
{
  std::uint64_t ignored_generation = 0;
  return debug_pop_for(img, armors, timestamp, ignored_generation, timeout);
}

bool MultiThreadDetector::debug_pop_for(
  cv::Mat & img, std::list<Armor> & armors, std::chrono::steady_clock::time_point & timestamp,
  std::uint64_t & capture_generation, std::chrono::milliseconds timeout)
{
  PendingResult result;
  if (!queue_.pop_for(result, timeout)) return false;

  std::tie(img, armors, timestamp, capture_generation) =
    postprocess_debug_result(std::move(result));
  return true;
}

MultiThreadDetector::DebugResult MultiThreadDetector::postprocess_debug_result(PendingResult result)
{
  auto [img, t, infer_request, capture_generation] = std::move(result);
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI

  return {img, std::move(armors), t, capture_generation};
}

}  // namespace multithread

}  // namespace auto_aim
