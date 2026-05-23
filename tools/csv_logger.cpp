#include "csv_logger.hpp"

#include <fmt/chrono.h>

#include <chrono>
#include <filesystem>
#include <utility>

namespace tools
{
CsvLogger::CsvLogger(std::vector<std::string> fields, std::string prefix, std::string folder)
: fields_(std::move(fields))
{
  std::filesystem::create_directories(folder);
  path_ = fmt::format(
    "{}/{}_{:%Y-%m-%d_%H-%M-%S}.csv", folder, prefix, std::chrono::system_clock::now());
  writer_.open(path_);

  for (size_t i = 0; i < fields_.size(); ++i) {
    if (i > 0) writer_ << ',';
    writer_ << fields_[i];
  }
  writer_ << '\n';
}

CsvLogger::~CsvLogger()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (writer_.is_open()) writer_.close();
}

void CsvLogger::write(const nlohmann::json & data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!writer_.is_open()) return;

  for (size_t i = 0; i < fields_.size(); ++i) {
    if (i > 0) writer_ << ',';

    auto it = data.find(fields_[i]);
    if (it == data.end() || it->is_null()) {
      writer_ << 0;
    } else {
      write_value(*it);
    }
  }
  writer_ << '\n';
}

const std::string & CsvLogger::path() const { return path_; }

void CsvLogger::write_value(const nlohmann::json & value)
{
  if (value.is_boolean()) {
    writer_ << (value.get<bool>() ? 1 : 0);
  } else if (value.is_number()) {
    writer_ << value;
  } else if (value.is_string()) {
    auto text = value.get<std::string>();
    writer_ << '"';
    for (const auto c : text) {
      if (c == '"') writer_ << "\"\"";
      else writer_ << c;
    }
    writer_ << '"';
  } else {
    writer_ << 0;
  }
}

}  // namespace tools
