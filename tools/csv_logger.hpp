#ifndef TOOLS__CSV_LOGGER_HPP
#define TOOLS__CSV_LOGGER_HPP

#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace tools
{
class CsvLogger
{
public:
  CsvLogger(
    std::vector<std::string> fields, std::string prefix = "plot",
    std::string folder = "logs");

  ~CsvLogger();

  void write(const nlohmann::json & data);

  const std::string & path() const;

private:
  std::vector<std::string> fields_;
  std::string path_;
  std::ofstream writer_;
  std::mutex mutex_;

  void write_value(const nlohmann::json & value);
};

}  // namespace tools

#endif  // TOOLS__CSV_LOGGER_HPP
