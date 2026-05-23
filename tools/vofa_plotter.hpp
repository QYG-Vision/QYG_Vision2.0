#ifndef TOOLS__VOFA_PLOTTER_HPP
#define TOOLS__VOFA_PLOTTER_HPP

#include <netinet/in.h>

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace tools
{
class VofaPlotter
{
public:
  VofaPlotter(
    std::vector<std::string> fields, std::string prefix = "pnp",
    std::string host = "127.0.0.1", uint16_t port = 1347);

  ~VofaPlotter();

  void plot(const nlohmann::json & data);

  std::string channel_description() const;

  const std::string & host() const;

  uint16_t port() const;

private:
  std::vector<std::string> fields_;
  std::string prefix_;
  std::string host_;
  uint16_t port_;
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;

  static double value_as_double(const nlohmann::json & value);
};

}  // namespace tools

#endif  // TOOLS__VOFA_PLOTTER_HPP
