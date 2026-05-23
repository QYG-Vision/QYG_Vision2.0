#include "vofa_plotter.hpp"

#include <arpa/inet.h>
#include <fmt/core.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>
#include <utility>

namespace tools
{
VofaPlotter::VofaPlotter(
  std::vector<std::string> fields, std::string prefix, std::string host, uint16_t port)
: fields_(std::move(fields)), prefix_(std::move(prefix)), host_(std::move(host)), port_(port)
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port_);
  destination_.sin_addr.s_addr = ::inet_addr(host_.c_str());
}

VofaPlotter::~VofaPlotter()
{
  if (socket_ >= 0) ::close(socket_);
}

void VofaPlotter::plot(const nlohmann::json & data)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::ostringstream line;
  line << prefix_ << ':';
  for (size_t i = 0; i < fields_.size(); ++i) {
    if (i > 0) line << ',';

    auto it = data.find(fields_[i]);
    if (it == data.end() || it->is_null()) {
      line << 0.0;
    } else {
      line << value_as_double(*it);
    }
  }
  line << '\n';

  const auto frame = line.str();
  ::sendto(
    socket_, frame.c_str(), frame.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

std::string VofaPlotter::channel_description() const
{
  std::string description;
  for (size_t i = 0; i < fields_.size(); ++i) {
    if (i > 0) description += ", ";
    description += fmt::format("ch{}={}", i, fields_[i]);
  }
  return description;
}

const std::string & VofaPlotter::host() const { return host_; }

uint16_t VofaPlotter::port() const { return port_; }

double VofaPlotter::value_as_double(const nlohmann::json & value)
{
  if (value.is_boolean()) return value.get<bool>() ? 1.0 : 0.0;
  if (value.is_number()) return value.get<double>();
  return 0.0;
}

}  // namespace tools
