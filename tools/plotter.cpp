#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

namespace tools
{
Plotter::Plotter(std::string host, uint16_t port)
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port);
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter::~Plotter() { ::close(socket_); }

bool Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ < 0) return false;
  // 纯 JSON 文本（PlotJuggler UDP Server 标准格式，对应"不勾选 Multi-type dispatch"；
  // 与官方 udp_client.py / scripts/plotjuggler_test.py 的发送格式保持一致）
  const auto data = json.dump();
  const auto sent = ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
  return sent == static_cast<ssize_t>(data.length());
}

}  // namespace tools
