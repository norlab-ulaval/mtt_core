// Linux SocketCAN implementation of ICanInterface.
// Uses native kernel sockets (linux/can.h) + epoll for sub-millisecond receive.
// Replaces python-can (ThreadSafeBus) and the fcntl/ioctl interface_exists() hack.

#pragma once

#include <atomic>
#include <string>

#include "mtt_driver/hardware/can_interface.hpp"

namespace mtt::hardware {

class LinuxSocketCan final : public ICanInterface {
public:
  LinuxSocketCan() = default;
  ~LinuxSocketCan() override;

  bool open(const std::string& interface_name) override;
  bool send(const CanFrame& frame) override;
  std::optional<CanFrame> receive(std::chrono::milliseconds timeout) override;
  void close() override;
  bool is_open() const override { return socket_fd_ >= 0; }
  bool try_recover() override;
  const std::string& interface_name() const override { return interface_name_; }

private:
  int socket_fd_{-1};
  int epoll_fd_{-1};
  std::string interface_name_;

  bool bind_socket();
};

}  // namespace mtt::hardware
