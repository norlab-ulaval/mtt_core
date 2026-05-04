// Linux SocketCAN implementation — native kernel CAN sockets with epoll.
// No python-can, no third-party lib. Uses linux/can.h directly.

#include "mtt_driver/hardware/linux_socket_can.hpp"

#include <cstring>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace mtt::hardware {

LinuxSocketCan::~LinuxSocketCan()
{
  close();
}

bool LinuxSocketCan::open(const std::string& interface_name)
{
  interface_name_ = interface_name;

  // Create raw CAN socket
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) return false;

  if (!bind_socket()) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Create epoll instance for non-blocking receive
  epoll_fd_ = ::epoll_create1(0);
  if (epoll_fd_ < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  epoll_event ev{};
  ev.events   = EPOLLIN;
  ev.data.fd  = socket_fd_;
  if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, socket_fd_, &ev) < 0) {
    ::close(epoll_fd_);
    ::close(socket_fd_);
    epoll_fd_ = socket_fd_ = -1;
    return false;
  }

  return true;
}

bool LinuxSocketCan::bind_socket()
{
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) return false;

  sockaddr_can addr{};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  return ::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
}

bool LinuxSocketCan::send(const CanFrame& frame)
{
  if (socket_fd_ < 0) return false;

  can_frame raw{};
  raw.can_id  = frame.is_extended
    ? ((frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG)
    : (frame.id & CAN_SFF_MASK);
  raw.can_dlc = frame.dlc;
  std::copy(frame.data.begin(), frame.data.begin() + frame.dlc, raw.data);

  ssize_t written = ::write(socket_fd_, &raw, sizeof(raw));
  return written == sizeof(raw);
}

std::optional<CanFrame> LinuxSocketCan::receive(std::chrono::milliseconds timeout)
{
  if (socket_fd_ < 0 || epoll_fd_ < 0) return std::nullopt;

  epoll_event events[1];
  int nfds = ::epoll_wait(epoll_fd_, events, 1, static_cast<int>(timeout.count()));
  if (nfds <= 0) return std::nullopt;  // timeout or error

  can_frame raw{};
  ssize_t nbytes = ::read(socket_fd_, &raw, sizeof(raw));
  if (nbytes < static_cast<ssize_t>(sizeof(raw))) return std::nullopt;

  CanFrame frame;
  frame.is_extended = (raw.can_id & CAN_EFF_FLAG) != 0;
  frame.id  = frame.is_extended
    ? (raw.can_id & CAN_EFF_MASK)
    : (raw.can_id & CAN_SFF_MASK);
  frame.dlc = raw.can_dlc;
  std::copy(raw.data, raw.data + raw.can_dlc, frame.data.begin());
  return frame;
}

void LinuxSocketCan::close()
{
  if (epoll_fd_ >= 0) { ::close(epoll_fd_); epoll_fd_ = -1; }
  if (socket_fd_ >= 0) { ::close(socket_fd_); socket_fd_ = -1; }
}

bool LinuxSocketCan::try_recover()
{
  // Attempt a soft restart of the CAN interface via sysfs
  std::string cmd = "ip link set " + interface_name_ + " type can restart-ms 100";
  bool ok = (std::system(cmd.c_str()) == 0);
  if (ok) {
    // Re-bind after restart
    close();
    return open(interface_name_);
  }
  return false;
}

}  // namespace mtt::hardware
