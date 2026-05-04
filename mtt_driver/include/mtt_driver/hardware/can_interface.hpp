// Abstract CAN interface — hardware-agnostic contract.
// Both LinuxSocketCan (real robot) and VirtualCan (tests) implement this.

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace mtt::hardware {

struct CanFrame {
  uint32_t id{0};
  std::array<uint8_t, 8> data{};
  uint8_t dlc{8};  // data length code
  bool is_extended{false};
};

class ICanInterface {
public:
  virtual ~ICanInterface() = default;

  // Open the interface (e.g. "can0", "vcan0"). Returns true on success.
  virtual bool open(const std::string& interface_name) = 0;

  // Send an 8-byte frame. Returns true on success.
  virtual bool send(const CanFrame& frame) = 0;

  // Block until a frame arrives or timeout elapses.
  // Returns nullopt on timeout or error.
  virtual std::optional<CanFrame> receive(std::chrono::milliseconds timeout) = 0;

  // Close and release the socket/resource.
  virtual void close() = 0;

  // True when the interface is open and usable.
  virtual bool is_open() const = 0;

  // Attempt a soft recovery (e.g. after BUS-OFF). Returns true if recovered.
  virtual bool try_recover() { return false; }

  // Name of the interface as opened (e.g. "can0").
  virtual const std::string& interface_name() const = 0;
};

}  // namespace mtt::hardware
