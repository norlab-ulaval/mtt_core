#pragma once

#include <string>

namespace mtt_control
{

enum class ControlMode
{
  Stop,
  Manual,
  Auto,
};

inline std::string to_string(ControlMode mode)
{
  switch (mode) {
    case ControlMode::Stop:
      return "STOP";
    case ControlMode::Manual:
      return "MANUAL";
    case ControlMode::Auto:
      return "AUTO";
  }
  return "STOP";
}

inline ControlMode control_mode_from_string(const std::string & value)
{
  if (value == "AUTO") {
    return ControlMode::Auto;
  }
  if (value == "MANUAL") {
    return ControlMode::Manual;
  }
  return ControlMode::Stop;
}

}  // namespace mtt_control
