// MTT-154 CAN Frame Codec
// Encodes the 8-byte command frame (ID 0x001) and decodes the 8-byte telemetry frame (ID 0x2FF).
// All CAN protocol knowledge is centralized here. If firmware changes a bit position, fix it here.
// Ported from the manual bit operations in mtt_driver.py.

#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "mtt_driver/logic/tachometer.hpp"
#include "mtt_driver/logic/vehicle_params.hpp"

namespace mtt::can {

// ── CAN IDs ──
constexpr uint32_t kCommandId    = 0x001;
constexpr uint32_t kExternalCommandId = 0x100;
constexpr uint32_t kTelemetryId  = 0x2FF;
constexpr uint32_t kMainControllerVersionId   = 0x300;
constexpr uint32_t kBatteryControllerVersionId = 0x301;

// ── Byte positions in the 8-byte command frame ──
enum FrameIndex : uint8_t {
  kVehicleType     = 0,
  kGlobalSwitches  = 1,
  kThrottle        = 2,
  kWinch           = 3,
  kBrake           = 4,
  kSteer           = 5,
  kDirectionMode   = 6,
  kReserved        = 7,
};

// ── Enumerations (match firmware protocol) ──
enum class Direction : uint8_t { Forward = 0x00, Reverse = 0x01 };
enum class SteeringMode : uint8_t { OpenLoop = 0, CloseLoop = 1 };
enum class VehicleType : uint8_t { SingleTrack = 0x00, SbsLeft = 0x01, SbsRight = 0x02 };
enum class WinchState : uint8_t { Neutral = 0x7F, In = 0xE5, Out = 0x18 };
enum class LightState : uint8_t { Off = 0x00, On = 0x01 };
enum class SafetyState : uint8_t { Locked = 0x00, Unlocked = 0x01 };

// ── Command Frame ──
// Mutable representation of the 8-byte outgoing CAN frame.
struct CommandFrame {
  std::array<uint8_t, 8> data{};

  // Vehicle type (byte 0)
  void set_vehicle_type(VehicleType type) { data[kVehicleType] = static_cast<uint8_t>(type); }

  // Safety switch — Bit 7 (0x80) per CAN spec v1.1 DBC + Bit 3 (0x08) for firmware compat.
  // Both bits are set/cleared together: DBC says Bit 7, firmware may still check Bit 3.
  // 1 = Unlocked (motion allowed), 0 = Locked.
  void set_safety(SafetyState state) {
    if (state == SafetyState::Unlocked)
      data[kGlobalSwitches] |= 0b1000'1000;   // Bit 7 (DBC) | Bit 3 (firmware compat)
    else
      data[kGlobalSwitches] &= 0b0111'0111;   // clear both
  }

  // Direction — Bit 5 (0x20) per CAN spec v1.1.
  // Standardised mapping:
  //   Bit 5 = 1 → Forward
  //   Bit 5 = 0 → Reverse
  void set_direction(Direction dir) {
    if (dir == Direction::Forward)
      data[kGlobalSwitches] |= 0b0010'0000;  // set bit 5 = Forward
    else
      data[kGlobalSwitches] &= 0b1101'1111;  // clear bit 5 = Reverse
  }

  Direction get_direction() const {
    return (data[kGlobalSwitches] & 0b0010'0000) ? Direction::Forward : Direction::Reverse;
  }

  VehicleType get_vehicle_type() const {
    return static_cast<VehicleType>(data[kVehicleType]);
  }

  SafetyState get_safety() const {
    return (data[kGlobalSwitches] & 0b1000'0000)
      ? SafetyState::Unlocked
      : SafetyState::Locked;
  }

  bool light_off_estop_patch() const {
    return (data[kGlobalSwitches] & 0b0100'0000) != 0;
  }

  WinchState get_winch() const {
    return static_cast<WinchState>(data[kWinch]);
  }

  // Light (bit 6 of byte 1)
  void set_light(LightState state) {
    if (state == LightState::Off)
      data[kGlobalSwitches] |= 0b0100'0000;   // inverted logic per firmware
    else
      data[kGlobalSwitches] &= 0b1011'1111;
  }

  // Throttle (byte 2, 0..230)
  bool set_throttle_raw(uint8_t value) {
    if (value > VehicleParams::throttle_max) return false;
    data[kThrottle] = value;
    return true;
  }

  // Throttle from normalized 0..1
  bool set_throttle(double percent) {
    if (percent < 0.0 || percent > 1.0) return false;
    auto raw = static_cast<uint8_t>(percent * VehicleParams::throttle_max + 0.5);
    data[kThrottle] = raw;
    return true;
  }

  // Winch (byte 3)
  void set_winch(WinchState state) { data[kWinch] = static_cast<uint8_t>(state); }

  // Brake (byte 4, 0..255)
  bool set_brake_raw(uint8_t value) {
    data[kBrake] = value;
    return true;
  }

  bool set_brake(double percent) {
    if (percent < 0.0 || percent > 1.0) return false;
    data[kBrake] = static_cast<uint8_t>(percent * VehicleParams::brake_max + 0.5);
    return true;
  }

  // Steer (byte 5, 0..255, center = 127)
  bool set_steer_raw(uint8_t value) {
    data[kSteer] = value;
    return true;
  }

  // Steer from normalized -1..+1
  bool set_steer(double normalized) {
    if (normalized < -1.0 || normalized > 1.0) return false;
    if (std::abs(normalized) < VehicleParams::steering_deadband_normalized) {
      data[kSteer] = VehicleParams::steering_center_byte;
    } else {
      auto raw = static_cast<uint8_t>((normalized + 1.0) * 0.5 * VehicleParams::steering_max_byte + 0.5);
      data[kSteer] = raw;
    }
    return true;
  }

  uint8_t steer_raw()    const { return data[kSteer]; }
  uint8_t throttle_raw() const { return data[kThrottle]; }
  uint8_t brake_raw()    const { return data[kBrake]; }
  uint8_t reserved_raw() const { return data[kReserved]; }

  // Steering mode (bit 0 of byte 6)
  void set_steering_mode(SteeringMode mode) {
    if (mode == SteeringMode::CloseLoop)
      data[kDirectionMode] |= 0b0000'0001;
    else
      data[kDirectionMode] &= 0b1111'1110;
  }

  SteeringMode get_steering_mode() const {
    return (data[kDirectionMode] & 0b0000'0001)
      ? SteeringMode::CloseLoop
      : SteeringMode::OpenLoop;
  }

  // Initialize to safe defaults (matches _setup_initial_frame in Python)
  void init_defaults() {
    data.fill(0);
    set_vehicle_type(VehicleType::SingleTrack);
    set_safety(SafetyState::Unlocked);
    set_direction(Direction::Forward);
    set_light(LightState::Off);
    set_winch(WinchState::Neutral);
    set_steer_raw(VehicleParams::steering_center_byte);
    set_steering_mode(SteeringMode::CloseLoop);
  }
};

inline const char* vehicle_type_to_string(VehicleType type)
{
  switch (type) {
    case VehicleType::SingleTrack:
      return "SingleTrack";
    case VehicleType::SbsLeft:
      return "SideBySideLeft";
    case VehicleType::SbsRight:
      return "SideBySideRight";
  }
  return "Unknown";
}

inline const char* winch_state_to_string(WinchState state)
{
  switch (state) {
    case WinchState::Neutral:
      return "Neutral";
    case WinchState::In:
      return "In";
    case WinchState::Out:
      return "Out";
  }
  return "Unknown";
}

// ── Telemetry Frame Decoder ──
// Decodes the 8-byte incoming CAN frame from ID 0x2FF.
struct TelemetryDecoder {
  // Decode raw 8-byte frame into tachometer reading.
  // Layout per firmware:
  //   byte[0]   : int8_t  temperature_a
  //   byte[1]   : int8_t  temperature_b
  //   byte[2:4] : uint16_t (big-endian) instantaneous RPS
  //   byte[4:8] : uint32_t (big-endian) cumulative ticks
  static std::optional<TachometerReading> decode(const uint8_t* data, size_t len) {
    if (len < 8) return std::nullopt;

    TachometerReading r;
    r.temperature_a = static_cast<int8_t>(data[0]);
    r.temperature_b = static_cast<int8_t>(data[1]);
    r.instant_rps   = static_cast<uint16_t>((data[2] << 8) | data[3]);
    r.cumulative_ticks = static_cast<uint32_t>(
        (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]);

    return r;
  }
};

// ── BMS CAN IDs ──
// ROYPOW S51105 BMS broadcasts on 4 frames.
constexpr uint32_t kBmsCellTempsId  = 0x600;  // CellTemp1..4
constexpr uint32_t kBmsSysTempsId   = 0x601;  // AmbientTemp, MosfetTemp, HeatpadA/B
constexpr uint32_t kBmsCoreId       = 0x602;  // SOC, current, voltage, heatpad state
constexpr uint32_t kBmsDateTimeId   = 0x603;  // Timestamp / extra (not decoded here)
constexpr uint32_t kChargerCommandId = 0x1806E5F4;
constexpr uint32_t kChargerStatusId  = 0x18FF50E5;

inline const char* frame_name_from_id(uint32_t id)
{
  switch (id) {
    case kCommandId:
      return "MTT_Control_Joystick_001";
    case kExternalCommandId:
      return "MTT_Control_External_100";
    case kTelemetryId:
      return "MTT_Main_Status_2FF";
    case kMainControllerVersionId:
      return "MTT_Main_Controller_Version_300";
    case kBatteryControllerVersionId:
      return "MTT_Battery_Controller_Version_301";
    case kBmsCellTempsId:
      return "MTT_BMS_Cell_Temperatures_600";
    case kBmsSysTempsId:
      return "MTT_BMS_System_Temperatures_601";
    case kBmsCoreId:
      return "MTT_BMS_Core_Status_602";
    case kBmsDateTimeId:
      return "MTT_BMS_Date_Time_Remaining_603";
    case kChargerCommandId:
      return "MTT_Charger_Command_1806E5F4";
    case kChargerStatusId:
      return "MTT_Charger_Status_18FF50E5";
    default:
      return "Unknown";
  }
}

inline bool is_known_mtt_frame(uint32_t id)
{
  return id == kCommandId ||
         id == kExternalCommandId ||
         id == kTelemetryId ||
         id == kMainControllerVersionId ||
         id == kBatteryControllerVersionId ||
         id == kBmsCellTempsId ||
         id == kBmsSysTempsId ||
         id == kBmsCoreId ||
         id == kBmsDateTimeId ||
         id == kChargerCommandId ||
         id == kChargerStatusId;
}

struct ControllerVersionReading {
  uint32_t main_hardware_revision_raw{0};
  uint32_t main_software_revision_raw{0};
  uint32_t battery_hardware_revision_raw{0};
  uint32_t battery_software_revision_raw{0};
  bool has_main_controller_version{false};
  bool has_battery_controller_version{false};
};

// ── BMS Reading ──
// Accumulates data across the 3 useful BMS frames.
// NOTE: BatteryCurrent_raw and BatteryVoltage_raw are published as-is
// per the DBC (factor=1, offset=0, unit="raw") because physical scaling
// (e.g. 10mV/LSB for voltage) is not confirmed in the specification.
// Verify against a calibrated meter before trusting the computed power.
struct BmsReading {
  // ── 0x602 ──
  uint8_t  soc_percent{0};          // State of charge (0-100 %)
  int16_t  battery_current_raw{0};  // Signed int, unit not confirmed
  uint16_t battery_voltage_raw{0};  // Unsigned int, unit not confirmed
  uint16_t charge_time_min{0};      // Estimated time to full charge (min)
  bool heatpad_b_on{false};
  bool heatpad_a_on{false};
  uint8_t heatpads_reserved{0};

  // ── 0x600 ──
  int16_t cell_temp[4]{};           // 4 cell group temperatures (°C direct int)

  // ── 0x601 ──
  int16_t ambient_temp{0};          // Ambient temperature (°C)
  int16_t mosfet_temp{0};           // MOSFET temperature (°C)
  int16_t heatpad_a_temp{0};        // Heatpad A temperature (°C)
  int16_t heatpad_b_temp{0};        // Heatpad B temperature (°C)

  // ── 0x603 ──
  uint16_t charge_time_remaining_603_raw{0};
  uint16_t year_month_raw{0};
  uint16_t day_hour_raw{0};
  uint16_t minute_second_raw{0};

  // ── Charger 29-bit IDs ──
  uint16_t charger_max_voltage_raw{0};
  uint16_t charger_max_current_raw{0};
  uint16_t charger_configured_voltage_raw{0};
  uint16_t charger_configured_current_raw{0};

  // ── Freshness flags ──
  bool has_soc{false};          // set after first 0x602
  bool has_cell_temps{false};   // set after first 0x600
  bool has_sys_temps{false};    // set after first 0x601
  bool has_datetime{false};     // set after first 0x603
  bool has_charger_command{false};
  bool has_charger_status{false};
};

// ── BMS Decoder ──
struct BmsDecoder {
  // Decode one BMS frame and accumulate into `out`.
  // Returns true if the frame was recognized and decoded.
  // Layout from MTT_CAN_v1_1_simple.dbc.
  static bool decode(uint32_t id, const uint8_t* data, size_t len, BmsReading& out) {
    if (len < 8) return false;

    if (id == kBmsCellTempsId) {
      // 0x600: 4x int16 big-endian (Motorola MSB)
      out.cell_temp[0] = static_cast<int16_t>((data[0] << 8) | data[1]);
      out.cell_temp[1] = static_cast<int16_t>((data[2] << 8) | data[3]);
      out.cell_temp[2] = static_cast<int16_t>((data[4] << 8) | data[5]);
      out.cell_temp[3] = static_cast<int16_t>((data[6] << 8) | data[7]);
      out.has_cell_temps = true;
      return true;
    }

    if (id == kBmsSysTempsId) {
      // 0x601: 4x int16 big-endian
      out.ambient_temp  = static_cast<int16_t>((data[0] << 8) | data[1]);
      out.mosfet_temp   = static_cast<int16_t>((data[2] << 8) | data[3]);
      out.heatpad_a_temp = static_cast<int16_t>((data[4] << 8) | data[5]);
      out.heatpad_b_temp = static_cast<int16_t>((data[6] << 8) | data[7]);
      out.has_sys_temps = true;
      return true;
    }

    if (id == kBmsCoreId) {
      // 0x602: SOC (byte 0), current (bytes 1:3, int16 BE), voltage (bytes 3:5, uint16 BE),
      //        heatpad flags (byte 5), charge time remaining (bytes 6:8, uint16 BE)
      out.soc_percent         = data[0];
      out.battery_current_raw = static_cast<int16_t>((data[1] << 8) | data[2]);
      out.battery_voltage_raw = static_cast<uint16_t>((data[3] << 8) | data[4]);
      out.heatpad_b_on        = (data[5] & 0x01) != 0;
      out.heatpad_a_on        = (data[5] & 0x02) != 0;
      out.heatpads_reserved   = static_cast<uint8_t>((data[5] >> 2) & 0x3F);
      out.charge_time_min     = static_cast<uint16_t>((data[6] << 8) | data[7]);
      out.has_soc = true;
      return true;
    }

    if (id == kBmsDateTimeId) {
      out.charge_time_remaining_603_raw = static_cast<uint16_t>((data[0] << 8) | data[1]);
      out.year_month_raw                = static_cast<uint16_t>((data[2] << 8) | data[3]);
      out.day_hour_raw                  = static_cast<uint16_t>((data[4] << 8) | data[5]);
      out.minute_second_raw             = static_cast<uint16_t>((data[6] << 8) | data[7]);
      out.has_datetime = true;
      return true;
    }

    if (id == kChargerCommandId) {
      out.charger_max_voltage_raw = static_cast<uint16_t>((data[0] << 8) | data[1]);
      out.charger_max_current_raw = static_cast<uint16_t>((data[2] << 8) | data[3]);
      out.has_charger_command = true;
      return true;
    }

    if (id == kChargerStatusId) {
      out.charger_configured_voltage_raw = static_cast<uint16_t>((data[0] << 8) | data[1]);
      out.charger_configured_current_raw = static_cast<uint16_t>((data[2] << 8) | data[3]);
      out.has_charger_status = true;
      return true;
    }

    return false;
  }
};

struct ControllerVersionDecoder {
  static bool decode(uint32_t id, const uint8_t* data, size_t len, ControllerVersionReading& out) {
    if (len < 8) {
      return false;
    }

    const uint32_t hardware = static_cast<uint32_t>(
      (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    const uint32_t software = static_cast<uint32_t>(
      (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]);

    if (id == kMainControllerVersionId) {
      out.main_hardware_revision_raw = hardware;
      out.main_software_revision_raw = software;
      out.has_main_controller_version = true;
      return true;
    }

    if (id == kBatteryControllerVersionId) {
      out.battery_hardware_revision_raw = hardware;
      out.battery_software_revision_raw = software;
      out.has_battery_controller_version = true;
      return true;
    }

    return false;
  }
};

}  // namespace mtt::can
