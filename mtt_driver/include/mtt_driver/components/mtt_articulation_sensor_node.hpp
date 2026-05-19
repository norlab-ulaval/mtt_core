#ifndef MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SENSOR_NODE_HPP_
#define MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SENSOR_NODE_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace mtt
{

/// Reads both articulation potentiometers from the STM32 over serial.
///
/// STM32 frame (6 bytes, 1 kHz):
///   [0xAA | adc1_low | adc1_high_nibble | adc2_low | adc2_high_nibble | checksum]
///   checksum = adc1_low ^ adc1_high_nibble ^ adc2_low ^ adc2_high_nibble
///
/// ADC1 (PA0, 8-bit)  → pitch (hitch longitudinal / tilt angle)
///   bits range: 0–255 — LUT optional (use pitch_bit_coords / pitch_angle_coords_deg)
///   Publishes: /hardware/articulation_pitch_bits (raw, always)
///              /hardware/articulation_pitch_rad   (calibrated, when LUT provided)
///
/// ADC2 (PA6, 12-bit) → yaw (primary articulation steering angle)
///   bits range: 0–4095 — LUT from Badger driver (34-point, −49° to +48°)
///   Publishes: /hardware/articulation_angle (radians, 100 Hz)
class MttArticulationSensorNode : public rclcpp::Node
{
public:
  explicit MttArticulationSensorNode(const rclcpp::NodeOptions & options);
  ~MttArticulationSensorNode();

private:
  void read_loop();
  void publish_timer_callback();
  double interpolate_lut(double bits,
    const std::vector<double> & bit_coords,
    const std::vector<double> & angle_coords_deg) const;
  void process_byte(uint8_t byte);

  // ── Parameters ─────────────────────────────────────────────────────────
  std::string serial_port_name_;
  int baud_rate_;
  double publish_rate_hz_;
  int filter_window_size_;

  // Yaw (ADC2)
  bool yaw_invert_sign_;
  double yaw_angle_offset_rad_;
  std::vector<double> yaw_bit_coords_;
  std::vector<double> yaw_angle_coords_deg_;

  // Pitch (ADC1) — two conversion modes (priority: LUT > linear > uncalibrated=0)
  //
  // Mode 1 — LUT (full calibration, like yaw):
  //   pitch_bit_coords + pitch_angle_coords_deg  (multi-point, non-linear)
  //
  // Mode 2 — linear fit (2-parameter, sufficient for a potentiometer):
  //   pitch_bits_zero:   ADC1 bits at α=0 (flat terrain, hitch at rest)
  //   pitch_deg_per_bit: degrees per ADC count  (sign = mounting direction)
  //   → pitch_deg = (bits - pitch_bits_zero) * pitch_deg_per_bit
  //
  // When neither is configured: pitch_rad=0 and pitch_lut_available_=false
  // (odometry will mark pitch_fresh=false → factor graph uses flat prior)
  bool pitch_invert_sign_;
  double pitch_angle_offset_rad_;
  std::vector<double> pitch_bit_coords_;
  std::vector<double> pitch_angle_coords_deg_;
  double pitch_bits_zero_;    // linear mode: ADC1 bits at α=0
  double pitch_deg_per_bit_;  // linear mode: deg/bit (0 = uncalibrated)

  // ── Serial ─────────────────────────────────────────────────────────────
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};

  // ── Frame parser state machine (6-byte frame) ──────────────────────────
  uint8_t parse_state_{0};
  uint8_t adc1_low_{0};
  uint8_t adc1_high_{0};
  uint8_t adc2_low_{0};
  uint8_t adc2_high_{0};

  // ── DSP ────────────────────────────────────────────────────────────────
  std::mutex data_mutex_;

  // Yaw
  std::deque<int> yaw_filter_buf_;
  double latest_yaw_rad_{0.0};

  // Pitch
  std::deque<int> pitch_filter_buf_;
  double latest_pitch_bits_{0.0};      // filtered raw bits (diagnostic)
  double latest_pitch_rad_{0.0};       // calibrated, or 0 if no LUT
  bool pitch_lut_available_{false};

  // ── ROS ────────────────────────────────────────────────────────────────
  // Yaw — primary articulation angle
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
  // Pitch — raw bits (always published, useful before calibration)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_bits_pub_;
  // Pitch — calibrated radians (only published when LUT configured)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_rad_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mtt

#endif  // MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SENSOR_NODE_HPP_
