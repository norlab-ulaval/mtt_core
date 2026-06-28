#include "mtt_driver/components/mtt_articulation_sensor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

#include "rclcpp_components/register_node_macro.hpp"

namespace mtt
{

MttArticulationSensorNode::MttArticulationSensorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mtt_articulation_sensor_node", options)
{
  // ── Parameters ──
  serial_port_name_ = declare_parameter("serial_port",
    std::string("/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF373146363143224542-if02"));
  baud_rate_              = declare_parameter("baud_rate", 921600);
  publish_rate_hz_        = declare_parameter("publish_rate_hz", 100.0);
  data_timeout_s_         = declare_parameter("data_timeout_s", 0.25);
  filter_window_size_     = declare_parameter("filter_window_size", 50);

  // ── Yaw (ADC2, 12-bit, PA6) ──
  // Convention: positive = left (CCW, REP-103). LUT from Badger driver.
  // Low bits ≈ +48° (left), high bits ≈ -49° (right) — do NOT invert by default.
  yaw_invert_sign_        = declare_parameter("invert_sign", false);
  yaw_angle_offset_rad_   = declare_parameter("angle_offset_rad", 0.0);

  // 34-point calibration table — matches Badger steering_interface.py exactly.
  // bit_coords:       raw ADC2 values (0–4095 range, 12-bit)
  // angle_coords_deg: corresponding angle in degrees (0 = straight, + = left)
  yaw_bit_coords_ = declare_parameter("bit_coords", std::vector<double>{
    82,  214, 313, 430, 510, 600, 723, 831,  929,  1028, 1128, 1201,
    1306,1410,1529,1628,1737,1826,1906,2016, 2119, 2226, 2337, 2443,
    2520,2617,2716,2815,2908,3052,3106,3258, 3323, 3408
  });
  yaw_angle_coords_deg_ = declare_parameter("angle_coords_deg", std::vector<double>{
    48, 43, 40, 36, 35, 32, 29, 25, 23, 20, 18, 15,
    12,  9,  5,  2,  0, -2, -4, -7, -8,-10,-14,-18,
    -20,-21,-28,-26,-30,-35,-40,-43,-45,-49
  });

  // ── Pitch (ADC1, 8-bit, PA0) ──
  // Hitch longitudinal / tilt angle.
  // Two calibration modes (see header). Use whichever is available:
  //   Mode 1 — full LUT  : pitch_bit_coords + pitch_angle_coords_deg
  //   Mode 2 — linear fit: pitch_bits_zero + pitch_deg_per_bit
  pitch_invert_sign_      = declare_parameter("pitch_invert_sign",    false);
  pitch_angle_offset_rad_ = declare_parameter("pitch_angle_offset_rad", 0.0);
  pitch_bit_coords_       = declare_parameter("pitch_bit_coords",    std::vector<double>{});
  pitch_angle_coords_deg_ = declare_parameter("pitch_angle_coords_deg", std::vector<double>{});
  pitch_bits_zero_        = declare_parameter("pitch_bits_zero",    128.0); // ADC1 bits at α=0
  pitch_deg_per_bit_      = declare_parameter("pitch_deg_per_bit",    0.0); // deg/bit (0 = uncalibrated)

  reconnect_delay_s_ = declare_parameter("reconnect_delay_s", 1.0);

  const bool pitch_lut_full =
    !pitch_bit_coords_.empty() && !pitch_angle_coords_deg_.empty() &&
    pitch_bit_coords_.size() == pitch_angle_coords_deg_.size();
  const bool pitch_linear = (pitch_deg_per_bit_ != 0.0);

  pitch_lut_available_ = pitch_lut_full || pitch_linear;

  // ── Publishers ──
  // Yaw — primary articulation steering angle (same topic as before)
  yaw_pub_ = create_publisher<std_msgs::msg::Float64>("/hardware/articulation_angle", 10);

  // Pitch — raw ADC bits (always, useful for offline LUT calibration)
  pitch_bits_pub_ = create_publisher<std_msgs::msg::Float64>("/hardware/articulation_pitch_bits", 10);

  // Pitch — calibrated radians (ALWAYS published; 0.0 until calibration params are set)
  pitch_rad_pub_ = create_publisher<std_msgs::msg::Float64>("/hardware/articulation_pitch_rad", 10);

  if (pitch_lut_full) {
    RCLCPP_INFO(get_logger(), "Pitch: LUT mode (%zu points)", pitch_bit_coords_.size());
  } else if (pitch_linear) {
    RCLCPP_INFO(get_logger(),
      "Pitch: linear mode  bits_zero=%.1f  deg/bit=%.4f  offset=%.4f rad",
      pitch_bits_zero_, pitch_deg_per_bit_, pitch_angle_offset_rad_);
  } else {
    RCLCPP_WARN(get_logger(),
      "Pitch: UNCALIBRATED — /hardware/articulation_pitch_rad=0.0 until "
      "pitch_bits_zero + pitch_deg_per_bit (or full LUT) are configured. "
      "Set in mtt_articulation_sensor_node YAML.");
  }

  // ── Timer ──
  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&MttArticulationSensorNode::publish_timer_callback, this));

  // ── Serial ──
  try {
    io_context_  = std::make_unique<boost::asio::io_context>();
    serial_port_ = std::make_unique<boost::asio::serial_port>(*io_context_, serial_port_name_);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    running_     = true;
    read_thread_ = std::thread(&MttArticulationSensorNode::read_loop, this);
    RCLCPP_INFO(get_logger(), "Articulation sensor on %s @ %d baud",
      serial_port_name_.c_str(), baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port %s: %s",
      serial_port_name_.c_str(), e.what());
  }
}

MttArticulationSensorNode::~MttArticulationSensorNode()
{
  running_ = false;
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

// ── Serial reader ──

void MttArticulationSensorNode::read_loop()
{
  uint8_t buf[64];  // block read: much more efficient than 1 byte at a time
  while (running_ && rclcpp::ok()) {
    // If port not open (init failure or disconnect): try to reconnect
    if (!serial_port_ || !serial_port_->is_open()) {
      close_and_reopen_serial();
      std::this_thread::sleep_for(
        std::chrono::duration<double>(reconnect_delay_s_));
      continue;
    }
    try {
      boost::system::error_code ec;
      const size_t len = serial_port_->read_some(
        boost::asio::buffer(buf, sizeof(buf)), ec);
      if (!ec && len > 0) {
        for (size_t i = 0; i < len; ++i) {
          process_byte(buf[i]);
        }
      } else if (ec) {
        // Serial error (USB CDC reset, cable pull, STM32 brownout...)
        RCLCPP_WARN(get_logger(),
          "Serial error: %s — closing and will reconnect in %.1fs",
          ec.message().c_str(), reconnect_delay_s_);
        close_and_reopen_serial();
        std::this_thread::sleep_for(
          std::chrono::duration<double>(reconnect_delay_s_));
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "Serial exception: %s — closing and will reconnect in %.1fs",
        e.what(), reconnect_delay_s_);
      close_and_reopen_serial();
      std::this_thread::sleep_for(
        std::chrono::duration<double>(reconnect_delay_s_));
    }
  }
}
// ── Serial auto-reconnect ──
//
// Called on any serial error or when the port is found closed.
// Closes the current port (if open), increments reconnect counter,
// and tries to reopen on the same device path.
// The caller is responsible for sleeping reconnect_delay_s_ before retrying.
void MttArticulationSensorNode::close_and_reopen_serial()
{
  // Close whatever we have
  try {
    if (serial_port_ && serial_port_->is_open()) {
      serial_port_->close();
    }
  } catch (...) {}
  serial_port_.reset();

  ++reconnect_count_;
  RCLCPP_WARN(get_logger(),
    "Articulation serial reconnect #%d on %s @ %d baud —"
    " likely STM32 USB-CDC reset (thermal/power event).",
    reconnect_count_, serial_port_name_.c_str(), baud_rate_);

  try {
    // io_context stays alive — just create a new serial_port on it
    serial_port_ = std::make_unique<boost::asio::serial_port>(
      *io_context_, serial_port_name_);
    serial_port_->set_option(
      boost::asio::serial_port_base::baud_rate(baud_rate_));
    RCLCPP_INFO(get_logger(),
      "Articulation serial reconnected (attempt #%d) on %s @ %d baud",
      reconnect_count_, serial_port_name_.c_str(), baud_rate_);
    // Reset frame parser so stale partial frame from before disconnect is discarded
    parse_state_ = 0;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Reconnect #%d failed: %s — will retry in %.1fs",
      reconnect_count_, e.what(), reconnect_delay_s_);
    serial_port_.reset();
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    has_valid_frame_ = false;
    yaw_filter_buf_.clear();
    pitch_filter_buf_.clear();
  }
}


//
// STM32 Create_Tx_buffer layout:
//   byte[0] = 0xAA          (sync)
//   byte[1] = adc1 & 0xFF   (pitch low)
//   byte[2] = adc1 >> 8     (pitch high nibble, 0 for 8-bit ADC)
//   byte[3] = adc2 & 0xFF   (yaw low)
//   byte[4] = adc2 >> 8     (yaw high nibble)
//   byte[5] = b[1]^b[2]^b[3]^b[4]  (checksum = XOR of all 4 data bytes)
//
void MttArticulationSensorNode::process_byte(uint8_t byte)
{
  switch (parse_state_) {
    case 0:  // Wait for sync
      if (byte == 0xAA) { parse_state_ = 1; }
      break;
    case 1:  // adc1 low byte (pitch)
      adc1_low_   = byte;
      parse_state_ = 2;
      break;
    case 2:  // adc1 high nibble (pitch — always 0 for 8-bit ADC)
      adc1_high_  = byte;
      parse_state_ = 3;
      break;
    case 3:  // adc2 low byte (yaw)
      adc2_low_   = byte;
      parse_state_ = 4;
      break;
    case 4:  // adc2 high nibble (yaw)
      adc2_high_  = byte;
      parse_state_ = 5;
      break;
    case 5:  // Checksum — XOR of all 4 data bytes
    {
      const uint8_t expected = adc1_low_ ^ adc1_high_ ^ adc2_low_ ^ adc2_high_;
      if (byte == expected) {
        // Reconstruct 12-bit values
        const int pitch_bits = adc1_low_ | ((adc1_high_ & 0x0F) << 8);  // 0–255 (8-bit ADC)
        const int yaw_bits   = adc2_low_ | ((adc2_high_ & 0x0F) << 8);  // 0–4095 (12-bit ADC)

        std::lock_guard<std::mutex> lock(data_mutex_);

        // ── Yaw (ADC2) ──
        yaw_filter_buf_.push_back(yaw_bits);
        while (static_cast<int>(yaw_filter_buf_.size()) > filter_window_size_) {
          yaw_filter_buf_.pop_front();
        }
        if (!yaw_filter_buf_.empty()) {
          const double filtered = std::accumulate(
            yaw_filter_buf_.begin(), yaw_filter_buf_.end(), 0.0) / yaw_filter_buf_.size();
          double angle_rad = interpolate_lut(filtered, yaw_bit_coords_, yaw_angle_coords_deg_)
                             * (M_PI / 180.0);
          if (yaw_invert_sign_) { angle_rad = -angle_rad; }
          latest_yaw_rad_ = angle_rad + yaw_angle_offset_rad_;
        }

        // ── Pitch (ADC1) ──
        pitch_filter_buf_.push_back(pitch_bits);
        while (static_cast<int>(pitch_filter_buf_.size()) > filter_window_size_) {
          pitch_filter_buf_.pop_front();
        }
        if (!pitch_filter_buf_.empty()) {
          latest_pitch_bits_ = std::accumulate(
            pitch_filter_buf_.begin(), pitch_filter_buf_.end(), 0.0) / pitch_filter_buf_.size();

          // Conversion: LUT (priority) → linear model → 0.0 (uncalibrated)
          const bool pitch_lut_full =
            !pitch_bit_coords_.empty() && !pitch_angle_coords_deg_.empty() &&
            pitch_bit_coords_.size() == pitch_angle_coords_deg_.size();

          double pitch_rad = 0.0;
          if (pitch_lut_full) {
            // Mode 1: multi-point LUT (same as yaw)
            pitch_rad = interpolate_lut(
              latest_pitch_bits_, pitch_bit_coords_, pitch_angle_coords_deg_)
              * (M_PI / 180.0);
          } else if (pitch_deg_per_bit_ != 0.0) {
            // Mode 2: linear fit  α = (bits - bits_zero) * deg/bit * π/180
            pitch_rad = (latest_pitch_bits_ - pitch_bits_zero_) * pitch_deg_per_bit_
                        * (M_PI / 180.0);
          }
          if (pitch_invert_sign_) { pitch_rad = -pitch_rad; }
          latest_pitch_rad_ = pitch_rad + pitch_angle_offset_rad_;
        }

        has_valid_frame_ = true;
        last_valid_frame_time_ = std::chrono::steady_clock::now();
      }
      // Back to sync hunt regardless of checksum result
      parse_state_ = 0;
      break;
    }
    default:
      parse_state_ = 0;
      break;
  }
}

// ── LUT interpolation (shared by yaw and pitch) ──

double MttArticulationSensorNode::interpolate_lut(
  double bits,
  const std::vector<double> & bit_coords,
  const std::vector<double> & angle_coords_deg) const
{
  if (bit_coords.empty()) { return 0.0; }
  if (bits <= bit_coords.front()) { return angle_coords_deg.front(); }
  if (bits >= bit_coords.back())  { return angle_coords_deg.back(); }

  auto it = std::lower_bound(bit_coords.begin(), bit_coords.end(), bits);
  const size_t i = std::distance(bit_coords.begin(), it);

  const double x0 = bit_coords[i - 1],  x1 = bit_coords[i];
  const double y0 = angle_coords_deg[i - 1], y1 = angle_coords_deg[i];
  return y0 + (bits - x0) * (y1 - y0) / (x1 - x0);
}

// ── Publish timer ──

void MttArticulationSensorNode::publish_timer_callback()
{
  std_msgs::msg::Float64 yaw_msg, pitch_bits_msg, pitch_rad_msg;
  bool data_fresh = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_fresh = has_valid_frame_ &&
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_valid_frame_time_).count() <= data_timeout_s_;
    if (data_fresh) {
      yaw_msg.data        = latest_yaw_rad_;
      pitch_bits_msg.data = latest_pitch_bits_;
      pitch_rad_msg.data  = latest_pitch_rad_;
    }
  }

  if (!data_fresh) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No valid STM32 articulation frame within %.2fs; suppressing stale feedback",
      data_timeout_s_);
    return;
  }

  yaw_pub_->publish(yaw_msg);
  pitch_bits_pub_->publish(pitch_bits_msg);
  pitch_rad_pub_->publish(pitch_rad_msg);  // always published (0.0 until calibrated)
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttArticulationSensorNode)
