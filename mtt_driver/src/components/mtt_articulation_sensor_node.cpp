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
  // ── Parameters ────────────────────────────────────────────────────────────
  serial_port_name_ = declare_parameter("serial_port", 
    std::string("/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF373146363143225155-if02"));
  baud_rate_ = declare_parameter("baud_rate", 921600);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 100.0);
  filter_window_size_ = declare_parameter("filter_window_size", 50);
  // Convention: positive angle = LEFT turn (CCW, REP-103), negative = RIGHT.
  // The raw LUT (bit_coords → angle_coords_deg) already follows this convention:
  //   low bits ≈ +48° (left), high bits ≈ -49° (right).
  // invert_sign should be false.  Only set true if the encoder is physically
  // mounted in the opposite orientation.
  invert_sign_ = declare_parameter("invert_sign", false);
  angle_offset_rad_ = declare_parameter("angle_offset_rad", 0.0);

  // Default coordinates from Badger driver
  std::vector<double> default_bit_coords = {
    82, 214, 313, 430, 510, 600, 723, 831, 929, 1028, 1128, 1201, 1306, 1410, 1529, 1628,
    1737, 1826, 1906, 2016, 2119, 2226, 2337, 2443, 2520, 2617, 2716, 2815, 2908, 3052, 3106, 3258, 3323, 3408
  };
  std::vector<double> default_angle_coords_deg = {
    48, 43, 40, 36, 35, 32, 29, 25, 23, 20, 18, 15, 12, 9, 5, 2, 0, -2, -4, -7, -8, -10, -14, -18, -20, -21, -28, -26, -30, -35, -40, -43, -45, -49
  };

  bit_coords_ = declare_parameter("bit_coords", default_bit_coords);
  angle_coords_deg_ = declare_parameter("angle_coords_deg", default_angle_coords_deg);

  // ── ROS ───────────────────────────────────────────────────────────────────
  angle_pub_ = create_publisher<std_msgs::msg::Float64>("/hardware/articulation_angle", 10);
  
  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&MttArticulationSensorNode::publish_timer_callback, this));

  // ── Serial Initialization ─────────────────────────────────────────────────
  try {
    io_context_ = std::make_unique<boost::asio::io_context>();
    serial_port_ = std::make_unique<boost::asio::serial_port>(*io_context_, serial_port_name_);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    
    running_ = true;
    read_thread_ = std::thread(&MttArticulationSensorNode::read_loop, this);
    
    RCLCPP_INFO(get_logger(), "Started articulation sensor node on %s at %d baud", 
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

void MttArticulationSensorNode::read_loop()
{
  uint8_t buffer[1];
  while (running_ && rclcpp::ok()) {
    try {
      boost::system::error_code ec;
      size_t len = serial_port_->read_some(boost::asio::buffer(buffer, 1), ec);
      if (!ec && len > 0) {
        process_byte(buffer[0]);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Serial read error: %s", e.what());
    }
  }
}

void MttArticulationSensorNode::process_byte(uint8_t byte)
{
  switch (parse_state_) {
    case 0: // Wait for Sync 0xAA
      if (byte == 0xAA) {
        parse_state_ = 1;
      }
      break;
    case 1: // Low Byte
      low_byte_ = byte;
      parse_state_ = 2;
      break;
    case 2: // High Nibble
      high_nibble_ = byte;
      parse_state_ = 3;
      break;
    case 3: // Checksum
      {
        uint8_t checksum = byte;
        if ((low_byte_ ^ high_nibble_) == checksum) {
          int val = low_byte_ | ((high_nibble_ & 0x0F) << 8);
          
          std::lock_guard<std::mutex> lock(data_mutex_);
          filter_buf_.push_back(val);
          while (filter_buf_.size() > static_cast<size_t>(filter_window_size_)) {
            filter_buf_.pop_front();
          }
          
          if (!filter_buf_.empty()) {
            double sum = std::accumulate(filter_buf_.begin(), filter_buf_.end(), 0.0);
            current_filtered_bits_ = sum / filter_buf_.size();
            
            double angle_deg = get_angle_from_bits(current_filtered_bits_);
            double angle_rad = angle_deg * (M_PI / 180.0);
            
            if (invert_sign_) {
              angle_rad = -angle_rad;
            }
            latest_angle_rad_ = angle_rad + angle_offset_rad_;
          }
        }
        parse_state_ = 0;
      }
      break;
  }
}

double MttArticulationSensorNode::get_angle_from_bits(double bits)
{
  if (bit_coords_.empty() || angle_coords_deg_.empty()) return 0.0;
  if (bits <= bit_coords_.front()) return angle_coords_deg_.front();
  if (bits >= bit_coords_.back()) return angle_coords_deg_.back();

  // Linear interpolation
  auto it = std::lower_bound(bit_coords_.begin(), bit_coords_.end(), bits);
  size_t i = std::distance(bit_coords_.begin(), it);
  
  if (i == 0) return angle_coords_deg_.front();
  
  double x0 = bit_coords_[i - 1];
  double x1 = bit_coords_[i];
  double y0 = angle_coords_deg_[i - 1];
  double y1 = angle_coords_deg_[i];
  
  return y0 + (bits - x0) * (y1 - y0) / (x1 - x0);
}

void MttArticulationSensorNode::publish_timer_callback()
{
  std_msgs::msg::Float64 msg;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    msg.data = latest_angle_rad_;
  }
  angle_pub_->publish(msg);
}

}  // namespace mtt

RCLCPP_COMPONENTS_REGISTER_NODE(mtt::MttArticulationSensorNode)
