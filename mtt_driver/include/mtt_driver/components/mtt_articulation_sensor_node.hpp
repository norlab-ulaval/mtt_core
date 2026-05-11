#ifndef MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SENSOR_NODE_HPP_
#define MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SENSOR_NODE_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <boost/asio.hpp>

namespace mtt
{

class MttArticulationSensorNode : public rclcpp::Node
{
public:
  explicit MttArticulationSensorNode(const rclcpp::NodeOptions & options);
  ~MttArticulationSensorNode();

private:
  void read_loop();
  void publish_timer_callback();
  double get_angle_from_bits(double bits);
  void process_byte(uint8_t byte);

  // Parameters
  std::string serial_port_name_;
  int baud_rate_;
  double publish_rate_hz_;
  int filter_window_size_;
  bool invert_sign_;
  double angle_offset_rad_;
  std::vector<double> bit_coords_;
  std::vector<double> angle_coords_deg_;

  // Serial communication
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};

  // Parsing state
  uint8_t parse_state_{0};
  uint8_t low_byte_{0};
  uint8_t high_nibble_{0};

  // DSP
  std::deque<int> filter_buf_;
  std::mutex data_mutex_;
  double current_filtered_bits_{0.0};
  double latest_angle_rad_{0.0};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mtt

#endif  // MTT_DRIVER__COMPONENTS__MTT_ARTICULATION_SENSOR_NODE_HPP_
