// Sibling of mtt_ice_slip_detector_node.cpp's auto_com_switch mode: same
// toggle/publish/cooldown idiom on mtt_control/com_direction_sign, but
// triggered by a remote-controller button rising edge instead of slip.

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "mtt_control/common/joystick_state.hpp"

namespace mtt_control
{

class MttComButtonSwitchNode : public rclcpp::Node
{
public:
  explicit MttComButtonSwitchNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("mtt_com_button_switch_node", options)
  {
    joy_topic_ = declare_parameter("joy_topic", std::string("joy"));
    com_direction_topic_ =
      declare_parameter("com_direction_topic", std::string("mtt_control/com_direction_sign"));
    com_toggle_button_index_ = declare_parameter("com_toggle_button_index", -1);
    switch_cooldown_s_ = declare_parameter("switch_cooldown_s", 0.5);
    enabled_ = declare_parameter("enabled", true);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) { on_joy(msg); });
    com_direction_sub_ = create_subscription<std_msgs::msg::Float64>(
      com_direction_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { on_com_direction(msg); });

    com_direction_pub_ = create_publisher<std_msgs::msg::Float64>(com_direction_topic_, 10);
    enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/com_button_switch/enabled", 10);
    com_direction_state_pub_ =
      create_publisher<std_msgs::msg::Float64>("/com_button_switch/com_direction_sign", 10);

    RCLCPP_INFO(
      get_logger(),
      "COM button switch ready: joy=%s com_direction=%s button_index=%d cooldown=%.2fs "
      "enabled=%s",
      joy_topic_.c_str(), com_direction_topic_.c_str(), com_toggle_button_index_,
      switch_cooldown_s_, enabled_ ? "true" : "false");
  }

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    joystick_state_.update(*msg);

    publish_diagnostics();

    if (!enabled_ || com_toggle_button_index_ < 0) {
      return;
    }
    if (!joystick_state_.button_rising(static_cast<std::size_t>(com_toggle_button_index_))) {
      return;
    }

    const rclcpp::Time now_time = now();
    if (has_last_switch_time_) {
      const double elapsed_s = (now_time - last_switch_time_).seconds();
      if (elapsed_s < switch_cooldown_s_) {
        return;
      }
    }

    com_direction_sign_ = com_direction_sign_ < 0.0 ? 1.0 : -1.0;
    auto sign_msg = std_msgs::msg::Float64();
    sign_msg.data = com_direction_sign_;
    com_direction_pub_->publish(sign_msg);
    last_switch_time_ = now_time;
    has_last_switch_time_ = true;
    RCLCPP_WARN(
      get_logger(),
      "COM direction switch on button press: sign=%.0f", com_direction_sign_);
    publish_diagnostics();
  }

  void on_com_direction(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    com_direction_sign_ = msg->data < 0.0 ? -1.0 : 1.0;
  }

  void publish_diagnostics()
  {
    std_msgs::msg::Bool enabled_msg;
    enabled_msg.data = enabled_;
    enabled_pub_->publish(enabled_msg);

    std_msgs::msg::Float64 sign_msg;
    sign_msg.data = com_direction_sign_;
    com_direction_state_pub_->publish(sign_msg);
  }

  std::string joy_topic_;
  std::string com_direction_topic_;
  int com_toggle_button_index_{-1};
  double switch_cooldown_s_{0.5};
  bool enabled_{true};

  JoystickState joystick_state_;
  double com_direction_sign_{1.0};
  bool has_last_switch_time_{false};
  rclcpp::Time last_switch_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr com_direction_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_state_pub_;
};

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttComButtonSwitchNode>());
  rclcpp::shutdown();
  return 0;
}
