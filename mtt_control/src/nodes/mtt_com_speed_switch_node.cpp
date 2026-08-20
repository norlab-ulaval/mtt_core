// Sibling of mtt_ice_slip_detector_node.cpp's auto_com_switch mode: same
// toggle/publish/cooldown idiom on mtt_control/com_direction_sign, but
// triggered by /sensor/speed crossing a threshold (accelerating past it or
// decelerating past it) instead of slip.

#include <algorithm>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

namespace mtt_control
{

class MttComSpeedSwitchNode : public rclcpp::Node
{
public:
  explicit MttComSpeedSwitchNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("mtt_com_speed_switch_node", options)
  {
    speed_topic_ = declare_parameter("speed_topic", std::string("/sensor/speed"));
    com_direction_topic_ =
      declare_parameter("com_direction_topic", std::string("mtt_control/com_direction_sign"));
    speed_threshold_ms_ = declare_parameter("speed_threshold_ms", 2.0);
    speed_hysteresis_ms_ = declare_parameter("speed_hysteresis_ms", 0.2);
    switch_cooldown_s_ = declare_parameter("switch_cooldown_s", 1.0);
    enabled_ = declare_parameter("enabled", true);

    // com_forward_sign: which direction_sign value corresponds to the COM position
    // that helps at HIGH speed (typically: mass shifted toward rear for acceleration).
    // +1.0 (default) or -1.0 depending on physical motor mounting side.
    // crossing ABOVE threshold → set com_direction_sign = +com_forward_sign
    // crossing BELOW threshold → set com_direction_sign = -com_forward_sign
    com_forward_sign_ = declare_parameter("com_forward_sign", 1.0);
    com_forward_sign_ = (com_forward_sign_ >= 0.0) ? 1.0 : -1.0;  // snap to ±1

    speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      speed_topic_, 20,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { on_speed(msg); });
    com_direction_sub_ = create_subscription<std_msgs::msg::Float64>(
      com_direction_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { on_com_direction(msg); });

    com_direction_pub_ = create_publisher<std_msgs::msg::Float64>(com_direction_topic_, 10);
    enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/com_speed_switch/enabled", 10);
    com_direction_state_pub_ =
      create_publisher<std_msgs::msg::Float64>("/com_speed_switch/com_direction_sign", 10);
    speed_state_pub_ = create_publisher<std_msgs::msg::Float64>("/com_speed_switch/speed_ms", 10);
    above_threshold_pub_ =
      create_publisher<std_msgs::msg::Bool>("/com_speed_switch/above_threshold", 10);

    RCLCPP_INFO(
      get_logger(),
      "COM speed switch ready: speed=%s com_direction=%s threshold=%.2fm/s "
      "hysteresis=%.2fm/s cooldown=%.2fs enabled=%s",
      speed_topic_.c_str(), com_direction_topic_.c_str(), speed_threshold_ms_,
      speed_hysteresis_ms_, switch_cooldown_s_, enabled_ ? "true" : "false");
  }

private:
  void on_speed(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite speed");
      return;
    }
    const double speed_ms = static_cast<double>(msg->data);

    if (!has_state_) {
      above_threshold_ = speed_ms >= speed_threshold_ms_;
      has_state_ = true;
      publish_diagnostics(speed_ms);
      return;
    }

    const double enter_high = speed_threshold_ms_ + speed_hysteresis_ms_;
    const double enter_low = speed_threshold_ms_ - speed_hysteresis_ms_;
    bool new_above = above_threshold_;
    if (above_threshold_ && speed_ms < enter_low) {
      new_above = false;
    } else if (!above_threshold_ && speed_ms > enter_high) {
      new_above = true;
    }

    if (new_above != above_threshold_) {
      try_switch(speed_ms, new_above);
    }
    above_threshold_ = new_above;
    publish_diagnostics(speed_ms);
  }

  // going_above: true  = speed just crossed ABOVE threshold (accelerating)
  //              false = speed just crossed BELOW threshold (decelerating)
  void try_switch(double speed_ms, bool going_above)
  {
    if (!enabled_) {
      return;
    }

    const rclcpp::Time now_time = now();
    if (has_last_switch_time_) {
      const double elapsed_s = (now_time - last_switch_time_).seconds();
      if (elapsed_s < switch_cooldown_s_) {
        return;
      }
    }

    // Direction-aware deterministic switch — replaces the old blind toggle.
    // High speed (going_above=true)  → COM in "forward/acceleration" position.
    // Low speed  (going_above=false) → COM in the opposite position.
    const double desired_sign = going_above ? com_forward_sign_ : -com_forward_sign_;

    if (std::abs(desired_sign - com_direction_sign_) < 0.5) {
      // Already at the desired sign for this speed regime. Nothing to do.
      return;
    }

    com_direction_sign_ = desired_sign;
    auto sign_msg = std_msgs::msg::Float64();
    sign_msg.data = com_direction_sign_;
    com_direction_pub_->publish(sign_msg);
    last_switch_time_ = now_time;
    has_last_switch_time_ = true;
    RCLCPP_WARN(
      get_logger(),
      "COM direction switch on speed threshold crossing: speed=%.2f m/s %s threshold → sign=%.0f [com_forward_sign=%.0f]",
      speed_ms, going_above ? "above" : "below", com_direction_sign_, com_forward_sign_);
  }

  void on_com_direction(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    com_direction_sign_ = msg->data < 0.0 ? -1.0 : 1.0;
  }

  void publish_diagnostics(double speed_ms)
  {
    std_msgs::msg::Bool enabled_msg;
    enabled_msg.data = enabled_;
    enabled_pub_->publish(enabled_msg);

    std_msgs::msg::Float64 sign_msg;
    sign_msg.data = com_direction_sign_;
    com_direction_state_pub_->publish(sign_msg);

    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = speed_ms;
    speed_state_pub_->publish(speed_msg);

    std_msgs::msg::Bool above_msg;
    above_msg.data = above_threshold_;
    above_threshold_pub_->publish(above_msg);
  }

  std::string speed_topic_;
  std::string com_direction_topic_;
  double speed_threshold_ms_{2.0};
  double speed_hysteresis_ms_{0.2};
  double switch_cooldown_s_{1.0};
  bool enabled_{true};
  // +1.0 or -1.0: which direction_sign value corresponds to high-speed / forward regime.
  // Set via 'com_forward_sign' parameter (runtime.env: COM_FORWARD_SIGN).
  double com_forward_sign_{1.0};

  bool has_state_{false};
  bool above_threshold_{false};
  double com_direction_sign_{1.0};
  bool has_last_switch_time_{false};
  rclcpp::Time last_switch_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr com_direction_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr com_direction_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr above_threshold_pub_;
};

}  // namespace mtt_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_control::MttComSpeedSwitchNode>());
  rclcpp::shutdown();
  return 0;
}
