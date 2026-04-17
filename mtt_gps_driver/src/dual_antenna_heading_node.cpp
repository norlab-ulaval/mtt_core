// Dual antenna heading computation node.
// Subscribes to two NavSatFix topics (left and right antenna),
// computes the heading vector between them, and publishes as QuaternionStamped.
// The heading is the perpendicular to the baseline (i.e., the forward direction).

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

// WGS84 constants for local tangent plane conversion
static constexpr double kEarthRadiusM = 6378137.0;
static constexpr double kDegToRad = M_PI / 180.0;

class DualAntennaHeadingNode : public rclcpp::Node {
public:
  DualAntennaHeadingNode() : Node("dual_antenna_heading_node") {
    declare_parameter("baseline_m", 1.1);         // Expected baseline distance
    declare_parameter("max_age_s", 0.5);           // Max time delta between fixes
    declare_parameter("min_fix_quality", 1);        // Minimum quality (1=GPS, 4=RTK)
    declare_parameter("output_frame_id", "base_link");
    // Antenna orientation: left antenna is at +Y in robot frame,
    // so the forward direction (X) is perpendicular to left→right baseline.
    // heading = atan2(east_left - east_right, north_left - north_right) - pi/2

    baseline_m_ = get_parameter("baseline_m").as_double();
    max_age_s_ = get_parameter("max_age_s").as_double();
    min_fix_quality_ = get_parameter("min_fix_quality").as_int();
    output_frame_ = get_parameter("output_frame_id").as_string();

    heading_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
        "gps/heading", 10);
    // Also publish as IMU message for easy fusion with robot_localization
    heading_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "gps/heading_imu", 10);

    left_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps_left/fix", 10,
        [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          left_fix_ = *msg;
          try_compute_heading();
        });

    right_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps_right/fix", 10,
        [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          right_fix_ = *msg;
          try_compute_heading();
        });

    RCLCPP_INFO(get_logger(),
                "Dual antenna heading node started (baseline=%.2fm)", baseline_m_);
  }

private:
  void try_compute_heading() {
    // Need both fixes
    if (left_fix_.status.status < min_fix_quality_ - 2 ||
        right_fix_.status.status < min_fix_quality_ - 2) {
      return;  // Not enough fix quality
    }

    // Check time delta
    double t_left = rclcpp::Time(left_fix_.header.stamp).seconds();
    double t_right = rclcpp::Time(right_fix_.header.stamp).seconds();
    if (std::abs(t_left - t_right) > max_age_s_) return;

    // Convert to local tangent plane (meters)
    double lat_ref = (left_fix_.latitude + right_fix_.latitude) * 0.5 * kDegToRad;
    double m_per_deg_lat = kEarthRadiusM * kDegToRad;
    double m_per_deg_lon = kEarthRadiusM * kDegToRad * std::cos(lat_ref);

    double dn = (left_fix_.latitude - right_fix_.latitude) * m_per_deg_lat;
    double de = (left_fix_.longitude - right_fix_.longitude) * m_per_deg_lon;
    double measured_baseline = std::sqrt(dn * dn + de * de);

    // Sanity check baseline (should be close to expected)
    if (measured_baseline < baseline_m_ * 0.5 ||
        measured_baseline > baseline_m_ * 2.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Baseline mismatch: expected %.2fm, got %.2fm", baseline_m_, measured_baseline);
      return;
    }

    // Heading: baseline vector is left→right (roughly robot's -Y to +Y direction)
    // The angle of the baseline vector in ENU is:
    double baseline_angle = std::atan2(de, dn);
    // The forward direction (robot X) is perpendicular to L→R baseline.
    // If left is at +Y (port side), then forward = baseline_angle + pi/2
    double heading_enu = baseline_angle + M_PI / 2.0;

    // Normalize to [0, 2pi)
    if (heading_enu < 0) heading_enu += 2.0 * M_PI;
    if (heading_enu >= 2.0 * M_PI) heading_enu -= 2.0 * M_PI;

    // Publish as QuaternionStamped
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading_enu);  // ENU heading → quaternion

    auto stamp = now();

    geometry_msgs::msg::QuaternionStamped qmsg;
    qmsg.header.stamp = stamp;
    qmsg.header.frame_id = output_frame_;
    qmsg.quaternion.x = q.x();
    qmsg.quaternion.y = q.y();
    qmsg.quaternion.z = q.z();
    qmsg.quaternion.w = q.w();
    heading_pub_->publish(qmsg);

    // Publish as IMU (for robot_localization compatibility)
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = output_frame_;
    imu_msg.orientation = qmsg.quaternion;
    // Covariance: heading only, pitch/roll unknown from GPS
    double heading_var = (0.5 * M_PI / 180.0);  // ~0.5° for 1m baseline RTK
    heading_var *= heading_var;
    imu_msg.orientation_covariance = {
      999.0, 0.0, 0.0,       // roll unknown
      0.0, 999.0, 0.0,       // pitch unknown
      0.0, 0.0, heading_var  // yaw from dual antenna
    };
    // Mark angular velocity and linear acceleration as unused
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;
    heading_imu_pub_->publish(imu_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr heading_imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr right_sub_;

  sensor_msgs::msg::NavSatFix left_fix_;
  sensor_msgs::msg::NavSatFix right_fix_;

  double baseline_m_;
  double max_age_s_;
  int min_fix_quality_;
  std::string output_frame_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualAntennaHeadingNode>());
  rclcpp::shutdown();
  return 0;
}
