// Emlid Reach RS TCP client node.
// Connects to a Reach RS receiver via TCP, reads NMEA stream,
// and publishes sensor_msgs/NavSatFix + nmea_msgs/Sentence.

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "mtt_gps_driver/nmea_parser.hpp"

using namespace std::chrono_literals;

class ReachTcpNode : public rclcpp::Node {
public:
  ReachTcpNode() : Node("reach_tcp_node") {
    declare_parameter("host", "192.168.2.10");
    declare_parameter("port", 5001);
    declare_parameter("frame_id", "gps_left_link");
    declare_parameter("reconnect_interval_s", 3.0);

    host_ = get_parameter("host").as_string();
    port_ = get_parameter("port").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    reconnect_s_ = get_parameter("reconnect_interval_s").as_double();

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    nmea_pub_ = create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 10);

    // Run TCP reader in a separate thread
    reader_thread_ = std::thread(&ReachTcpNode::tcp_loop, this);

    RCLCPP_INFO(get_logger(), "Reach TCP node started: %s:%d → frame=%s",
                host_.c_str(), port_, frame_id_.c_str());
  }

  ~ReachTcpNode() override {
    running_ = false;
    if (reader_thread_.joinable()) reader_thread_.join();
    if (sock_fd_ >= 0) ::close(sock_fd_);
  }

private:
  void tcp_loop() {
    while (running_ && rclcpp::ok()) {
      if (!connect_to_reach()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Cannot connect to %s:%d, retrying...", host_.c_str(), port_);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(reconnect_s_ * 1000)));
        continue;
      }

      RCLCPP_INFO(get_logger(), "Connected to Reach RS at %s:%d", host_.c_str(), port_);
      read_stream();
      ::close(sock_fd_);
      sock_fd_ = -1;
    }
  }

  bool connect_to_reach() {
    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) return false;

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }

    if (::connect(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    return true;
  }

  void read_stream() {
    std::string buffer;
    char chunk[1024];

    while (running_ && rclcpp::ok()) {
      ssize_t n = ::recv(sock_fd_, chunk, sizeof(chunk) - 1, 0);
      if (n <= 0) {
        RCLCPP_WARN(get_logger(), "Connection lost to %s:%d", host_.c_str(), port_);
        break;
      }
      chunk[n] = '\0';
      buffer.append(chunk, static_cast<size_t>(n));

      // Process complete lines
      size_t pos;
      while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);

        // Strip trailing \r
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] != '$') continue;

        process_sentence(line);
      }
    }
  }

  void process_sentence(const std::string& sentence) {
    auto stamp = now();

    // Always publish raw NMEA
    nmea_msgs::msg::Sentence nmea_msg;
    nmea_msg.header.stamp = stamp;
    nmea_msg.header.frame_id = frame_id_;
    nmea_msg.sentence = sentence;
    nmea_pub_->publish(nmea_msg);

    // Parse GGA for NavSatFix
    if (sentence.find("GGA") != std::string::npos) {
      auto gga = mtt_gps::parse_gga(sentence);
      if (!gga) return;

      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = stamp;
      fix.header.frame_id = frame_id_;
      fix.latitude = gga->latitude_deg;
      fix.longitude = gga->longitude_deg;
      fix.altitude = gga->altitude_m;

      // Map fix quality to ROS status
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      switch (gga->fix_quality) {
        case 0:
          fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
          break;
        case 1: case 2:
          fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
          break;
        case 4: case 5:
          fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
          break;
        default:
          fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      }

      // Covariance from HDOP (approximate)
      double h_cov = gga->hdop * gga->hdop;
      double v_cov = h_cov * 4.0;  // Vertical is typically 2x worse
      // RTK Fix: much tighter covariance
      if (gga->fix_quality == 4) {
        h_cov = 0.01 * 0.01;  // 1 cm
        v_cov = 0.02 * 0.02;  // 2 cm
      } else if (gga->fix_quality == 5) {
        h_cov = 0.1 * 0.1;    // 10 cm float
        v_cov = 0.2 * 0.2;
      }
      fix.position_covariance = {
        h_cov, 0.0, 0.0,
        0.0, h_cov, 0.0,
        0.0, 0.0, v_cov
      };
      fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

      fix_pub_->publish(fix);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_pub_;

  std::string host_;
  int port_;
  std::string frame_id_;
  double reconnect_s_;
  int sock_fd_{-1};
  std::atomic<bool> running_{true};
  std::thread reader_thread_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReachTcpNode>());
  rclcpp::shutdown();
  return 0;
}
