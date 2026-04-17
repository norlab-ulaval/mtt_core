// Emlid Reach RS unified driver node.
// Supports both TCP (WiFi) and USB Serial connections via `connection_type` parameter.
// Reads NMEA stream, publishes NavSatFix, raw NMEA sentences, and TimeReference.

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

// --- TCP ---
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

// --- Serial ---
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "mtt_gps_driver/nmea_parser.hpp"

using namespace std::chrono_literals;

class ReachDriverNode : public rclcpp::Node {
public:
  ReachDriverNode() : Node("reach_driver_node") {
    // Common params
    declare_parameter("connection_type", "tcp");  // "tcp" or "serial"
    declare_parameter("frame_id", "gps_left_link");
    declare_parameter("reconnect_interval_s", 3.0);

    // TCP params
    declare_parameter("host", "192.168.2.10");
    declare_parameter("port", 5001);

    // Serial params
    declare_parameter("serial_port", "/dev/reach_left");
    declare_parameter("baud_rate", 115200);

    conn_type_   = get_parameter("connection_type").as_string();
    frame_id_    = get_parameter("frame_id").as_string();
    reconnect_s_ = get_parameter("reconnect_interval_s").as_double();
    host_        = get_parameter("host").as_string();
    port_        = get_parameter("port").as_int();
    serial_port_ = get_parameter("serial_port").as_string();
    baud_rate_   = get_parameter("baud_rate").as_int();

    fix_pub_     = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    nmea_pub_    = create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 50);
    timeref_pub_ = create_publisher<sensor_msgs::msg::TimeReference>("time_reference", 10);

    if (conn_type_ != "tcp" && conn_type_ != "serial") {
      RCLCPP_FATAL(get_logger(), "connection_type must be 'tcp' or 'serial', got: '%s'",
                   conn_type_.c_str());
      throw std::invalid_argument("Invalid connection_type: " + conn_type_);
    }

    reader_thread_ = std::thread(&ReachDriverNode::io_loop, this);

    RCLCPP_INFO(get_logger(), "Reach driver started [%s] frame=%s",
                conn_type_.c_str(), frame_id_.c_str());
    if (conn_type_ == "tcp") {
      RCLCPP_INFO(get_logger(), "  TCP: %s:%d", host_.c_str(), port_);
    } else {
      RCLCPP_INFO(get_logger(), "  Serial: %s @ %d baud", serial_port_.c_str(), baud_rate_);
    }
  }

  ~ReachDriverNode() override {
    running_ = false;
    if (reader_thread_.joinable()) reader_thread_.join();
    close_fd();
  }

private:
  // ─── Main I/O loop (reconnects on failure) ────────────────────────
  void io_loop() {
    while (running_ && rclcpp::ok()) {
      bool ok = (conn_type_ == "tcp") ? connect_tcp() : connect_serial();
      if (!ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Cannot connect [%s], retrying in %.1fs...",
                             conn_type_.c_str(), reconnect_s_);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(reconnect_s_ * 1000)));
        continue;
      }

      if (conn_type_ == "tcp") {
        RCLCPP_INFO(get_logger(), "Connected via TCP: %s:%d", host_.c_str(), port_);
      } else {
        RCLCPP_INFO(get_logger(), "Connected via serial: %s", serial_port_.c_str());
      }

      read_stream();
      close_fd();
    }
  }

  // ─── TCP connection ────────────────────────────────────────────────
  bool connect_tcp() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) return false;

    // 5-second receive timeout
    struct timeval tv{5, 0};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
      ::close(fd_); fd_ = -1;
      return false;
    }
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(fd_); fd_ = -1;
      return false;
    }
    return true;
  }

  // ─── Serial connection (POSIX termios) ────────────────────────────
  bool connect_serial() {
    fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    // Set to blocking
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      ::close(fd_); fd_ = -1;
      return false;
    }

    // Baud rate
    speed_t speed = B115200;
    switch (baud_rate_) {
      case 9600:   speed = B9600;   break;
      case 38400:  speed = B38400;  break;
      case 57600:  speed = B57600;  break;
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      default:
        RCLCPP_WARN(get_logger(), "Unknown baud %d, defaulting to 115200", baud_rate_);
    }
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);

    // 8N1, raw mode, no flow control
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 10;  // 1-second read timeout on serial

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      ::close(fd_); fd_ = -1;
      return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close_fd() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  // ─── NMEA stream reader (works for both TCP and serial) ───────────
  void read_stream() {
    std::string buffer;
    char chunk[1024];

    while (running_ && rclcpp::ok()) {
      ssize_t n;
      if (conn_type_ == "tcp") {
        n = ::recv(fd_, chunk, sizeof(chunk) - 1, 0);
      } else {
        n = ::read(fd_, chunk, sizeof(chunk) - 1);
      }

      if (n <= 0) {
        if (n == 0 || errno != EAGAIN) {
          RCLCPP_WARN(get_logger(), "Connection lost on [%s]", conn_type_.c_str());
          break;
        }
        continue;
      }
      chunk[n] = '\0';
      buffer.append(chunk, static_cast<size_t>(n));

      // Process complete lines
      size_t pos;
      while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.size() >= 6 && line[0] == '$') {
          process_sentence(line);
        }
      }
    }
  }

  // ─── NMEA sentence processor ──────────────────────────────────────
  void process_sentence(const std::string& sentence) {
    auto stamp = now();

    // Always publish raw NMEA (for post-processing, RTK, logging)
    nmea_msgs::msg::Sentence nmea_msg;
    nmea_msg.header.stamp     = stamp;
    nmea_msg.header.frame_id  = frame_id_;
    nmea_msg.sentence         = sentence;
    nmea_pub_->publish(nmea_msg);

    // GGA → NavSatFix
    if (sentence.find("GGA") != std::string::npos) {
      auto gga = mtt_gps::parse_gga(sentence);
      if (!gga) return;

      // Publish TimeReference for temporal calibration
      sensor_msgs::msg::TimeReference time_ref;
      time_ref.header.stamp    = stamp;
      time_ref.header.frame_id = frame_id_;
      time_ref.source          = "gps_utc";
      // GPS time in seconds since midnight
      rclcpp::Time gps_t(static_cast<int64_t>(gga->timestamp_utc * 1e9));
      time_ref.time_ref = gps_t;
      timeref_pub_->publish(time_ref);

      // NavSatFix
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp     = stamp;
      fix.header.frame_id  = frame_id_;
      fix.latitude         = gga->latitude_deg;
      fix.longitude        = gga->longitude_deg;
      fix.altitude         = gga->altitude_m;
      fix.status.service   = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

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

      // Adaptive covariance based on fix quality
      double h_cov, v_cov;
      if (gga->fix_quality == 4) {        // RTK Fix: ~1 cm
        h_cov = 0.01 * 0.01;
        v_cov = 0.02 * 0.02;
      } else if (gga->fix_quality == 5) { // RTK Float: ~10 cm
        h_cov = 0.10 * 0.10;
        v_cov = 0.20 * 0.20;
      } else if (gga->fix_quality == 2) { // DGPS: ~50 cm
        h_cov = 0.50 * 0.50;
        v_cov = 1.00 * 1.00;
      } else {                            // SPP: HDOP-based
        h_cov = gga->hdop * gga->hdop;
        v_cov = h_cov * 4.0;
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

  // ─── Members ──────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr    fix_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr       nmea_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr timeref_pub_;

  std::string conn_type_, frame_id_, host_, serial_port_;
  int port_, baud_rate_;
  double reconnect_s_;
  int fd_{-1};
  std::atomic<bool> running_{true};
  std::thread reader_thread_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReachDriverNode>());
  rclcpp::shutdown();
  return 0;
}
