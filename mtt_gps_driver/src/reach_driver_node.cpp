// Emlid Reach RS unified driver node.
// Supports both TCP (WiFi) and USB Serial connections via `connection_type` parameter.
//
// Timestamp strategy — GPS-time stamped fixes:
//   GGA gives UTC time-of-day (hhmmss.ss).
//   RMC gives the calendar date (DDMMYY) + the same time-of-day.
//   When both are known we compute the exact GPS UTC nanosecond epoch and use
//   it as the message stamp.  This gives ~1ms accuracy without hardware PPS.
//   On startup (before first RMC), we fall back to now().
//
// Topics published (all relative to node namespace, e.g. /gps/):
//   fix            — sensor_msgs/NavSatFix  with adaptive RTK covariance
//   nmea_sentence  — nmea_msgs/Sentence     raw NMEA for PPK post-processing
//   time_reference — sensor_msgs/TimeReference  GPS UTC vs. ROS clock offset

#include <chrono>
#include <cstring>
#include <ctime>
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

// ─── Helper: convert GPS calendar date + time-of-day → nanoseconds since Unix epoch ──
// Returns 0 if date is not yet known (day == 0).
static int64_t gps_utc_to_ns(int day, int month, int year_2digit, double time_of_day_s)
{
  if (day == 0) return 0;
  struct tm t{};
  t.tm_year  = year_2digit + 100;  // years since 1900; 20xx → 100+yy
  t.tm_mon   = month - 1;          // 0-indexed
  t.tm_mday  = day;
  t.tm_hour  = 0;
  t.tm_min   = 0;
  t.tm_sec   = 0;
  t.tm_isdst = 0;
  // timegm: POSIX, converts UTC tm to Unix time_t without local timezone
  time_t unix_epoch = timegm(&t);
  double unix_s = static_cast<double>(unix_epoch) + time_of_day_s;
  return static_cast<int64_t>(unix_s * 1.0e9);
}

class ReachDriverNode : public rclcpp::Node {
public:
  ReachDriverNode() : Node("reach_driver_node") {
    // ── Parameters ──────────────────────────────────────────────────
    declare_parameter("connection_type", "serial");  // "tcp" | "serial"
    declare_parameter("frame_id", "gps_rover_link");
    declare_parameter("reconnect_interval_s", 3.0);

    // TCP
    declare_parameter("host", "192.168.2.10");
    declare_parameter("port", 9001);

    // Serial
    declare_parameter("serial_port", "/dev/reach_rover");
    declare_parameter("baud_rate", 115200);

    conn_type_   = get_parameter("connection_type").as_string();
    frame_id_    = get_parameter("frame_id").as_string();
    reconnect_s_ = get_parameter("reconnect_interval_s").as_double();
    host_        = get_parameter("host").as_string();
    port_        = get_parameter("port").as_int();
    serial_port_ = get_parameter("serial_port").as_string();
    baud_rate_   = get_parameter("baud_rate").as_int();

    if (conn_type_ != "tcp" && conn_type_ != "serial") {
      RCLCPP_FATAL(get_logger(), "connection_type must be 'tcp' or 'serial', got: '%s'",
                   conn_type_.c_str());
      throw std::invalid_argument("Invalid connection_type: " + conn_type_);
    }

    // ── Publishers ──────────────────────────────────────────────────
    fix_pub_     = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
    nmea_pub_    = create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 50);
    timeref_pub_ = create_publisher<sensor_msgs::msg::TimeReference>("time_reference", 10);
    diag_timer_  = create_wall_timer(15s, std::bind(&ReachDriverNode::report_diagnostics, this));

    reader_thread_ = std::thread(&ReachDriverNode::io_loop, this);

    RCLCPP_INFO(get_logger(), "Reach driver started [%s] frame=%s",
                conn_type_.c_str(), frame_id_.c_str());
    if (conn_type_ == "tcp") {
      RCLCPP_INFO(get_logger(), "  TCP target: %s:%d", host_.c_str(), port_);
    } else {
      RCLCPP_INFO(get_logger(), "  Serial: %s @ %d baud", serial_port_.c_str(), baud_rate_);
    }
    RCLCPP_INFO(get_logger(), "  Timestamp: GPS-UTC once RMC date received, ROS clock otherwise");
  }

  ~ReachDriverNode() override {
    running_ = false;
    if (reader_thread_.joinable()) reader_thread_.join();
    close_fd();
  }

private:
  // ─── I/O loop (reconnects on failure) ─────────────────────────────
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

    struct timeval tv{5, 0};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
      ::close(fd_); fd_ = -1; return false;
    }
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(fd_); fd_ = -1; return false;
    }
    return true;
  }

  // ─── Serial connection (POSIX termios) ────────────────────────────
  bool connect_serial() {
    fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      ::close(fd_); fd_ = -1; return false;
    }

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

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 10;  // 1-second read timeout

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      ::close(fd_); fd_ = -1; return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close_fd() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  // ─── NMEA stream reader ────────────────────────────────────────────
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

  // ─── GPS-time stamp computation ───────────────────────────────────
  // Returns a GPS-UTC-based rclcpp::Time when date is known, or now() otherwise.
  // Also publishes TimeReference on every call so the bag contains the full
  // GPS↔ROS clock correspondence for post-processing alignment.
  rclcpp::Time make_gps_stamp(double time_of_day_s) {
    auto ros_now = now();

    int64_t gps_ns = gps_utc_to_ns(rmc_day_, rmc_month_, rmc_year_, time_of_day_s);
    if (gps_ns == 0) {
      // Date not yet received from RMC — fall back to ROS clock
      return ros_now;
    }

    rclcpp::Time gps_time(gps_ns, RCL_ROS_TIME);

    // Publish TimeReference: GPS UTC vs. ROS reception time.
    // This lets post-processing tools compute the exact GPS↔ROS offset.
    sensor_msgs::msg::TimeReference time_ref;
    time_ref.header.stamp    = ros_now;
    time_ref.header.frame_id = frame_id_;
    time_ref.source          = "gps_utc";
    time_ref.time_ref        = gps_time;
    timeref_pub_->publish(time_ref);
    ++timeref_publish_count_;

    // Warn if the GPS↔ROS offset is large (> 1 second indicates clock sync issue)
    double offset_s = (ros_now - gps_time).seconds();
    if (std::abs(offset_s) > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "GPS↔ROS clock offset = %.3f s — consider NTP sync to GPS time", offset_s);
    }

    return gps_time;
  }

  // ─── NMEA sentence processor ──────────────────────────────────────
  void process_sentence(const std::string& sentence) {
    ++nmea_sentence_count_;

    // Always publish raw NMEA (for PPK post-processing and offline RTK recomputation)
    nmea_msgs::msg::Sentence nmea_msg;
    nmea_msg.header.stamp    = now();
    nmea_msg.header.frame_id = frame_id_;
    nmea_msg.sentence        = sentence;
    nmea_pub_->publish(nmea_msg);

    // ── RMC: update calendar date (needed for GPS-UTC timestamp) ────
    if (sentence.find("RMC") != std::string::npos) {
      ++rmc_sentence_count_;
      auto rmc = mtt_gps::parse_rmc(sentence);
      if (rmc && rmc->valid && rmc->day != 0) {
        ++rmc_valid_count_;
        rmc_day_   = rmc->day;
        rmc_month_ = rmc->month;
        rmc_year_  = rmc->year_2digit;
        if (!date_received_) {
          RCLCPP_INFO(get_logger(),
              "GPS date received: %02d/%02d/20%02d — switching to GPS-UTC timestamps",
              rmc_day_, rmc_month_, rmc_year_);
          date_received_ = true;
        }
      } else {
        ++rmc_rejected_count_;
      }
      return;
    }

    // ── GGA: position fix ────────────────────────────────────────────
    if (sentence.find("GGA") != std::string::npos) {
      ++gga_sentence_count_;
      auto gga = mtt_gps::parse_gga(sentence);
      if (!gga) {
        ++gga_rejected_count_;
        return;
      }
      ++gga_valid_count_;
      last_valid_gga_ros_time_ns_.store(now().nanoseconds());

      // GPS-time stamp (uses RMC date if available, ROS clock otherwise)
      rclcpp::Time stamp = make_gps_stamp(gga->timestamp_utc);

      // NavSatFix with adaptive RTK covariance
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp    = stamp;
      fix.header.frame_id = frame_id_;
      fix.latitude        = gga->latitude_deg;
      fix.longitude       = gga->longitude_deg;
      fix.altitude        = gga->altitude_m;
      fix.status.service  = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

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

      // Covariance: RTK Fix ~1cm, Float ~10cm, DGPS ~50cm, SPP HDOP-based
      double h_cov, v_cov;
      switch (gga->fix_quality) {
        case 4:  h_cov = 0.01 * 0.01; v_cov = 0.02 * 0.02; break;  // RTK Fix
        case 5:  h_cov = 0.10 * 0.10; v_cov = 0.20 * 0.20; break;  // RTK Float
        case 2:  h_cov = 0.50 * 0.50; v_cov = 1.00 * 1.00; break;  // DGPS
        default: h_cov = gga->hdop * gga->hdop; v_cov = h_cov * 4.0; break; // SPP
      }
      fix.position_covariance = {
        h_cov, 0.0, 0.0,
        0.0, h_cov, 0.0,
        0.0, 0.0, v_cov
      };
      fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

      fix_pub_->publish(fix);

      // Log fix quality changes
      if (gga->fix_quality != last_fix_quality_) {
        const char* qual_str[] = {"NO FIX", "GPS SPP", "DGPS", "?", "RTK Fix", "RTK Float"};
        int qi = std::min(gga->fix_quality, 5);
        RCLCPP_INFO(get_logger(), "Fix quality changed: %s (satellites=%d, HDOP=%.1f)",
                    qual_str[qi], gga->num_satellites, gga->hdop);
        last_fix_quality_ = gga->fix_quality;
      }
    }
  }

  void report_diagnostics() {
    const auto total_nmea = nmea_sentence_count_.load();
    const auto gga_seen = gga_sentence_count_.load();
    const auto gga_valid = gga_valid_count_.load();
    const auto gga_rejected = gga_rejected_count_.load();
    const auto rmc_seen = rmc_sentence_count_.load();
    const auto rmc_valid = rmc_valid_count_.load();
    const auto rmc_rejected = rmc_rejected_count_.load();
    const auto timeref_count = timeref_publish_count_.load();

    if (total_nmea == 0) {
      RCLCPP_WARN(get_logger(), "No NMEA sentence received yet on [%s]", conn_type_.c_str());
      return;
    }

    const auto last_valid_gga_ns = last_valid_gga_ros_time_ns_.load();
    const double last_valid_gga_age_s =
      last_valid_gga_ns > 0
        ? (now().nanoseconds() - last_valid_gga_ns) / 1.0e9
        : -1.0;

    if (gga_seen > 0 && gga_valid == 0) {
      RCLCPP_WARN(
        get_logger(),
        "NMEA is flowing but no valid GGA is being parsed yet "
        "(NMEA=%llu GGA=%llu rejected=%llu RMC=%llu valid_RMC=%llu). "
        "Check ReachView3 USB output: enable GGA at 5 Hz and verify checksums.",
        static_cast<unsigned long long>(total_nmea),
        static_cast<unsigned long long>(gga_seen),
        static_cast<unsigned long long>(gga_rejected),
        static_cast<unsigned long long>(rmc_seen),
        static_cast<unsigned long long>(rmc_valid));
      return;
    }

    if (gga_valid > 0 && rmc_seen > 0 && rmc_valid == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Valid GGA is present but no valid RMC date has been parsed yet "
        "(RMC=%llu rejected=%llu). /gps/fix can publish, but GPS UTC timestamps "
        "and /gps/time_reference stay degraded until RMC is valid.",
        static_cast<unsigned long long>(rmc_seen),
        static_cast<unsigned long long>(rmc_rejected));
      return;
    }

    if (last_valid_gga_age_s > 5.0) {
      RCLCPP_WARN(
        get_logger(),
        "Last valid GGA is stale (age=%.1fs). GPS sentences are present but the fix stream is not fresh.",
        last_valid_gga_age_s);
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      60000,
      "GPS NMEA stats: total=%llu GGA seen/valid/rejected=%llu/%llu/%llu "
      "RMC seen/valid/rejected=%llu/%llu/%llu time_refs=%llu",
      static_cast<unsigned long long>(total_nmea),
      static_cast<unsigned long long>(gga_seen),
      static_cast<unsigned long long>(gga_valid),
      static_cast<unsigned long long>(gga_rejected),
      static_cast<unsigned long long>(rmc_seen),
      static_cast<unsigned long long>(rmc_valid),
      static_cast<unsigned long long>(rmc_rejected),
      static_cast<unsigned long long>(timeref_count));
  }

  // ─── Members ──────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr      fix_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr          nmea_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr   timeref_pub_;

  std::string conn_type_, frame_id_, host_, serial_port_;
  int port_, baud_rate_;
  double reconnect_s_;
  int fd_{-1};
  std::atomic<bool> running_{true};
  std::thread reader_thread_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // GPS calendar date (from RMC) — 0 until first valid RMC received
  int  rmc_day_{0}, rmc_month_{0}, rmc_year_{0};
  bool date_received_{false};
  int  last_fix_quality_{-1};
  std::atomic<uint64_t> nmea_sentence_count_{0};
  std::atomic<uint64_t> gga_sentence_count_{0};
  std::atomic<uint64_t> gga_valid_count_{0};
  std::atomic<uint64_t> gga_rejected_count_{0};
  std::atomic<uint64_t> rmc_sentence_count_{0};
  std::atomic<uint64_t> rmc_valid_count_{0};
  std::atomic<uint64_t> rmc_rejected_count_{0};
  std::atomic<uint64_t> timeref_publish_count_{0};
  std::atomic<int64_t> last_valid_gga_ros_time_ns_{0};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReachDriverNode>());
  rclcpp::shutdown();
  return 0;
}
