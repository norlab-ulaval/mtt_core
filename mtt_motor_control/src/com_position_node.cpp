// ============================================================================
// ComPositionNode implementation — spring-return COM motor position controller.
// ============================================================================

#include "mtt_motor_control/com_position_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

using namespace std::chrono_literals;

namespace mtt_motor_control
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

ComPositionNode::ComPositionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("com_position_node", options)
{
    // --- Parameters ---
    amplitude_   = declare_parameter("amplitude_counts",  50000.0);
    setup_slew_  = declare_parameter("setup_slew",         4000.0);
    run_slew_    = declare_parameter("run_slew",          20000.0);
    rearm_slew_  = declare_parameter("rearm_slew",         4000.0);
    const double rate = declare_parameter("publish_rate_hz", 100.0);
    dt_ = 1.0 / rate;

    // Pre-configured home (NaN = use SETUP workflow or calibration file).
    home_position_counts_ = declare_parameter(
        "home_position_counts", std::numeric_limits<double>::quiet_NaN());

    // Calibration persistence
    calib_file_ = declare_parameter(
        "calibration_file", std::string("/data/mtt/com_calibration.yaml"));
    consistency_threshold_ = declare_parameter("home_consistency_threshold", 1000.0);

    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const auto & p) { return on_param_change(p); });

    // --- Publisher ---
    cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/motor/cmd_position", 10);

    // --- Subscriptions ---
    com_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
        "mtt_control/com_mode", 10,
        [this](const std_msgs::msg::Bool::SharedPtr m) { on_com_mode(m); });

    steer_sub_ = create_subscription<std_msgs::msg::Float64>(
        "mtt_control/com_steer", 10,
        [this](const std_msgs::msg::Float64::SharedPtr m) { on_com_steer(m); });

    set_home_sub_ = create_subscription<std_msgs::msg::Empty>(
        "mtt_control/com_set_home", 10,
        [this](const std_msgs::msg::Empty::SharedPtr m) { on_set_home(m); });

    com_park_sub_ = create_subscription<std_msgs::msg::Empty>(
        "mtt_control/com_park", 10,
        [this](const std_msgs::msg::Empty::SharedPtr m) { on_com_park(m); });

    deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
        "mtt_control/teleop_deadman", 10,
        [this](const std_msgs::msg::Bool::SharedPtr m) { on_deadman(m); });

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/motor/joint_state", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr m) { on_joint_state(m); });

    // --- Control timer ---
    loop_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt_)),
        [this]() { loop(); });

    RCLCPP_INFO(get_logger(),
                "ComPositionNode ready — amplitude=%.0f counts, "
                "setup_slew=%.0f, run_slew=%.0f, rearm_slew=%.0f counts/s",
                amplitude_, setup_slew_, run_slew_, rearm_slew_);
    if (!std::isnan(home_position_counts_)) {
        RCLCPP_INFO(get_logger(), "  home_position_counts=%.0f (from YAML)",
                    home_position_counts_);
    }
    RCLCPP_INFO(get_logger(), "  calibration_file=%s  consistency_threshold=%.0f counts",
                calib_file_.c_str(), consistency_threshold_);
}

// ---------------------------------------------------------------------------
// State machine helpers
// ---------------------------------------------------------------------------

void ComPositionNode::enter_setup()
{
    cmd_        = actual_;
    home_       = std::nullopt;
    state_      = State::SETUP;
    sliding_in_ = false;
    RCLCPP_INFO(get_logger(),
                "COM SETUP — jog-velocity mode, slew=%.0f counts/s, start=%.0f",
                setup_slew_, cmd_);
}

void ComPositionNode::enter_run()
{
    // Called by SET_HOME: motor is physically at cmd_ right now.
    home_                 = cmd_;
    home_position_counts_ = cmd_;   // persist in-memory for future restarts
    state_                = State::RUN;
    sliding_in_           = false;
    RCLCPP_INFO(get_logger(),
                "COM RUN — spring ±%.0f counts around home=%.0f",
                amplitude_, home_.value());
    // Motor is currently at home → save calibration immediately.
    save_calibration();
}

void ComPositionNode::enter_run_at_configured_home()
{
    cmd_        = actual_;  // start smoothing from current motor position
    home_       = home_position_counts_;
    state_      = State::RUN;
    sliding_in_ = false;
    RCLCPP_INFO(get_logger(),
                "COM RUN (configured home=%.0f) — motor will slew to home when deadman held.",
                home_.value());
}

void ComPositionNode::enter_park()
{
    state_        = State::PARK;
    park_arrived_ = false;
    sliding_in_   = false;
    RCLCPP_INFO(get_logger(),
                "COM PARK — returning to home (%.0f) at %.0f counts/s. "
                "Watch for 'PARK complete' before powering off.",
                home_.value(), rearm_slew_);
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------

void ComPositionNode::on_com_mode(const std_msgs::msg::Bool::SharedPtr msg)
{
    const bool new_mode = msg->data;

    if (new_mode && !com_mode_) {
        // COM turned ON
        if (seeded_) {
            if (!std::isnan(home_position_counts_)) {
                enter_run_at_configured_home();
            } else {
                enter_setup();
            }
        } else {
            pending_setup_ = true;
            RCLCPP_WARN(get_logger(),
                        "COM ON but /motor/joint_state not yet received — "
                        "SETUP pending.");
        }
    } else if (!new_mode && com_mode_) {
        // COM turned OFF — cancel any in-progress park or setup
        state_         = State::OFF;
        pending_setup_ = false;
        park_arrived_  = false;
        RCLCPP_INFO(get_logger(), "COM OFF — driver holds last position.");
    }

    com_mode_ = new_mode;
}

void ComPositionNode::on_com_steer(const std_msgs::msg::Float64::SharedPtr msg)
{
    steer_ = std::clamp(msg->data, -1.0, 1.0);
}

void ComPositionNode::on_set_home(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
    if (state_ != State::SETUP && state_ != State::RUN) {
        RCLCPP_WARN(get_logger(), "set_home ignored — COM not in SETUP/RUN.");
        return;
    }
    enter_run();
}

void ComPositionNode::on_com_park(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
    if (state_ == State::PARK) {
        RCLCPP_INFO(get_logger(), "com_park: already parking.");
        return;
    }
    if (state_ == State::SETUP) {
        RCLCPP_WARN(get_logger(), "com_park ignored — home not set yet (in SETUP).");
        return;
    }
    if (!home_.has_value()) {
        RCLCPP_WARN(get_logger(),
                    "com_park ignored — home not set. "
                    "Enter COM mode and use com_set_home first.");
        return;
    }
    // Accept from RUN or OFF (home known from previous session)
    enter_park();
}

void ComPositionNode::on_deadman(const std_msgs::msg::Bool::SharedPtr msg)
{
    deadman_ = msg->data;
}

void ComPositionNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.empty()) { return; }
    actual_ = msg->position[0];

    if (!seeded_) {
        seeded_ = true;
        if (pending_setup_) {
            pending_setup_ = false;
            // Priority: YAML param > calibration file > manual SETUP.
            if (std::isnan(home_position_counts_)) {
                try_restore_from_calibration();  // may set home_position_counts_
            }
            if (!std::isnan(home_position_counts_)) {
                enter_run_at_configured_home();
            } else {
                enter_setup();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 100-Hz control loop
// ---------------------------------------------------------------------------

void ComPositionNode::loop()
{
    if (state_ == State::OFF || !seeded_) {
        prev_deadman_ = deadman_;
        return;
    }

    // ── PARK: autonomous return to home, independent of dead-man ─────────
    if (state_ == State::PARK) {
        const double h    = home_.value();
        const double step = rearm_slew_ * dt_;
        cmd_ = move_toward(cmd_, h, step);

        if (!park_arrived_ && std::abs(cmd_ - h) <= step) {
            cmd_          = h;
            park_arrived_ = true;
            save_calibration();
            RCLCPP_INFO(get_logger(),
                        "COM PARK complete — motor at home (%.0f counts). "
                        "Calibration saved. Safe to power off.",
                        h);
        }

        auto out = std_msgs::msg::Float64();
        out.data = cmd_;
        cmd_pub_->publish(out);
        prev_deadman_ = deadman_;
        return;
    }

    // ── SETUP: jog-velocity mode ──────────────────────────────────────────
    if (state_ == State::SETUP) {
        if (deadman_) {
            cmd_ += steer_ * setup_slew_ * dt_;
        }

    // ── RUN: spring-return position mode ─────────────────────────────────
    } else {
        const double h      = home_.value();
        const double target = std::clamp(h + steer_ * amplitude_,
                                         h - amplitude_,
                                         h + amplitude_);
        if (deadman_) {
            if (!prev_deadman_) { sliding_in_ = true; }

            const double slew = sliding_in_ ? rearm_slew_ : run_slew_;
            const double step = slew * dt_;
            cmd_ = move_toward(cmd_, target, step);

            if (sliding_in_ && std::abs(cmd_ - target) <= step) {
                sliding_in_ = false;
            }
        }
    }

    // Publish (SETUP + RUN)
    auto out = std_msgs::msg::Float64();
    out.data = cmd_;
    cmd_pub_->publish(out);

    prev_deadman_ = deadman_;
}

// ---------------------------------------------------------------------------
// Calibration persistence
// ---------------------------------------------------------------------------

void ComPositionNode::save_calibration() const
{
    if (!home_.has_value()) { return; }

    // Ensure directory exists
    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(calib_file_).parent_path(), ec);
    if (ec) {
        RCLCPP_WARN(get_logger(),
                    "Cannot create calibration directory for %s: %s",
                    calib_file_.c_str(), ec.message().c_str());
        return;
    }

    std::ofstream f(calib_file_);
    if (!f.is_open()) {
        RCLCPP_WARN(get_logger(),
                    "Cannot write calibration file %s", calib_file_.c_str());
        return;
    }

    f << std::fixed << std::setprecision(1)
      << "# MTT COM motor calibration — written by com_position_node\n"
      << "# DO NOT edit manually. Re-home via SETUP → com_set_home.\n"
      << "home_counts: "    << home_.value() << "\n"
      << "actual_at_save: " << actual_       << "\n";

    RCLCPP_INFO(get_logger(),
                "Calibration saved → %s  (home=%.0f  actual=%.0f)",
                calib_file_.c_str(), home_.value(), actual_);
}

bool ComPositionNode::try_restore_from_calibration()
{
    std::ifstream f(calib_file_);
    if (!f.is_open()) { return false; }

    double saved_home   = std::numeric_limits<double>::quiet_NaN();
    double saved_actual = std::numeric_limits<double>::quiet_NaN();
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') { continue; }
        if (line.rfind("home_counts:", 0) == 0) {
            try { saved_home   = std::stod(line.substr(12)); } catch (...) {}
        } else if (line.rfind("actual_at_save:", 0) == 0) {
            try { saved_actual = std::stod(line.substr(15)); } catch (...) {}
        }
    }

    if (std::isnan(saved_home) || std::isnan(saved_actual)) {
        RCLCPP_WARN(get_logger(),
                    "Calibration file %s is malformed — ignoring.",
                    calib_file_.c_str());
        return false;
    }

    const double drift = std::abs(actual_ - saved_actual);
    if (drift < consistency_threshold_) {
        home_position_counts_ = saved_home;
        RCLCPP_INFO(get_logger(),
                    "Calibration restored from %s — home=%.0f, "
                    "position drift=%.0f counts (< %.0f threshold). "
                    "Entering RUN directly (no SETUP needed).",
                    calib_file_.c_str(), saved_home, drift, consistency_threshold_);
        return true;
    }

    RCLCPP_WARN(get_logger(),
                "Calibration file found but position mismatch: "
                "actual=%.0f, saved=%.0f, drift=%.0f counts (>= %.0f threshold). "
                "Motor was likely power-cycled. SETUP required — "
                "motor should be at home position, just press com_set_home.",
                actual_, saved_actual, drift, consistency_threshold_);
    return false;
}

// ---------------------------------------------------------------------------
// Parameter hot-reload
// ---------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult
ComPositionNode::on_param_change(const std::vector<rclcpp::Parameter> & params)
{
    for (const auto & p : params) {
        if      (p.get_name() == "amplitude_counts")         amplitude_              = p.as_double();
        else if (p.get_name() == "setup_slew")               setup_slew_             = p.as_double();
        else if (p.get_name() == "run_slew")                 run_slew_               = p.as_double();
        else if (p.get_name() == "rearm_slew")               rearm_slew_             = p.as_double();
        else if (p.get_name() == "home_position_counts")     home_position_counts_   = p.as_double();
        else if (p.get_name() == "home_consistency_threshold") consistency_threshold_ = p.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

// ---------------------------------------------------------------------------
// Geometry helper
// ---------------------------------------------------------------------------

double ComPositionNode::move_toward(double cur, double target, double step) noexcept
{
    const double delta = target - cur;
    if (std::abs(delta) <= step) { return target; }
    return cur + std::copysign(step, delta);
}

}  // namespace mtt_motor_control

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mtt_motor_control::ComPositionNode>());
    rclcpp::shutdown();
    return 0;
}
