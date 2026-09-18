#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_msgs/msg/motion_restriction.hpp"
#include "mentorpi_msgs/msg/nearest_person.hpp"
#include "motion_control/follow_control.hpp"
#include "motion_control/guard_decision.hpp"
#include "motion_control/ros_inputs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motion_control {

static_assert(kControlAutoFollow == mentorpi_msgs::msg::ControlState::AUTO_FOLLOW);

// SD036 D6 (F13, F14). Independent of the planner: rolls the desired command out against a zone
// narrower than the bumper and is the only publisher of /control/motion_restriction.
class ObstacleGuardNode : public rclcpp::Node {
 public:
  ObstacleGuardNode() : Node("obstacle_guard") {
    const double rate_hz = require_positive("rate_hz", 20.0);
    nearest_timeout_ = std::chrono::milliseconds(require_positive_int("nearest_timeout_ms", 1000));
    command_timeout_ = std::chrono::milliseconds(require_positive_int("command_timeout_ms", 300));
    const auto scan_timeout =
        std::chrono::milliseconds(require_positive_int("scan_timeout_ms", 300));
    const auto persons_timeout =
        std::chrono::milliseconds(require_positive_int("persons_timeout_ms", 1000));

    params_.footprint.front = require_positive("footprint_front", 0.175);
    params_.footprint.back = require_positive("footprint_back", 0.166);
    params_.footprint.half_width = require_positive("footprint_half_width", 0.123);
    const double self_filter_pad = require_non_negative("self_filter_pad", 0.02);
    person_exclusion_radius_ = require_non_negative("person_exclusion_radius", 0.25);
    params_.stop_margin = require_positive("stop_margin", 0.02);
    params_.stop_horizon_s = require_positive("stop_horizon_s", 0.5);
    params_.rollout_dt = require_positive("rollout_dt", 0.05);
    params_.hold_s = static_cast<double>(require_positive_int("hold_ms", 300)) / 1000.0;
    if (!(params_.rollout_dt <= params_.stop_horizon_s)) {
      throw std::invalid_argument("rollout_dt must be <= stop_horizon_s");
    }

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    state_sub_ = create_subscription<mentorpi_msgs::msg::ControlState>(
        "/control/state", state_qos, [this](const mentorpi_msgs::msg::ControlState::SharedPtr msg) {
          have_state_ = true;
          state_ = msg->state;
        });
    nearest_sub_ = create_subscription<mentorpi_msgs::msg::NearestPerson>(
        "/perception/nearest_person", last_qos,
        [this](const mentorpi_msgs::msg::NearestPerson::SharedPtr msg) {
          have_nearest_ = true;
          nearest_valid_ = msg->valid;
          nearest_coasting_ = msg->coasting;
          nearest_stamp_ = std::chrono::steady_clock::now();
        });
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/pnc/desired_twist", last_qos, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          have_command_ = true;
          command_ = Twist2d{msg->linear.x, msg->angular.z};
          command_stamp_ = std::chrono::steady_clock::now();
        });
    scan_ = std::make_unique<ScanPoints>(*this, params_.footprint, self_filter_pad, scan_timeout);
    persons_ = std::make_unique<PersonPoints>(*this, persons_timeout);

    restriction_pub_ = create_publisher<mentorpi_msgs::msg::MotionRestriction>(
        "/control/motion_restriction", last_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&ObstacleGuardNode::on_timer, this));

    RCLCPP_INFO(
        get_logger(),
        "obstacle_guard started (rate_hz=%.1f stop_margin=%.3f stop_horizon_s=%.2f "
        "rollout_dt=%.2f hold_ms=%.0f footprint front=%.3f back=%.3f half_width=%.3f "
        "pad=%.3f person_radius=%.2f nearest_timeout_ms=%ld command_timeout_ms=%ld "
        "scan_timeout_ms=%ld persons_timeout_ms=%ld)",
        rate_hz, params_.stop_margin, params_.stop_horizon_s, params_.rollout_dt,
        params_.hold_s * 1000.0, params_.footprint.front, params_.footprint.back,
        params_.footprint.half_width, self_filter_pad, person_exclusion_radius_,
        static_cast<long>(nearest_timeout_.count()), static_cast<long>(command_timeout_.count()),
        static_cast<long>(scan_timeout.count()), static_cast<long>(persons_timeout.count()));
  }

 private:
  double require_positive(const char* name, double default_value) {
    declare_parameter<double>(name, default_value);
    const double value = get_parameter(name).as_double();
    if (!(value > 0.0)) {
      throw std::invalid_argument(std::string(name) + " must be > 0");
    }
    return value;
  }

  double require_non_negative(const char* name, double default_value) {
    declare_parameter<double>(name, default_value);
    const double value = get_parameter(name).as_double();
    if (!(value >= 0.0)) {
      throw std::invalid_argument(std::string(name) + " must be >= 0");
    }
    return value;
  }

  int64_t require_positive_int(const char* name, int64_t default_value) {
    declare_parameter<int64_t>(name, default_value);
    const int64_t value = get_parameter(name).as_int();
    if (value <= 0) {
      throw std::invalid_argument(std::string(name) + " must be > 0");
    }
    return value;
  }

  void on_timer() {
    const auto now = std::chrono::steady_clock::now();

    std::vector<Point2d> points;
    const bool scan_fresh = scan_->fresh();
    if (scan_fresh) {
      points = scan_->points();
      persons_->exclude(points, person_exclusion_radius_);
    }

    GuardInputs in{};
    in.have_state = have_state_;
    in.state = state_;
    in.target_usable = have_nearest_ && nearest_valid_ && !nearest_coasting_ &&
                       now - nearest_stamp_ <= nearest_timeout_;
    in.command_fresh = have_command_ && now - command_stamp_ <= command_timeout_;
    in.command = command_;
    in.scan_fresh = scan_fresh;
    in.points = scan_fresh ? &points : nullptr;
    in.now_s = std::chrono::duration<double>(now.time_since_epoch()).count();

    const GuardDecision d = decide_guard(in, params_, memory_);
    if (!have_published_ || d.stop_request != last_stop_ || std::strcmp(d.reason, last_reason_)) {
      RCLCPP_INFO(get_logger(), "guard stop_request %s reason='%s'",
                  d.stop_request ? "true" : "false", d.reason);
    }
    have_published_ = true;
    last_stop_ = d.stop_request;
    last_reason_ = d.reason;

    mentorpi_msgs::msg::MotionRestriction msg;
    msg.stop_request = d.stop_request;
    msg.reason = d.reason;
    restriction_pub_->publish(msg);
  }

  GuardParams params_{};
  GuardMemory memory_{};
  double person_exclusion_radius_{0.25};
  std::chrono::milliseconds nearest_timeout_{1000};
  std::chrono::milliseconds command_timeout_{300};

  bool have_state_{false};
  uint8_t state_{kControlForbidden};
  bool have_nearest_{false};
  bool nearest_valid_{false};
  bool nearest_coasting_{false};
  std::chrono::steady_clock::time_point nearest_stamp_{};
  bool have_command_{false};
  Twist2d command_{0.0, 0.0};
  std::chrono::steady_clock::time_point command_stamp_{};

  bool have_published_{false};
  bool last_stop_{false};
  const char* last_reason_{kReasonNone};

  std::unique_ptr<ScanPoints> scan_;
  std::unique_ptr<PersonPoints> persons_;
  rclcpp::Subscription<mentorpi_msgs::msg::ControlState>::SharedPtr state_sub_;
  rclcpp::Subscription<mentorpi_msgs::msg::NearestPerson>::SharedPtr nearest_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<mentorpi_msgs::msg::MotionRestriction>::SharedPtr restriction_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motion_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motion_control::ObstacleGuardNode>());
  rclcpp::shutdown();
  return 0;
}
