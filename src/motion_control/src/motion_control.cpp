#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_msgs/msg/nearest_person.hpp"
#include "motion_control/follow_control.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motion_control {

static_assert(kControlForbidden == mentorpi_msgs::msg::ControlState::FORBIDDEN);
static_assert(kControlManual == mentorpi_msgs::msg::ControlState::MANUAL);
static_assert(kControlAutoFollow == mentorpi_msgs::msg::ControlState::AUTO_FOLLOW);

class MotionControlNode : public rclcpp::Node {
 public:
  MotionControlNode() : Node("motion_control") {
    params_.standoff = require_positive("standoff", 0.5);
    params_.max_linear = require_positive("max_linear", 0.37);
    params_.max_linear_follow = require_positive("max_linear_follow", 0.20);
    params_.max_angular = require_positive("max_angular", 2.0);
    params_.kp_lin = require_positive("kp_lin", 0.8);
    params_.kp_ang = require_positive("kp_ang", 1.5);
    params_.dist_deadband = require_positive("dist_deadband", 0.05);
    params_.min_breakaway_linear = require_positive("min_breakaway_linear", 0.10);
    params_.accel_linear = require_positive("accel_linear", 0.30);
    params_.decel_linear = require_positive("decel_linear", 0.60);
    params_.ang_deadband = require_positive("ang_deadband", 0.05);
    params_.track_half_sum = require_positive("track_half_sum", 0.1407);

    if (!(params_.max_linear_follow <= params_.max_linear)) {
      throw std::invalid_argument("max_linear_follow must be <= max_linear");
    }

    // Read once at start; reject runtime sets instead of silently ignoring them.
    rcl_interfaces::msg::ParameterDescriptor start_only;
    start_only.read_only = true;
    declare_parameter<int64_t>("nearest_timeout_ms", 1000, start_only);
    declare_parameter<double>("rate_hz", 20.0);

    const int64_t nearest_timeout_ms = get_parameter("nearest_timeout_ms").as_int();
    if (nearest_timeout_ms <= 0) {
      throw std::invalid_argument("nearest_timeout_ms must be > 0");
    }
    nearest_timeout_ = std::chrono::milliseconds(nearest_timeout_ms);

    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }
    // Nominal period, not a measured one (D2.4): a late tick then only slows the ramp down.
    dt_ = 1.0 / rate_hz;

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    control_state_sub_ = create_subscription<mentorpi_msgs::msg::ControlState>(
        "/control/state", state_qos,
        std::bind(&MotionControlNode::on_control_state, this, std::placeholders::_1));
    nearest_sub_ = create_subscription<mentorpi_msgs::msg::NearestPerson>(
        "/perception/nearest_person", last_qos,
        std::bind(&MotionControlNode::on_nearest, this, std::placeholders::_1));

    desired_twist_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("/pnc/desired_twist", last_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&MotionControlNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
                "motion_control started (rate_hz=%.1f standoff=%.2f kp_lin=%.2f "
                "max_linear_follow=%.2f accel_linear=%.2f decel_linear=%.2f "
                "nearest_timeout_ms=%ld)",
                rate_hz, params_.standoff, params_.kp_lin, params_.max_linear_follow,
                params_.accel_linear, params_.decel_linear, nearest_timeout_ms);
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

  void on_control_state(const mentorpi_msgs::msg::ControlState::SharedPtr msg) {
    have_control_state_ = true;
    control_state_ = msg->state;
  }

  void on_nearest(const mentorpi_msgs::msg::NearestPerson::SharedPtr msg) {
    have_nearest_ = true;
    nearest_valid_ = msg->valid;
    nearest_coasting_ = msg->coasting;
    nearest_x_ = static_cast<double>(msg->person.x);
    nearest_y_ = static_cast<double>(msg->person.y);
    last_nearest_stamp_ = std::chrono::steady_clock::now();
  }

  void on_timer() {
    const bool nearest_fresh =
        have_nearest_ &&
        (std::chrono::steady_clock::now() - last_nearest_stamp_ <= nearest_timeout_);

    geometry_msgs::msg::Twist twist{};
    const bool gate_ok = have_control_state_ && control_state_ == kControlAutoFollow &&
                         nearest_valid_ && !nearest_coasting_ && nearest_fresh;
    if (gate_ok) {
      const Twist2d cmd = compute_follow_twist(nearest_x_, nearest_y_, prev_linear_, dt_, params_);
      twist.linear.x = cmd.linear_x;
      twist.angular.z = cmd.angular_z;
      prev_linear_ = cmd.linear_x;
    } else {
      // D4: the gate wins over the ramp. Zeros go out on this very tick, and the ramp state is
      // dropped so returning to follow starts from a standstill, not from the previous speed.
      prev_linear_ = 0.0;
    }
    desired_twist_pub_->publish(twist);
  }

  FollowControlParams params_{};
  double dt_{0.05};
  double prev_linear_{0.0};
  std::chrono::milliseconds nearest_timeout_{1000};
  bool have_control_state_{false};
  uint8_t control_state_{kControlForbidden};
  bool have_nearest_{false};
  bool nearest_valid_{false};
  bool nearest_coasting_{false};
  double nearest_x_{0.0};
  double nearest_y_{0.0};
  std::chrono::steady_clock::time_point last_nearest_stamp_{};

  rclcpp::Subscription<mentorpi_msgs::msg::ControlState>::SharedPtr control_state_sub_;
  rclcpp::Subscription<mentorpi_msgs::msg::NearestPerson>::SharedPtr nearest_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desired_twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motion_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motion_control::MotionControlNode>());
  rclcpp::shutdown();
  return 0;
}
