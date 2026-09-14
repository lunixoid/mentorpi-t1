#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "mentorpi_control/control_mode.hpp"
#include "mentorpi_control/control_mux_gate.hpp"
#include "mentorpi_control/publish_cadence.hpp"
#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_msgs/msg/motion_restriction.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mentorpi_control {

static_assert(kControlForbidden == mentorpi_msgs::msg::ControlState::FORBIDDEN);
static_assert(kControlManual == mentorpi_msgs::msg::ControlState::MANUAL);
static_assert(kControlAutoFollow == mentorpi_msgs::msg::ControlState::AUTO_FOLLOW);

class ControlMuxNode : public rclcpp::Node {
 public:
  ControlMuxNode() : Node("control_mux") {
    declare_parameter<double>("rate_hz", 20.0);

    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }
    publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    state_sub_ = create_subscription<mentorpi_msgs::msg::ControlState>(
        "/control/state", state_qos,
        std::bind(&ControlMuxNode::on_state, this, std::placeholders::_1));
    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/control/manual_cmd_vel", last_qos,
        std::bind(&ControlMuxNode::on_manual, this, std::placeholders::_1));
    follow_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/pnc/desired_twist", last_qos,
        std::bind(&ControlMuxNode::on_follow, this, std::placeholders::_1));
    restriction_sub_ = create_subscription<mentorpi_msgs::msg::MotionRestriction>(
        "/control/motion_restriction", last_qos,
        std::bind(&ControlMuxNode::on_restriction, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/vehicle/cmd_vel", last_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&ControlMuxNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "control_mux started (rate_hz=%.1f)", rate_hz);
  }

 private:
  Twist2d decide_now() const { return gate_mux(state_, stop_request_, manual_, follow_); }

  void emit(bool force) {
    const auto now = std::chrono::steady_clock::now();
    if (!force && !timer_keepalive_due(ever_sent_, now, last_cmd_send_, publish_period_)) {
      return;
    }
    const Twist2d cmd = decide_now();
    geometry_msgs::msg::Twist twist{};
    twist.linear.x = cmd.linear_x;
    twist.angular.z = cmd.angular_z;
    cmd_pub_->publish(twist);
    ever_sent_ = true;
    last_cmd_send_ = now;
  }

  static const char* state_name(uint8_t state) {
    switch (state) {
      case kControlForbidden:
        return "forbidden";
      case kControlManual:
        return "manual";
      case kControlAutoFollow:
        return "follow";
      default:
        return "unknown";
    }
  }

  void on_state(const mentorpi_msgs::msg::ControlState::SharedPtr msg) {
    const uint8_t next = msg->state;
    if (!have_state_ || next != state_) {
      RCLCPP_INFO(get_logger(), "mux state %s -> %s", have_state_ ? state_name(state_) : "none",
                  state_name(next));
    }
    have_state_ = true;
    state_ = next;
    emit(true);
  }

  void on_manual(const geometry_msgs::msg::Twist::SharedPtr msg) {
    manual_.linear_x = msg->linear.x;
    manual_.angular_z = msg->angular.z;
    emit(true);
  }

  void on_follow(const geometry_msgs::msg::Twist::SharedPtr msg) {
    follow_.linear_x = msg->linear.x;
    follow_.angular_z = msg->angular.z;
    emit(true);
  }

  void on_restriction(const mentorpi_msgs::msg::MotionRestriction::SharedPtr msg) {
    const bool next = msg->stop_request;
    if (!have_restriction_ || next != stop_request_) {
      RCLCPP_INFO(get_logger(), "mux stop_request %s", next ? "true" : "false");
    }
    have_restriction_ = true;
    stop_request_ = next;
    emit(true);
  }

  void on_timer() { emit(false); }

  uint8_t state_{kControlAutoFollow};
  bool have_state_{false};
  bool stop_request_{false};
  bool have_restriction_{false};
  Twist2d manual_{};
  Twist2d follow_{};
  std::chrono::nanoseconds publish_period_{};
  std::chrono::steady_clock::time_point last_cmd_send_{};
  bool ever_sent_{false};

  rclcpp::Subscription<mentorpi_msgs::msg::ControlState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr follow_sub_;
  rclcpp::Subscription<mentorpi_msgs::msg::MotionRestriction>::SharedPtr restriction_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mentorpi_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_control::ControlMuxNode>());
  rclcpp::shutdown();
  return 0;
}
