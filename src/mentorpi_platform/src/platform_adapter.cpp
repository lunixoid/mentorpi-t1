#include <chrono>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "mentorpi_msgs/msg/chassis_status.hpp"
#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_platform/adapter_gate.hpp"
#include "mentorpi_platform/publish_cadence.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mentorpi_platform {

class PlatformAdapterNode : public rclcpp::Node {
 public:
  PlatformAdapterNode() : Node("platform_adapter") {
    declare_parameter<int64_t>("cmd_timeout_ms", 100);
    declare_parameter<double>("rate_hz", 20.0);

    const int64_t cmd_timeout_ms = get_parameter("cmd_timeout_ms").as_int();
    if (cmd_timeout_ms <= 0) {
      throw std::invalid_argument("cmd_timeout_ms must be > 0");
    }
    cmd_timeout_ = std::chrono::milliseconds(cmd_timeout_ms);

    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }
    publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // control_mux is the sole publisher of /vehicle/cmd_vel. This node stays
    // the last gate: watchdog + FORBIDDEN fail-safe, not mode arbitration.
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/vehicle/cmd_vel", last_qos,
        std::bind(&PlatformAdapterNode::on_cmd, this, std::placeholders::_1));
    state_sub_ = create_subscription<mentorpi_msgs::msg::ControlState>(
        "/control/state", state_qos,
        std::bind(&PlatformAdapterNode::on_state, this, std::placeholders::_1));

    chassis_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("/hiwonder_controller/cmd_vel", last_qos);
    status_pub_ = create_publisher<mentorpi_msgs::msg::ChassisStatus>("/vehicle/status", last_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&PlatformAdapterNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "platform_adapter started (rate_hz=%.1f cmd_timeout_ms=%ld)", rate_hz,
                cmd_timeout_ms);
  }

 private:
  AdapterOutput decide_now() const {
    const bool timed_out =
        have_cmd_ && (std::chrono::steady_clock::now() - last_cmd_stamp_ > cmd_timeout_);
    Twist2d last;
    last.linear_x = last_twist_.linear.x;
    last.angular_z = last_twist_.angular.z;
    return gate_adapter(have_cmd_, timed_out, have_state_, forbidden_, last);
  }

  void emit(bool force) {
    const auto now = std::chrono::steady_clock::now();
    if (!force && !timer_keepalive_due(ever_sent_, now, last_chassis_send_, publish_period_)) {
      return;
    }
    const AdapterOutput out = decide_now();
    geometry_msgs::msg::Twist chassis{};
    chassis.linear.x = out.chassis.linear_x;
    chassis.angular.z = out.chassis.angular_z;
    chassis_pub_->publish(chassis);

    mentorpi_msgs::msg::ChassisStatus status;
    status.command_timeout = out.command_timeout;
    status.forbidden = out.forbidden;
    status_pub_->publish(status);
    ever_sent_ = true;
    last_chassis_send_ = now;
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_twist_ = *msg;
    last_cmd_stamp_ = std::chrono::steady_clock::now();
    have_cmd_ = true;
    emit(true);
  }

  void on_state(const mentorpi_msgs::msg::ControlState::SharedPtr msg) {
    have_state_ = true;
    forbidden_ = (msg->state == mentorpi_msgs::msg::ControlState::FORBIDDEN);
    emit(true);
  }

  void on_timer() { emit(false); }

  std::chrono::milliseconds cmd_timeout_{100};
  std::chrono::nanoseconds publish_period_{};
  std::chrono::steady_clock::time_point last_chassis_send_{};
  bool ever_sent_{false};
  geometry_msgs::msg::Twist last_twist_{};
  std::chrono::steady_clock::time_point last_cmd_stamp_{};
  bool have_cmd_{false};
  bool have_state_{false};
  bool forbidden_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<mentorpi_msgs::msg::ControlState>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_pub_;
  rclcpp::Publisher<mentorpi_msgs::msg::ChassisStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mentorpi_platform

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_platform::PlatformAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
