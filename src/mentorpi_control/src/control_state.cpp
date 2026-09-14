#include "mentorpi_msgs/msg/control_state.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "mentorpi_control/control_mode.hpp"
#include "mentorpi_msgs/msg/control_status.hpp"
#include "mentorpi_msgs/srv/set_control_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

namespace mentorpi_control {

static_assert(kControlForbidden == mentorpi_msgs::msg::ControlState::FORBIDDEN);
static_assert(kControlManual == mentorpi_msgs::msg::ControlState::MANUAL);
static_assert(kControlAutoFollow == mentorpi_msgs::msg::ControlState::AUTO_FOLLOW);

class ControlStateNode : public rclcpp::Node {
 public:
  ControlStateNode() : Node("control_state") {
    declare_parameter<int64_t>("remote_timeout_ms", 1000);
    declare_parameter<double>("rate_hz", 10.0);

    const int64_t remote_timeout_ms = get_parameter("remote_timeout_ms").as_int();
    if (remote_timeout_ms <= 0) {
      throw std::invalid_argument("remote_timeout_ms must be > 0");
    }
    remote_timeout_ = std::chrono::milliseconds(remote_timeout_ms);

    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    // Latch last /control/status for a late t1ctl subscriber. Volatile
    // subscribers remain compatible (durability offered is stronger).
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    toggle_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/control/mode_toggle", last_qos,
        std::bind(&ControlStateNode::on_toggle, this, std::placeholders::_1));
    remote_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/control/remote_controller", last_qos,
        std::bind(&ControlStateNode::on_remote, this, std::placeholders::_1));

    state_pub_ = create_publisher<mentorpi_msgs::msg::ControlState>("/control/state", state_qos);
    status_pub_ =
        create_publisher<mentorpi_msgs::msg::ControlStatus>("/control/status", status_qos);
    set_mode_srv_ = create_service<mentorpi_msgs::srv::SetControlMode>(
        "/control/set_mode", std::bind(&ControlStateNode::on_set_mode, this, std::placeholders::_1,
                                       std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&ControlStateNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "control_state started (rate_hz=%.1f remote_timeout_ms=%ld)", rate_hz,
                remote_timeout_ms);
  }

 private:
  void on_toggle(const std_msgs::msg::Empty::SharedPtr) { apply_mode_toggle(mode_); }

  void on_set_mode(const std::shared_ptr<mentorpi_msgs::srv::SetControlMode::Request> request,
                   std::shared_ptr<mentorpi_msgs::srv::SetControlMode::Response> response) {
    const SetControlModeResult result = apply_set_control_mode(mode_, request->target_state);
    response->success = result.success;
    response->active_state = result.active_state;
    response->reason = result.reason;
    if (!result.success) {
      RCLCPP_WARN(get_logger(), "set_mode rejected target_state=%u active_state=%u",
                  request->target_state, result.active_state);
    }
  }

  void on_remote(const std_msgs::msg::Bool::SharedPtr msg) {
    last_remote_ = msg->data;
    last_remote_stamp_ = std::chrono::steady_clock::now();
    have_remote_ = true;
  }

  void on_timer() {
    const bool remote_fresh =
        have_remote_ && (std::chrono::steady_clock::now() - last_remote_stamp_ <= remote_timeout_);
    const bool remote_controller = remote_fresh && last_remote_;

    mentorpi_msgs::msg::ControlState state;
    state.state = mode_.state;
    state_pub_->publish(state);

    mentorpi_msgs::msg::ControlStatus status;
    status.state = mode_.state;
    status.remote_controller = remote_controller;
    status.reason = control_status_reason(mode_);
    status_pub_->publish(status);
  }

  ControlModeState mode_{};
  std::chrono::milliseconds remote_timeout_{1000};
  bool last_remote_{false};
  std::chrono::steady_clock::time_point last_remote_stamp_{};
  bool have_remote_{false};

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr toggle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr remote_sub_;
  rclcpp::Publisher<mentorpi_msgs::msg::ControlState>::SharedPtr state_pub_;
  rclcpp::Publisher<mentorpi_msgs::msg::ControlStatus>::SharedPtr status_pub_;
  rclcpp::Service<mentorpi_msgs::srv::SetControlMode>::SharedPtr set_mode_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mentorpi_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_control::ControlStateNode>());
  rclcpp::shutdown();
  return 0;
}
