#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mentorpi_control/joy_freshness.hpp"
#include "mentorpi_control/pad_command.hpp"
#include "mentorpi_control/publish_cadence.hpp"
#include "mentorpi_msgs/msg/control_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

namespace mentorpi_control {

class PadTeleopNode : public rclcpp::Node {
 public:
  PadTeleopNode() : Node("pad_teleop") {
    declare_parameter<int64_t>("mode_button", 10);
    declare_parameter<int64_t>("mode_button_linux", 8);
    declare_parameter<int64_t>("linear_axis", 1);
    declare_parameter<int64_t>("angular_axis", 2);
    declare_parameter<double>("max_linear", 0.5);
    declare_parameter<double>("max_angular", 2.0);
    declare_parameter<double>("linear_enter", 0.10);
    declare_parameter<double>("linear_release", 0.06);
    declare_parameter<double>("linear_min", 0.10);
    declare_parameter<double>("angular_enter", 0.10);
    declare_parameter<double>("angular_release", 0.06);
    declare_parameter<double>("angular_min", 0.40);
    declare_parameter<int64_t>("joy_timeout_ms", 1000);
    declare_parameter<int64_t>("cmd_freshness_ms", 100);
    declare_parameter<double>("rate_hz", 20.0);

    mode_button_ = get_parameter("mode_button").as_int();
    mode_button_linux_ = get_parameter("mode_button_linux").as_int();
    linear_axis_ = get_parameter("linear_axis").as_int();
    angular_axis_ = get_parameter("angular_axis").as_int();
    if (mode_button_ < 0 || mode_button_linux_ < 0 || linear_axis_ < 0 || angular_axis_ < 0) {
      throw std::invalid_argument("mode_button, mode_button_linux and axes must be >= 0");
    }

    shape_params_.linear.max_out = get_parameter("max_linear").as_double();
    shape_params_.angular.max_out = get_parameter("max_angular").as_double();
    if (!(shape_params_.linear.max_out > 0.0) || !(shape_params_.angular.max_out > 0.0)) {
      throw std::invalid_argument("max_linear and max_angular must be > 0");
    }

    auto load_axis = [this](const char* prefix, AxisShapeParams& axis) {
      axis.enter = get_parameter(std::string(prefix) + "_enter").as_double();
      axis.release = get_parameter(std::string(prefix) + "_release").as_double();
      axis.min_out = get_parameter(std::string(prefix) + "_min").as_double();
      if (axis.enter < 0.0 || !(axis.enter < 1.0)) {
        throw std::invalid_argument(std::string(prefix) + "_enter must be in [0, 1)");
      }
      if (axis.release < 0.0 || axis.release > axis.enter) {
        throw std::invalid_argument(std::string(prefix) + "_release must be in [0, enter]");
      }
      if (axis.min_out < 0.0 || !(axis.min_out < axis.max_out)) {
        throw std::invalid_argument(std::string(prefix) + "_min must be in [0, max)");
      }
    };
    load_axis("linear", shape_params_.linear);
    load_axis("angular", shape_params_.angular);

    const int64_t joy_timeout_ms = get_parameter("joy_timeout_ms").as_int();
    if (joy_timeout_ms <= 0) {
      throw std::invalid_argument("joy_timeout_ms must be > 0");
    }
    joy_timeout_ = std::chrono::milliseconds(joy_timeout_ms);

    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }
    publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));

    int64_t cmd_freshness_ms = get_parameter("cmd_freshness_ms").as_int();
    if (cmd_freshness_ms <= 0) {
      cmd_freshness_ms = default_cmd_freshness_ms(rate_hz);
    }
    if (cmd_freshness_ms >= joy_timeout_ms) {
      throw std::invalid_argument("cmd_freshness_ms must be > 0 and < joy_timeout_ms");
    }
    cmd_freshness_ = std::chrono::milliseconds(cmd_freshness_ms);

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    rrc_joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/ros_robot_controller/joy", last_qos,
        std::bind(&PadTeleopNode::on_rrc_joy, this, std::placeholders::_1));
    linux_joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", last_qos, std::bind(&PadTeleopNode::on_linux_joy, this, std::placeholders::_1));
    state_sub_ = create_subscription<mentorpi_msgs::msg::ControlState>(
        "/control/state", state_qos,
        std::bind(&PadTeleopNode::on_state, this, std::placeholders::_1));

    toggle_pub_ = create_publisher<std_msgs::msg::Empty>("/control/mode_toggle", last_qos);
    remote_pub_ = create_publisher<std_msgs::msg::Bool>("/control/remote_controller", last_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&PadTeleopNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
                "pad_teleop started (rate_hz=%.1f cmd_freshness_ms=%ld "
                "joy_timeout_ms=%ld mode_button=%ld mode_button_linux=%ld "
                "linear enter=%.3f release=%.3f min=%.3f max=%.3f "
                "angular enter=%.3f release=%.3f min=%.3f max=%.3f)",
                rate_hz, cmd_freshness_ms, joy_timeout_ms, mode_button_, mode_button_linux_,
                shape_params_.linear.enter, shape_params_.linear.release,
                shape_params_.linear.min_out, shape_params_.linear.max_out,
                shape_params_.angular.enter, shape_params_.angular.release,
                shape_params_.angular.min_out, shape_params_.angular.max_out);
  }

 private:
  bool command_fresh(std::chrono::steady_clock::time_point now) const {
    return is_fresh(have_joy_, last_joy_stamp_, now, cmd_freshness_);
  }

  bool remote_fresh(std::chrono::steady_clock::time_point now) const {
    return is_fresh(have_joy_, last_joy_stamp_, now, joy_timeout_);
  }

  PadCommand decide_now(std::chrono::steady_clock::time_point now) {
    return decide_pad(static_cast<bool>(cmd_pub_), command_fresh(now), remote_fresh(now),
                      last_joy_.axes, linear_axis_, angular_axis_, shape_params_, shape_state_);
  }

  void emit_cmd(const PadCommand& cmd, bool force) {
    if (!cmd_pub_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!force && !timer_keepalive_due(ever_cmd_sent_, now, last_cmd_send_, publish_period_)) {
      return;
    }
    geometry_msgs::msg::Twist twist{};
    twist.linear.x = cmd.twist.linear_x;
    twist.angular.z = cmd.twist.angular_z;
    cmd_pub_->publish(twist);
    ever_cmd_sent_ = true;
    last_cmd_send_ = now;
  }

  void set_manual(bool manual) {
    if (manual) {
      if (!cmd_pub_) {
        const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        // Mux consumes this; pad_teleop must not publish /vehicle/cmd_vel.
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/control/manual_cmd_vel", last_qos);
      }
      emit_cmd(decide_now(std::chrono::steady_clock::now()), true);
      return;
    }
    shape_state_.reset();
    cmd_pub_.reset();
  }

  void ingest_joy(const sensor_msgs::msg::Joy::SharedPtr msg, int64_t mode_button,
                  int32_t& last_mode_button) {
    last_joy_ = *msg;
    last_joy_stamp_ = std::chrono::steady_clock::now();
    have_joy_ = true;

    if (static_cast<size_t>(mode_button) < msg->buttons.size()) {
      const int32_t pressed = msg->buttons[static_cast<size_t>(mode_button)];
      if (last_mode_button == 0 && pressed != 0) {
        toggle_pub_->publish(std_msgs::msg::Empty());
      }
      last_mode_button = pressed;
    }

    emit_cmd(decide_now(last_joy_stamp_), true);
  }

  void on_rrc_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    ingest_joy(msg, mode_button_, last_mode_button_rrc_);
  }

  void on_linux_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    ingest_joy(msg, mode_button_linux_, last_mode_button_linux_);
  }

  void on_state(const mentorpi_msgs::msg::ControlState::SharedPtr msg) {
    set_manual(msg->state == mentorpi_msgs::msg::ControlState::MANUAL);
  }

  void on_timer() {
    const auto now = std::chrono::steady_clock::now();
    const PadCommand cmd = decide_now(now);

    std_msgs::msg::Bool remote;
    remote.data = cmd.remote;
    remote_pub_->publish(remote);

    emit_cmd(cmd, false);
  }

  int64_t mode_button_{10};
  int64_t mode_button_linux_{8};
  int64_t linear_axis_{1};
  int64_t angular_axis_{2};
  JoyTwistParams shape_params_{};
  JoyShapeState shape_state_{};
  std::chrono::milliseconds joy_timeout_{1000};
  std::chrono::milliseconds cmd_freshness_{100};
  std::chrono::nanoseconds publish_period_{};
  std::chrono::steady_clock::time_point last_cmd_send_{};
  bool ever_cmd_sent_{false};

  sensor_msgs::msg::Joy last_joy_{};
  std::chrono::steady_clock::time_point last_joy_stamp_{};
  bool have_joy_{false};
  int32_t last_mode_button_rrc_{0};
  int32_t last_mode_button_linux_{0};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rrc_joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr linux_joy_sub_;
  rclcpp::Subscription<mentorpi_msgs::msg::ControlState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr toggle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remote_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mentorpi_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_control::PadTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
