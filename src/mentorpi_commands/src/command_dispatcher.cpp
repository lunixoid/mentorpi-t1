#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "mentorpi_commands/command_table.hpp"
#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_msgs/msg/named_command.hpp"
#include "mentorpi_msgs/srv/set_control_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mentorpi_commands {

static_assert(kControlForbidden == mentorpi_msgs::msg::ControlState::FORBIDDEN);
static_assert(kControlManual == mentorpi_msgs::msg::ControlState::MANUAL);
static_assert(kControlAutoFollow == mentorpi_msgs::msg::ControlState::AUTO_FOLLOW);

class CommandDispatcherNode : public rclcpp::Node {
 public:
  CommandDispatcherNode() : Node("command_dispatcher") {
    declare_parameter<int64_t>("set_mode_timeout_ms", 1000);

    const int64_t set_mode_timeout_ms = get_parameter("set_mode_timeout_ms").as_int();
    if (set_mode_timeout_ms <= 0) {
      throw std::invalid_argument("set_mode_timeout_ms must be > 0");
    }
    set_mode_timeout_ = std::chrono::milliseconds(set_mode_timeout_ms);

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    command_sub_ = create_subscription<mentorpi_msgs::msg::NamedCommand>(
        "/commands/named", command_qos,
        std::bind(&CommandDispatcherNode::on_named_command, this, std::placeholders::_1));
    set_mode_client_ = create_client<mentorpi_msgs::srv::SetControlMode>("/control/set_mode");

    timeout_timer_ = create_wall_timer(
        set_mode_timeout_, std::bind(&CommandDispatcherNode::on_set_mode_timeout, this));
    timeout_timer_->cancel();

    RCLCPP_INFO(get_logger(), "command_dispatcher started (set_mode_timeout_ms=%ld)",
                set_mode_timeout_ms);
  }

 private:
  using SetMode = mentorpi_msgs::srv::SetControlMode;

  void on_named_command(const mentorpi_msgs::msg::NamedCommand::SharedPtr msg) {
    const auto target = control_mode_for_command(msg->name);
    if (!target.has_value()) {
      RCLCPP_WARN(get_logger(), "command rejected name=%s source=%s reason=unknown command",
                  msg->name.c_str(), msg->source.c_str());
      return;
    }
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "command rejected name=%s source=%s reason=set_mode unavailable",
                  msg->name.c_str(), msg->source.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "command name=%s source=%s -> set_mode target=%u", msg->name.c_str(),
                msg->source.c_str(), static_cast<unsigned>(*target));

    auto request = std::make_shared<SetMode::Request>();
    request->target_state = *target;

    const uint64_t id = ++call_seq_;
    pending_id_ = id;
    pending_name_ = msg->name;
    pending_source_ = msg->source;
    timeout_timer_->reset();

    set_mode_client_->async_send_request(request,
                                         [this, id](rclcpp::Client<SetMode>::SharedFuture future) {
                                           on_set_mode_done(id, std::move(future));
                                         });
  }

  void on_set_mode_done(uint64_t id, rclcpp::Client<SetMode>::SharedFuture future) {
    if (id != pending_id_) {
      return;
    }
    timeout_timer_->cancel();
    pending_id_ = 0;
    const auto response = future.get();
    RCLCPP_INFO(get_logger(), "command name=%s source=%s done success=%d active_state=%u",
                pending_name_.c_str(), pending_source_.c_str(), static_cast<int>(response->success),
                static_cast<unsigned>(response->active_state));
  }

  void on_set_mode_timeout() {
    timeout_timer_->cancel();
    if (pending_id_ == 0) {
      return;
    }
    pending_id_ = 0;
    RCLCPP_WARN(get_logger(), "command rejected name=%s source=%s reason=set_mode timeout",
                pending_name_.c_str(), pending_source_.c_str());
  }

  std::chrono::milliseconds set_mode_timeout_{1000};
  uint64_t call_seq_{0};
  uint64_t pending_id_{0};
  std::string pending_name_;
  std::string pending_source_;

  rclcpp::Subscription<mentorpi_msgs::msg::NamedCommand>::SharedPtr command_sub_;
  rclcpp::Client<SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace mentorpi_commands

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_commands::CommandDispatcherNode>());
  rclcpp::shutdown();
  return 0;
}
