#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_msgs/msg/follow_person_status.hpp"
#include "mentorpi_msgs/msg/nearest_person.hpp"
#include "mission_control/follow_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mission_control {

static_assert(kFollowInactive == mentorpi_msgs::msg::FollowPersonStatus::INACTIVE);
static_assert(kFollowHold == mentorpi_msgs::msg::FollowPersonStatus::HOLD);
static_assert(kFollowFollowing == mentorpi_msgs::msg::FollowPersonStatus::FOLLOWING);

static_assert(kControlForbidden == mentorpi_msgs::msg::ControlState::FORBIDDEN);
static_assert(kControlManual == mentorpi_msgs::msg::ControlState::MANUAL);
static_assert(kControlAutoFollow == mentorpi_msgs::msg::ControlState::AUTO_FOLLOW);

class MissionControlNode : public rclcpp::Node {
 public:
  MissionControlNode() : Node("mission_control") {
    declare_parameter<int64_t>("nearest_timeout_ms", 1000);
    declare_parameter<double>("rate_hz", 10.0);

    const int64_t nearest_timeout_ms = get_parameter("nearest_timeout_ms").as_int();
    if (nearest_timeout_ms <= 0) {
      throw std::invalid_argument("nearest_timeout_ms must be > 0");
    }
    nearest_timeout_ = std::chrono::milliseconds(nearest_timeout_ms);

    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto nearest_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    control_state_sub_ = create_subscription<mentorpi_msgs::msg::ControlState>(
        "/control/state", state_qos,
        std::bind(&MissionControlNode::on_control_state, this, std::placeholders::_1));
    nearest_sub_ = create_subscription<mentorpi_msgs::msg::NearestPerson>(
        "/perception/nearest_person", nearest_qos,
        std::bind(&MissionControlNode::on_nearest, this, std::placeholders::_1));

    status_pub_ = create_publisher<mentorpi_msgs::msg::FollowPersonStatus>(
        "/pnc/follow_person/status", state_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&MissionControlNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "mission_control started (rate_hz=%.1f nearest_timeout_ms=%ld)",
                rate_hz, nearest_timeout_ms);
  }

 private:
  void on_control_state(const mentorpi_msgs::msg::ControlState::SharedPtr msg) {
    have_control_state_ = true;
    control_state_ = msg->state;
  }

  void on_nearest(const mentorpi_msgs::msg::NearestPerson::SharedPtr msg) {
    have_nearest_ = true;
    nearest_valid_ = msg->valid;
    last_nearest_stamp_ = std::chrono::steady_clock::now();
  }

  void on_timer() {
    const bool nearest_fresh =
        have_nearest_ &&
        (std::chrono::steady_clock::now() - last_nearest_stamp_ <= nearest_timeout_);

    const uint8_t state = evaluate_follow_behavior(have_control_state_, control_state_,
                                                   nearest_valid_, nearest_fresh);

    mentorpi_msgs::msg::FollowPersonStatus status;
    status.state = state;
    status_pub_->publish(status);
  }

  std::chrono::milliseconds nearest_timeout_{1000};
  bool have_control_state_{false};
  uint8_t control_state_{kControlForbidden};
  bool have_nearest_{false};
  bool nearest_valid_{false};
  std::chrono::steady_clock::time_point last_nearest_stamp_{};

  rclcpp::Subscription<mentorpi_msgs::msg::ControlState>::SharedPtr control_state_sub_;
  rclcpp::Subscription<mentorpi_msgs::msg::NearestPerson>::SharedPtr nearest_sub_;
  rclcpp::Publisher<mentorpi_msgs::msg::FollowPersonStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mission_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_control::MissionControlNode>());
  rclcpp::shutdown();
  return 0;
}
