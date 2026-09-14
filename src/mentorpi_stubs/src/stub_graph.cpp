#include <chrono>
#include <memory>
#include <stdexcept>

#include "mentorpi_msgs/msg/motion_restriction.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mentorpi_stubs {

class StubGraphNode : public rclcpp::Node {
 public:
  StubGraphNode() : Node("stub_graph") {
    declare_parameter<double>("rate_hz", 10.0);
    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }

    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // /pnc/desired_twist comes from motion_control (F11). Restriction stays a stub until F12/F13.
    restriction_pub_ = create_publisher<mentorpi_msgs::msg::MotionRestriction>(
        "/control/motion_restriction", last_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&StubGraphNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "stub_graph started (rate_hz=%.1f)", rate_hz);
  }

 private:
  void on_timer() {
    mentorpi_msgs::msg::MotionRestriction restriction;
    restriction.stop_request = false;
    restriction.reason.clear();
    restriction_pub_->publish(restriction);
  }

  rclcpp::Publisher<mentorpi_msgs::msg::MotionRestriction>::SharedPtr restriction_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mentorpi_stubs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_stubs::StubGraphNode>());
  rclcpp::shutdown();
  return 0;
}
