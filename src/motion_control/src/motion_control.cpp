#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "mentorpi_msgs/msg/control_state.hpp"
#include "mentorpi_msgs/msg/motion_restriction.hpp"
#include "mentorpi_msgs/msg/nearest_person.hpp"
#include "mentorpi_msgs/msg/obstacle_avoidance_status.hpp"
#include "motion_control/avoidance_planner.hpp"
#include "motion_control/follow_control.hpp"
#include "motion_control/ros_inputs.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motion_control {

using StatusMsg = mentorpi_msgs::msg::ObstacleAvoidanceStatus;

static_assert(kControlForbidden == mentorpi_msgs::msg::ControlState::FORBIDDEN);
static_assert(kControlManual == mentorpi_msgs::msg::ControlState::MANUAL);
static_assert(kControlAutoFollow == mentorpi_msgs::msg::ControlState::AUTO_FOLLOW);
static_assert(static_cast<uint8_t>(AvoidState::kFree) == StatusMsg::FREE);
static_assert(static_cast<uint8_t>(AvoidState::kAvoiding) == StatusMsg::AVOIDING);
static_assert(static_cast<uint8_t>(AvoidState::kBlocked) == StatusMsg::BLOCKED);
static_assert(static_cast<uint8_t>(Maneuver::kNone) == StatusMsg::MANEUVER_NONE);
static_assert(static_cast<uint8_t>(Maneuver::kArc) == StatusMsg::MANEUVER_ARC);
static_assert(static_cast<uint8_t>(Maneuver::kRotate) == StatusMsg::MANEUVER_ROTATE);
static_assert(static_cast<uint8_t>(Maneuver::kReverse) == StatusMsg::MANEUVER_REVERSE);

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

    declare_avoidance_parameters();

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    control_state_sub_ = create_subscription<mentorpi_msgs::msg::ControlState>(
        "/control/state", state_qos,
        std::bind(&MotionControlNode::on_control_state, this, std::placeholders::_1));
    nearest_sub_ = create_subscription<mentorpi_msgs::msg::NearestPerson>(
        "/perception/nearest_person", last_qos,
        std::bind(&MotionControlNode::on_nearest, this, std::placeholders::_1));
    // SD036 D5.4: a stop from obstacle_guard drops the ramp, as a closed gate does.
    restriction_sub_ = create_subscription<mentorpi_msgs::msg::MotionRestriction>(
        "/control/motion_restriction", last_qos,
        [this](const mentorpi_msgs::msg::MotionRestriction::SharedPtr msg) {
          stop_request_ = msg->stop_request;
        });

    // SD036 D5.1. Without enable_avoidance the node stays exactly SD025: no scan, no persons.
    if (enable_avoidance_) {
      scan_ =
          std::make_unique<ScanPoints>(*this, avoid_.footprint, self_filter_pad_, scan_timeout_);
      persons_ = std::make_unique<PersonPoints>(*this, persons_timeout_);
    }

    desired_twist_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("/pnc/desired_twist", last_qos);
    status_pub_ = create_publisher<StatusMsg>("/pnc/obstacle_avoidance/status", last_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&MotionControlNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
                "motion_control started (rate_hz=%.1f standoff=%.2f kp_lin=%.2f "
                "max_linear_follow=%.2f accel_linear=%.2f decel_linear=%.2f "
                "nearest_timeout_ms=%ld)",
                rate_hz, params_.standoff, params_.kp_lin, params_.max_linear_follow,
                params_.accel_linear, params_.decel_linear, nearest_timeout_ms);
    RCLCPP_INFO(get_logger(),
                "avoidance %s (footprint front=%.3f back=%.3f half_width=%.3f pad=%.3f "
                "person_radius=%.2f margin=%.2f..%.2f@%.2f influence=%.2f horizon_s=%.2f "
                "rollout_dt=%.2f samples=%dx%d max_angular_avoid=%.2f rotate=%.2f "
                "reverse=%.2f/%.2f/%.2fs hysteresis=%.2f fov=%.3f-%.2f w_goal=%.2f "
                "w_heading=%.2f w_fov=%.2f kp_repulse=%.3f w_smooth=%.2f scan_timeout_ms=%ld "
                "persons_timeout_ms=%ld)",
                enable_avoidance_ ? "on" : "off", avoid_.footprint.front, avoid_.footprint.back,
                avoid_.footprint.half_width, self_filter_pad_, person_exclusion_radius_,
                avoid_.margin_min, avoid_.margin_max, avoid_.margin_speed_ref,
                avoid_.influence_range, avoid_.horizon_s, avoid_.rollout_dt,
                avoid_.arc_linear_samples, avoid_.arc_angular_samples, avoid_.max_angular_avoid,
                avoid_.rotate_angular, avoid_.reverse_linear, avoid_.reverse_angular,
                avoid_.reverse_min_s, avoid_.free_hysteresis, avoid_.camera_half_fov,
                avoid_.fov_margin, avoid_.w_goal, avoid_.w_heading, avoid_.w_fov, avoid_.kp_repulse,
                avoid_.w_smooth, static_cast<long>(scan_timeout_.count()),
                static_cast<long>(persons_timeout_.count()));
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

  // SD036 I4.2, I4.3 and the D5.3 checks. Defaults are the starting values; the YAML wins.
  void declare_avoidance_parameters() {
    declare_parameter<bool>("enable_avoidance", true);
    enable_avoidance_ = get_parameter("enable_avoidance").as_bool();

    avoid_.footprint.front = require_positive("footprint_front", 0.175);
    avoid_.footprint.back = require_positive("footprint_back", 0.166);
    avoid_.footprint.half_width = require_positive("footprint_half_width", 0.123);
    self_filter_pad_ = require_non_negative("self_filter_pad", 0.02);
    person_exclusion_radius_ = require_non_negative("person_exclusion_radius", 0.25);
    scan_timeout_ = std::chrono::milliseconds(require_positive_int("scan_timeout_ms", 300));
    persons_timeout_ = std::chrono::milliseconds(require_positive_int("persons_timeout_ms", 1000));

    avoid_.margin_min = require_positive("margin_min", 0.05);
    avoid_.margin_max = require_positive("margin_max", 0.12);
    avoid_.margin_speed_ref = require_positive("margin_speed_ref", 0.25);
    avoid_.influence_range = require_positive("influence_range", 0.25);
    avoid_.horizon_s = require_positive("horizon_s", 1.5);
    avoid_.rollout_dt = require_positive("rollout_dt", 0.1);
    avoid_.max_angular_avoid = require_positive("max_angular_avoid", 1.0);
    avoid_.rotate_angular = require_positive("rotate_angular", 0.8);
    avoid_.reverse_linear = require_positive("reverse_linear", 0.10);
    avoid_.reverse_angular = require_non_negative("reverse_angular", 0.5);
    avoid_.reverse_min_s = require_non_negative("reverse_min_s", 0.8);
    avoid_.free_hysteresis = require_non_negative("free_hysteresis", 0.03);
    avoid_.camera_half_fov = require_positive("camera_half_fov", 0.645);
    avoid_.fov_margin = require_non_negative("fov_margin", 0.10);
    avoid_.w_goal = require_non_negative("w_goal", 1.0);
    avoid_.w_heading = require_non_negative("w_heading", 0.3);
    avoid_.w_fov = require_non_negative("w_fov", 5.0);
    avoid_.kp_repulse = require_non_negative("kp_repulse", 0.01);
    avoid_.w_smooth = require_non_negative("w_smooth", 0.2);
    avoid_.arc_linear_samples = static_cast<int>(require_positive_int("arc_linear_samples", 4));
    avoid_.arc_angular_samples = static_cast<int>(require_positive_int("arc_angular_samples", 9));

    const auto check = [](bool ok, const char* what) {
      if (!ok) {
        throw std::invalid_argument(what);
      }
    };
    check(avoid_.margin_min <= avoid_.margin_max && avoid_.margin_max <= avoid_.influence_range,
          "margin_min <= margin_max <= influence_range is required");
    check(self_filter_pad_ < avoid_.margin_min, "self_filter_pad must be < margin_min");
    check(avoid_.rotate_angular * params_.track_half_sum <= params_.max_linear,
          "rotate_angular * track_half_sum must be <= max_linear");
    check(params_.min_breakaway_linear <= avoid_.reverse_linear &&
              avoid_.reverse_linear <= params_.max_linear,
          "min_breakaway_linear <= reverse_linear <= max_linear is required");
    check(avoid_.rollout_dt <= avoid_.horizon_s, "rollout_dt must be <= horizon_s");
    check(avoid_.camera_half_fov > avoid_.fov_margin, "camera_half_fov must be > fov_margin");
    check(avoid_.arc_angular_samples % 2 == 1, "arc_angular_samples must be odd");
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

    StatusMsg status;
    status.header.stamp = now();
    status.header.frame_id = kBaseFrame;
    status.maneuver = StatusMsg::MANEUVER_NONE;
    status.min_clearance = -1.0f;
    status.bumper_margin = 0.0f;

    // Obstacle points with people excluded, whenever the scan is usable: the status then shows
    // clearance in Manual too, which is how the stand check reads it.
    const bool scan_usable = scan_ && scan_->fresh();
    std::vector<Point2d> points;
    if (scan_usable) {
      points = scan_->points();
      const std::size_t excluded = persons_->exclude(points, person_exclusion_radius_);
      status.obstacle_points = static_cast<uint16_t>(std::min<std::size_t>(points.size(), 65535));
      status.excluded_points = static_cast<uint16_t>(std::min<std::size_t>(excluded, 65535));
      const double clearance = min_clearance(avoid_.footprint, Pose2d{0.0, 0.0, 0.0}, points);
      status.min_clearance = std::isfinite(clearance) ? static_cast<float>(clearance) : -1.0f;
    }

    geometry_msgs::msg::Twist twist{};
    const bool gate_ok = have_control_state_ && control_state_ == kControlAutoFollow &&
                         nearest_valid_ && !nearest_coasting_ && nearest_fresh;
    if (!gate_ok) {
      // D4 of SD025: the gate wins over the ramp. Zeros go out on this very tick, and the ramp
      // state is dropped so returning to follow starts from a standstill.
      reset_avoidance(memory_);
      status.state = StatusMsg::INACTIVE;
    } else {
      if (stop_request_) {
        reset_avoidance(memory_);  // SD036 D5.4
      }
      Twist2d cmd{0.0, 0.0};
      if (!enable_avoidance_ || !scan_usable) {
        cmd = compute_follow_twist(nearest_x_, nearest_y_, memory_.prev_linear, dt_, params_);
        memory_.prev_linear = cmd.linear_x;
        status.state = enable_avoidance_ ? StatusMsg::NO_SCAN : StatusMsg::DISABLED;
      } else {
        const AvoidanceResult r =
            plan_avoidance(nearest_x_, nearest_y_, points, dt_, params_, avoid_, memory_);
        cmd = r.cmd;
        status.state = static_cast<uint8_t>(r.state);
        status.maneuver = static_cast<uint8_t>(r.maneuver);
        status.bumper_margin = static_cast<float>(r.bumper_margin);
      }
      twist.linear.x = cmd.linear_x;
      twist.angular.z = cmd.angular_z;
    }
    desired_twist_pub_->publish(twist);
    status_pub_->publish(status);
  }

  FollowControlParams params_{};
  AvoidanceParams avoid_{};
  AvoidanceMemory memory_{};
  bool enable_avoidance_{true};
  double self_filter_pad_{0.02};
  double person_exclusion_radius_{0.25};
  std::chrono::milliseconds scan_timeout_{300};
  std::chrono::milliseconds persons_timeout_{1000};
  double dt_{0.05};
  std::chrono::milliseconds nearest_timeout_{1000};
  bool have_control_state_{false};
  uint8_t control_state_{kControlForbidden};
  bool have_nearest_{false};
  bool nearest_valid_{false};
  bool nearest_coasting_{false};
  double nearest_x_{0.0};
  double nearest_y_{0.0};
  std::chrono::steady_clock::time_point last_nearest_stamp_{};
  bool stop_request_{false};

  std::unique_ptr<ScanPoints> scan_;
  std::unique_ptr<PersonPoints> persons_;
  rclcpp::Subscription<mentorpi_msgs::msg::ControlState>::SharedPtr control_state_sub_;
  rclcpp::Subscription<mentorpi_msgs::msg::NearestPerson>::SharedPtr nearest_sub_;
  rclcpp::Subscription<mentorpi_msgs::msg::MotionRestriction>::SharedPtr restriction_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desired_twist_pub_;
  rclcpp::Publisher<StatusMsg>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motion_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motion_control::MotionControlNode>());
  rclcpp::shutdown();
  return 0;
}
