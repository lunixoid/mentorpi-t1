#ifndef MOTION_CONTROL_ROS_INPUTS_HPP_
#define MOTION_CONTROL_ROS_INPUTS_HPP_

// SD036 D2 and I3 on the ROS side, shared by motion_control and obstacle_guard: the scan turned
// into base_footprint points through TF, and the person list for exclusion. Both nodes read the
// same topics through the same code, which is what keeps D6.2 true in the graph.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "mentorpi_msgs/msg/person_array.hpp"
#include "motion_control/obstacle_points.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace motion_control {

inline constexpr const char* kBaseFrame = "base_footprint";

// Latest scan as self-filtered points in base_footprint. The lidar pose comes from TF on every
// scan (a static transform, cheap); without it the scan does not count as fresh.
class ScanPoints {
 public:
  ScanPoints(rclcpp::Node& node, const Footprint& footprint, double self_filter_pad,
             std::chrono::milliseconds timeout)
      : node_(node), footprint_(footprint), pad_(self_filter_pad), timeout_(timeout) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    sub_ = node.create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { on_scan(*msg); });
  }

  bool fresh() const { return have_ && std::chrono::steady_clock::now() - stamp_ <= timeout_; }

  const std::vector<Point2d>& points() const { return points_; }

 private:
  void on_scan(const sensor_msgs::msg::LaserScan& msg) {
    Pose2d lidar{0.0, 0.0, 0.0};
    try {
      const auto tf =
          tf_buffer_->lookupTransform(kBaseFrame, msg.header.frame_id, tf2::TimePointZero);
      const auto& q = tf.transform.rotation;
      lidar.x = tf.transform.translation.x;
      lidar.y = tf.transform.translation.y;
      lidar.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                           "no TF %s <- %s, scan ignored: %s", kBaseFrame,
                           msg.header.frame_id.c_str(), ex.what());
      return;
    }
    points_ = scan_to_base_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                  msg.range_max, lidar);
    drop_self_points(points_, footprint_, pad_);
    stamp_ = std::chrono::steady_clock::now();
    have_ = true;
  }

  rclcpp::Node& node_;
  Footprint footprint_;
  double pad_;
  std::chrono::milliseconds timeout_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::vector<Point2d> points_;
  std::chrono::steady_clock::time_point stamp_{};
  bool have_{false};
};

// People from /perception/persons. PersonHypothesis x, y are already in base_footprint (F08).
class PersonPoints {
 public:
  PersonPoints(rclcpp::Node& node, std::chrono::milliseconds timeout) : timeout_(timeout) {
    sub_ = node.create_subscription<mentorpi_msgs::msg::PersonArray>(
        "/perception/persons", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const mentorpi_msgs::msg::PersonArray::SharedPtr msg) {
          persons_.clear();
          for (const auto& p : msg->persons) {
            persons_.push_back(Point2d{static_cast<double>(p.x), static_cast<double>(p.y)});
          }
          stamp_ = std::chrono::steady_clock::now();
          have_ = true;
        });
  }

  // D2.3: a stale list excludes nothing.
  std::size_t exclude(std::vector<Point2d>& points, double radius) const {
    const bool fresh = have_ && std::chrono::steady_clock::now() - stamp_ <= timeout_;
    return fresh ? exclude_person_points(points, persons_, radius) : 0;
  }

 private:
  std::chrono::milliseconds timeout_;
  rclcpp::Subscription<mentorpi_msgs::msg::PersonArray>::SharedPtr sub_;
  std::vector<Point2d> persons_;
  std::chrono::steady_clock::time_point stamp_{};
  bool have_{false};
};

}  // namespace motion_control

#endif  // MOTION_CONTROL_ROS_INPUTS_HPP_
