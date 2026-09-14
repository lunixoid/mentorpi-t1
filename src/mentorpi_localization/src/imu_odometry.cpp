#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mentorpi_localization {

namespace {

constexpr double kImuJointRoll = M_PI;
constexpr double kImuJointPitch = 0.0;
constexpr double kImuJointYaw = -M_PI / 2.0;

bool is_finite_vector3(const geometry_msgs::msg::Vector3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool is_valid_quaternion(const geometry_msgs::msg::Quaternion& q) {
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
    return false;
  }
  const double norm_sq = (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
  return norm_sq > 1e-24;
}

geometry_msgs::msg::Quaternion multiply_quaternions(const geometry_msgs::msg::Quaternion& left,
                                                    const geometry_msgs::msg::Quaternion& right) {
  tf2::Quaternion q_left;
  tf2::Quaternion q_right;
  tf2::fromMsg(left, q_left);
  tf2::fromMsg(right, q_right);
  q_left.normalize();
  q_right.normalize();
  const tf2::Quaternion product = q_left * q_right;
  return tf2::toMsg(product);
}

geometry_msgs::msg::Vector3 rotate_vector(const geometry_msgs::msg::Quaternion& rotation,
                                          const geometry_msgs::msg::Vector3& vector) {
  tf2::Quaternion q_rotation;
  tf2::fromMsg(rotation, q_rotation);
  q_rotation.normalize();
  const tf2::Vector3 rotated =
      tf2::quatRotate(q_rotation, tf2::Vector3(vector.x, vector.y, vector.z));
  geometry_msgs::msg::Vector3 result;
  result.x = rotated.x();
  result.y = rotated.y();
  result.z = rotated.z();
  return result;
}

geometry_msgs::msg::Quaternion default_imu_to_base_quaternion() {
  tf2::Quaternion base_to_imu;
  base_to_imu.setRPY(kImuJointRoll, kImuJointPitch, kImuJointYaw);
  base_to_imu.normalize();
  const tf2::Quaternion imu_to_base = base_to_imu.inverse();
  return tf2::toMsg(imu_to_base);
}

geometry_msgs::msg::Quaternion load_imu_to_base_quaternion(rclcpp::Node& node) {
  const auto default_q = default_imu_to_base_quaternion();
  node.declare_parameter<double>("imu_to_base_qx", default_q.x);
  node.declare_parameter<double>("imu_to_base_qy", default_q.y);
  node.declare_parameter<double>("imu_to_base_qz", default_q.z);
  node.declare_parameter<double>("imu_to_base_qw", default_q.w);

  geometry_msgs::msg::Quaternion q;
  q.x = node.get_parameter("imu_to_base_qx").as_double();
  q.y = node.get_parameter("imu_to_base_qy").as_double();
  q.z = node.get_parameter("imu_to_base_qz").as_double();
  q.w = node.get_parameter("imu_to_base_qw").as_double();
  if (!is_valid_quaternion(q)) {
    RCLCPP_WARN(node.get_logger(),
                "imu_to_base quaternion parameter is invalid; using URDF default");
    return default_q;
  }
  return q;
}

}  // namespace

class ImuOdometryNode : public rclcpp::Node {
 public:
  ImuOdometryNode() : Node("imu_odometry") {
    const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/imu_odom");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "imu_odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
    imu_to_base_q_ = load_imu_to_base_quaternion(*this);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, sensor_qos, std::bind(&ImuOdometryNode::on_imu, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, sensor_qos);

    RCLCPP_INFO(get_logger(), "imu_odometry started (imu=%s odom=%s frames=%s->%s)",
                imu_topic.c_str(), odom_topic.c_str(), odom_frame_id_.c_str(),
                base_frame_id_.c_str());
  }

 private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!is_valid_quaternion(msg->orientation)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "dropping IMU message: invalid orientation quaternion");
      return;
    }
    if (!is_finite_vector3(msg->angular_velocity)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "dropping IMU message: non-finite angular_velocity");
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = multiply_quaternions(msg->orientation, imu_to_base_q_);

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular = rotate_vector(imu_to_base_q_, msg->angular_velocity);

    odom_pub_->publish(odom);
  }

  std::string odom_frame_id_;
  std::string base_frame_id_;
  geometry_msgs::msg::Quaternion imu_to_base_q_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace mentorpi_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_localization::ImuOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
