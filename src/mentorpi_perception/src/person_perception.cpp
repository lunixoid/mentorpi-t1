#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "mentorpi_msgs/msg/nearest_person.hpp"
#include "mentorpi_msgs/msg/person_array.hpp"
#include "mentorpi_msgs/msg/person_hypothesis.hpp"
#include "mentorpi_perception/person_geometry.hpp"
#include "mentorpi_perception/person_target_lock.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace mentorpi_perception {

namespace {

constexpr const char* kPersonClass = "person";
constexpr const char* kOverlayTopic = "/perception/persons/overlay";
constexpr uint8_t kFloat32 = sensor_msgs::msg::PointField::FLOAT32;
constexpr int64_t kPersonsLatencyBudgetNs = 100 * 1000 * 1000;

bool is_fresh(bool have, std::chrono::steady_clock::time_point last,
              std::chrono::steady_clock::time_point now, std::chrono::milliseconds timeout) {
  return have && (now - last <= timeout);
}

// /aurora/points2 is a compacted list of valid points, not an organized grid
// (SD022 D6.1). Only the flat xyz layout is parsed; pixels are matched by
// projection, never by slot index.
bool parse_cloud_xyz(const sensor_msgs::msg::PointCloud2& msg, CloudXyzView* view) {
  if (view == nullptr) {
    return false;
  }
  if (msg.is_bigendian) {
    return false;
  }
  if (msg.point_step == 0 || msg.width == 0) {
    return false;
  }
  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  for (const auto& field : msg.fields) {
    if (field.datatype != kFloat32) {
      continue;
    }
    if (field.name == "x") {
      offset_x = static_cast<int>(field.offset);
    } else if (field.name == "y") {
      offset_y = static_cast<int>(field.offset);
    } else if (field.name == "z") {
      offset_z = static_cast<int>(field.offset);
    }
  }
  if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
    return false;
  }
  const std::size_t n_points =
      static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(std::max(msg.height, 1u));
  view->data = msg.data.data();
  view->data_bytes = msg.data.size();
  view->point_count = n_points;
  view->point_step = static_cast<int>(msg.point_step);
  view->offset_x = offset_x;
  view->offset_y = offset_y;
  view->offset_z = offset_z;
  return true;
}

CameraIntrinsics intrinsics_from_info(const sensor_msgs::msg::CameraInfo& msg) {
  CameraIntrinsics cam;
  cam.fx = msg.k[0];
  cam.fy = msg.k[4];
  cam.cx = msg.k[2];
  cam.cy = msg.k[5];
  cam.width = static_cast<int>(msg.width);
  cam.height = static_cast<int>(msg.height);
  return cam;
}

double bbox_center_x(const vision_msgs::msg::Detection2D& det) {
  return det.bbox.center.position.x;
}

double bbox_center_y(const vision_msgs::msg::Detection2D& det) {
  return det.bbox.center.position.y;
}

double bbox_center_u(const BboxPx& box) { return (box.u_min + box.u_max) * 0.5; }

bool header_stamp_is_zero(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec == 0 && stamp.nanosec == 0;
}

bool is_person(const vision_msgs::msg::Detection2D& det) {
  if (det.results.empty()) {
    return false;
  }
  return det.results.front().hypothesis.class_id == kPersonClass;
}

float person_score(const vision_msgs::msg::Detection2D& det) {
  if (det.results.empty()) {
    return 0.0f;
  }
  return static_cast<float>(det.results.front().hypothesis.score);
}

bool parse_detection_track_id(const std::string& id, std::int32_t* out) {
  if (out == nullptr || id.empty()) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(id.c_str(), &end, 10);
  if (errno != 0 || end == id.c_str() || *end != '\0') {
    return false;
  }
  if (v < std::numeric_limits<std::int32_t>::min() ||
      v > std::numeric_limits<std::int32_t>::max()) {
    return false;
  }
  *out = static_cast<std::int32_t>(v);
  return true;
}

mentorpi_msgs::msg::PersonHypothesis observation_to_hypothesis(const PersonObservation& obs) {
  mentorpi_msgs::msg::PersonHypothesis hyp;
  hyp.track_id = obs.track_id;
  hyp.x = obs.x;
  hyp.y = obs.y;
  hyp.range = obs.range;
  hyp.confidence = obs.confidence;
  return hyp;
}

// Overlay image has no iputils-ping. SOCK_DGRAM ICMP is the unprivileged
// equivalent of `ping -c 1 -W 1` (kernel fills checksum; ping_group_range
// on the stand is 0..max).
int ping_once(const std::string& host) {
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  if (::inet_pton(AF_INET, host.c_str(), &dst.sin_addr) != 1) {
    return 1;
  }

  const int fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_ICMP);
  if (fd < 0) {
    return 1;
  }

  uint8_t req[8] = {};
  req[0] = 8;
  static std::atomic<uint16_t> seq{1};
  const uint16_t n = seq.fetch_add(1);
  req[6] = static_cast<uint8_t>(n >> 8);
  req[7] = static_cast<uint8_t>(n & 0xff);

  if (::sendto(fd, req, sizeof(req), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) < 0) {
    ::close(fd);
    return 1;
  }

  pollfd pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;
  int pr = 0;
  do {
    pr = ::poll(&pfd, 1, 1000);
  } while (pr < 0 && errno == EINTR);
  if (pr <= 0) {
    ::close(fd);
    return 1;
  }

  uint8_t buf[64] = {};
  const ssize_t got = ::recvfrom(fd, buf, sizeof(buf), 0, nullptr, nullptr);
  ::close(fd);
  if (got < 8) {
    return 1;
  }
  return buf[0] == 0 ? 0 : 1;
}

void draw_person_boxes(cv::Mat& bgr, const vision_msgs::msg::Detection2DArray& dets) {
  if (bgr.empty() || bgr.cols <= 0 || bgr.rows <= 0) {
    return;
  }
  const int max_x = bgr.cols - 1;
  const int max_y = bgr.rows - 1;
  for (const auto& det : dets.detections) {
    if (!is_person(det)) {
      continue;
    }
    const double cx = bbox_center_x(det);
    const double cy = bbox_center_y(det);
    const double sx = det.bbox.size_x;
    const double sy = det.bbox.size_y;
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(sx) || !std::isfinite(sy)) {
      continue;
    }
    int x0 = static_cast<int>(std::lround(cx - sx * 0.5));
    int y0 = static_cast<int>(std::lround(cy - sy * 0.5));
    int x1 = static_cast<int>(std::lround(cx + sx * 0.5));
    int y1 = static_cast<int>(std::lround(cy + sy * 0.5));
    x0 = std::clamp(x0, 0, max_x);
    y0 = std::clamp(y0, 0, max_y);
    x1 = std::clamp(x1, 0, max_x);
    y1 = std::clamp(y1, 0, max_y);
    if (x1 <= x0 || y1 <= y0) {
      continue;
    }
    cv::rectangle(bgr, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 2);
  }
}

}  // namespace

class PersonPerceptionNode : public rclcpp::Node {
 public:
  PersonPerceptionNode() : Node("person_perception") {
    detections_source_ = declare_parameter<std::string>("detections_source", "mac");
    if (detections_source_ != "mac" && detections_source_ != "offline") {
      throw std::invalid_argument("detections_source must be 'mac' or 'offline'");
    }
    const std::string detections_topic =
        declare_parameter<std::string>("detections_topic", "/perception/detections_2d");
    const std::string onboard_detections_topic = declare_parameter<std::string>(
        "onboard_detections_topic", "/perception/detections_2d_onboard");
    const std::string points_topic =
        declare_parameter<std::string>("points_topic", "/aurora/points2");
    const std::string rgb_topic =
        declare_parameter<std::string>("rgb_topic", "/aurora/rgb/image_raw");
    const std::string camera_info_topic =
        declare_parameter<std::string>("camera_info_topic", "/aurora/rgb/camera_info");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    depth_frame_ = declare_parameter<std::string>("depth_frame", "depth_camera_link");

    const double rate_hz = declare_parameter<double>("rate_hz", 10.0);
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }

    const int64_t detections_timeout_ms = declare_parameter<int64_t>("detections_timeout_ms", 1000);
    if (detections_timeout_ms <= 0) {
      throw std::invalid_argument("detections_timeout_ms must be > 0");
    }
    detections_timeout_ = std::chrono::milliseconds(detections_timeout_ms);

    const int64_t points_timeout_ms = declare_parameter<int64_t>("points_timeout_ms", 700);
    if (points_timeout_ms <= 0) {
      throw std::invalid_argument("points_timeout_ms must be > 0");
    }
    const int64_t points_timeout_offline_ms =
        declare_parameter<int64_t>("points_timeout_offline_ms", 1000);
    if (points_timeout_offline_ms <= 0) {
      throw std::invalid_argument("points_timeout_offline_ms must be > 0");
    }
    points_timeout_mac_ms_ = points_timeout_ms;
    points_timeout_offline_ms_ = points_timeout_offline_ms;
    apply_points_timeout();

    publish_overlay_ = declare_parameter<bool>("publish_overlay", false);

    const double switch_margin_m = declare_parameter<double>("switch_margin_m", 0.30);
    if (switch_margin_m < 0.0) {
      throw std::invalid_argument("switch_margin_m must be >= 0");
    }
    lock_config_.switch_margin_m = static_cast<float>(switch_margin_m);

    lock_config_.target_lost_s = declare_parameter<double>("target_lost_s", 2.0);
    if (lock_config_.target_lost_s <= 0.0) {
      throw std::invalid_argument("target_lost_s must be > 0");
    }

    lock_config_.challenger_dwell_s = declare_parameter<double>("challenger_dwell_s", 2.0);
    if (lock_config_.challenger_dwell_s <= 0.0) {
      throw std::invalid_argument("challenger_dwell_s must be > 0");
    }

    const double handover_radius_m = declare_parameter<double>("handover_radius_m", 0.7);
    if (handover_radius_m < 0.0) {
      throw std::invalid_argument("handover_radius_m must be >= 0");
    }
    lock_config_.handover_radius_m = static_cast<float>(handover_radius_m);

    ego_compensation_enabled_ = declare_parameter<bool>("ego_compensation_enabled", true);
    ego_max_age_s_ = declare_parameter<double>("ego_max_age_s", 1.0);
    if (!(ego_max_age_s_ > 0.0)) {
      throw std::invalid_argument("ego_max_age_s must be > 0");
    }
    ego_max_yaw_rad_ = declare_parameter<double>("ego_max_yaw_rad", 0.6);
    if (!(ego_max_yaw_rad_ > 0.0)) {
      throw std::invalid_argument("ego_max_yaw_rad must be > 0");
    }
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    if (odom_frame_.empty()) {
      throw std::invalid_argument("odom_frame must be non-empty");
    }

    ethernet_ping_host_ = declare_parameter<std::string>("ethernet_ping_host", "192.168.88.57");
    if (ethernet_ping_host_.empty()) {
      throw std::invalid_argument("ethernet_ping_host must be non-empty");
    }
    const double ethernet_ping_period_s = declare_parameter<double>("ethernet_ping_period_s", 2.0);
    if (!(ethernet_ping_period_s > 0.0)) {
      throw std::invalid_argument("ethernet_ping_period_s must be > 0");
    }
    ethernet_ping_period_ = std::chrono::duration<double>(ethernet_ping_period_s);
    const int64_t fail_threshold = declare_parameter<int64_t>("ethernet_ping_fail_threshold", 3);
    if (fail_threshold < 1) {
      throw std::invalid_argument("ethernet_ping_fail_threshold must be >= 1");
    }
    ethernet_ping_fail_threshold_ = static_cast<int>(fail_threshold);
    ethernet_ip_ = declare_parameter<std::string>("ethernet_ip", "192.168.88.56");
    if (ethernet_ip_.empty()) {
      throw std::invalid_argument("ethernet_ip must be non-empty");
    }
    wifi_ap_ip_ = declare_parameter<std::string>("wifi_ap_ip", "192.168.149.1");
    if (wifi_ap_ip_.empty()) {
      throw std::invalid_argument("wifi_ap_ip must be non-empty");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto sensor_qos = rclcpp::SensorDataQoS();

    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        detections_topic, last_qos,
        std::bind(&PersonPerceptionNode::on_mac_detections, this, std::placeholders::_1));
    onboard_detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        onboard_detections_topic, last_qos,
        std::bind(&PersonPerceptionNode::on_onboard_detections, this, std::placeholders::_1));
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic, sensor_qos,
        std::bind(&PersonPerceptionNode::on_points, this, std::placeholders::_1));
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic, sensor_qos,
        std::bind(&PersonPerceptionNode::on_rgb, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, sensor_qos,
        std::bind(&PersonPerceptionNode::on_camera_info, this, std::placeholders::_1));

    persons_pub_ =
        create_publisher<mentorpi_msgs::msg::PersonArray>("/perception/persons", last_qos);
    nearest_pub_ =
        create_publisher<mentorpi_msgs::msg::NearestPerson>("/perception/nearest_person", last_qos);
    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(kOverlayTopic, sensor_qos);
    const auto dds_peer_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    dds_peer_pub_ = create_publisher<std_msgs::msg::String>("/perception/dds_peer", dds_peer_qos);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&PersonPerceptionNode::on_timer, this));

    param_cb_ = add_on_set_parameters_callback(
        std::bind(&PersonPerceptionNode::on_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "person_perception started (rate_hz=%.1f detections_timeout_ms=%ld "
                "points_timeout_ms=%ld points_timeout_offline_ms=%ld active_points_timeout_ms=%ld "
                "publish_overlay=%s detections_source=%s "
                "switch_margin_m=%.2f target_lost_s=%.1f challenger_dwell_s=%.1f "
                "handover_radius_m=%.2f ego_compensation_enabled=%s ego_max_age_s=%.1f "
                "ego_max_yaw_rad=%.2f odom_frame=%s "
                "detections=%s onboard_detections=%s points=%s rgb=%s %s->%s)",
                rate_hz, detections_timeout_ms, points_timeout_mac_ms_, points_timeout_offline_ms_,
                static_cast<int64_t>(points_timeout_.count()), publish_overlay_ ? "true" : "false",
                detections_source_.c_str(), lock_config_.switch_margin_m,
                lock_config_.target_lost_s, lock_config_.challenger_dwell_s,
                lock_config_.handover_radius_m, ego_compensation_enabled_ ? "true" : "false",
                ego_max_age_s_, ego_max_yaw_rad_, odom_frame_.c_str(), detections_topic.c_str(),
                onboard_detections_topic.c_str(), points_topic.c_str(), rgb_topic.c_str(),
                depth_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(get_logger(),
                "dds_peer ping host=%s period_s=%.1f fail_threshold=%d "
                "ethernet_ip=%s wifi_ap_ip=%s",
                ethernet_ping_host_.c_str(), ethernet_ping_period_s, ethernet_ping_fail_threshold_,
                ethernet_ip_.c_str(), wifi_ap_ip_.c_str());

    running_ = true;
    ping_thread_ = std::thread([this] { ping_loop(); });
  }

  ~PersonPerceptionNode() override {
    running_ = false;
    if (ping_thread_.joinable()) {
      ping_thread_.join();
    }
  }

 private:
  void ping_loop() {
    std::string last_peer;
    int consecutive_fails = 0;
    // Ethernet wins until enough consecutive ICMP failures (one lost
    // echo must not flap to AP while the Mac is still on the cable).
    bool ethernet_ok = true;
    while (running_ && rclcpp::ok()) {
      const bool ping_ok = ping_once(ethernet_ping_host_) == 0;
      if (ping_ok) {
        consecutive_fails = 0;
        ethernet_ok = true;
      } else if (consecutive_fails < ethernet_ping_fail_threshold_) {
        ++consecutive_fails;
        if (consecutive_fails < ethernet_ping_fail_threshold_) {
          RCLCPP_WARN(get_logger(), "ethernet ping fail %d/%d, hold %s", consecutive_fails,
                      ethernet_ping_fail_threshold_, ethernet_ip_.c_str());
        } else {
          ethernet_ok = false;
        }
      }
      const std::string peer = ethernet_ok ? ethernet_ip_ : wifi_ap_ip_;
      if (peer != last_peer) {
        last_peer = peer;
        RCLCPP_INFO(get_logger(), "dds_peer %s (ethernet ping %s)", peer.c_str(),
                    ethernet_ok ? "ok" : "fail");
      }
      std_msgs::msg::String msg;
      msg.data = peer;
      dds_peer_pub_->publish(msg);
      sleep_interruptible();
    }
  }

  void sleep_interruptible() {
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(ethernet_ping_period_);
    const auto max_slice = std::chrono::milliseconds(50);
    while (running_ && rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return;
      }
      auto slice = deadline - now;
      if (slice > max_slice) {
        slice = max_slice;
      }
      std::this_thread::sleep_for(slice);
    }
  }

  void apply_points_timeout() {
    const int64_t ms =
        detections_source_ == "offline" ? points_timeout_offline_ms_ : points_timeout_mac_ms_;
    points_timeout_ = std::chrono::milliseconds(ms);
  }

  void clear_detections_state() {
    have_detections_ = false;
    last_detections_.reset();
    last_detections_stamp_ = {};
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool next_overlay = publish_overlay_;
    bool have_overlay = false;
    std::string next_source = detections_source_;
    bool have_source = false;
    int64_t next_mac_ms = points_timeout_mac_ms_;
    int64_t next_offline_ms = points_timeout_offline_ms_;
    bool have_timeout = false;
    for (const auto& p : params) {
      if (p.get_name() == "publish_overlay") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "publish_overlay must be bool";
          return result;
        }
        next_overlay = p.as_bool();
        have_overlay = true;
        continue;
      }
      if (p.get_name() == "detections_source") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "detections_source must be string";
          return result;
        }
        next_source = p.as_string();
        if (next_source != "mac" && next_source != "offline") {
          result.successful = false;
          result.reason = "detections_source must be 'mac' or 'offline'";
          return result;
        }
        have_source = true;
        continue;
      }
      if (p.get_name() == "points_timeout_ms" || p.get_name() == "points_timeout_offline_ms") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          result.successful = false;
          result.reason = std::string(p.get_name()) + " must be integer";
          return result;
        }
        const int64_t value = p.as_int();
        if (value <= 0) {
          result.successful = false;
          result.reason = std::string(p.get_name()) + " must be > 0";
          return result;
        }
        if (p.get_name() == "points_timeout_ms") {
          next_mac_ms = value;
        } else {
          next_offline_ms = value;
        }
        have_timeout = true;
      }
    }
    if (have_overlay && next_overlay != publish_overlay_) {
      publish_overlay_ = next_overlay;
      RCLCPP_INFO(get_logger(), "publish_overlay=%s", publish_overlay_ ? "true" : "false");
    }
    const bool source_changed = have_source && next_source != detections_source_;
    if (have_timeout) {
      points_timeout_mac_ms_ = next_mac_ms;
      points_timeout_offline_ms_ = next_offline_ms;
    }
    if (source_changed) {
      detections_source_ = next_source;
      clear_detections_state();
      publish_timeout_empty();
    }
    if (source_changed || have_timeout) {
      apply_points_timeout();
      RCLCPP_INFO(get_logger(),
                  "detections_source=%s active_points_timeout_ms=%ld (mac=%ld offline=%ld)",
                  detections_source_.c_str(), static_cast<int64_t>(points_timeout_.count()),
                  points_timeout_mac_ms_, points_timeout_offline_ms_);
    }
    return result;
  }

  void accept_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
    last_detections_ = msg;
    last_detections_stamp_ = std::chrono::steady_clock::now();
    have_detections_ = true;
    publish_from_detections();
  }

  void on_mac_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
    if (detections_source_ != "mac") {
      return;
    }
    accept_detections(msg);
  }

  void on_onboard_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
    if (detections_source_ != "offline") {
      return;
    }
    accept_detections(msg);
  }

  void on_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    last_cloud_ = msg;
    last_cloud_stamp_ = std::chrono::steady_clock::now();
    have_cloud_ = true;
  }

  void on_rgb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (msg->width == 0 || msg->height == 0) {
      return;
    }
    have_rgb_ = true;
    if (!publish_overlay_) {
      return;
    }
    emit_overlay(msg);
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    const CameraIntrinsics cam = intrinsics_from_info(*msg);
    if (!intrinsics_valid(cam)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "camera_info has no usable intrinsics (fx=%.3f fy=%.3f)", cam.fx,
                           cam.fy);
      return;
    }
    if (!intrinsics_valid(intrinsics_)) {
      RCLCPP_INFO(get_logger(), "camera_info: fx=%.2f fy=%.2f cx=%.2f cy=%.2f %dx%d", cam.fx,
                  cam.fy, cam.cx, cam.cy, cam.width, cam.height);
    }
    intrinsics_ = cam;
  }

  bool detections_fresh(std::chrono::steady_clock::time_point now) const {
    return is_fresh(have_detections_, last_detections_stamp_, now, detections_timeout_) &&
           last_detections_ != nullptr;
  }

  void emit_overlay(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "overlay: cannot convert RGB encoding=%s to bgr8: %s",
                           msg->encoding.c_str(), ex.what());
      return;
    }
    if (cv_ptr == nullptr || cv_ptr->image.empty()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto dets = last_detections_;
    if (detections_fresh(now) && dets != nullptr) {
      draw_person_boxes(cv_ptr->image, *dets);
    }
    cv_ptr->header = msg->header;
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    overlay_pub_->publish(*cv_ptr->toImageMsg());
  }

  void warn_if_persons_stale(const mentorpi_msgs::msg::PersonArray& persons) {
    if (persons.persons.empty()) {
      return;
    }
    const rclcpp::Time t_now = now();
    const rclcpp::Time t_stamp(persons.header.stamp, t_now.get_clock_type());
    const int64_t age_ns = (t_now - t_stamp).nanoseconds();
    if (age_ns <= kPersonsLatencyBudgetNs) {
      return;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "persons latency %.0f ms exceeds 100 ms (now - header.stamp)",
                         static_cast<double>(age_ns) / 1.0e6);
  }

  void log_lock_event(const PersonTargetLockUpdateResult& lock_result) {
    switch (lock_result.event) {
      case PersonTargetLockEvent::kFirstLock:
        RCLCPP_INFO(get_logger(), "target lock id=%d range=%.2f m",
                    lock_result.nearest.person.track_id, lock_result.nearest.person.range);
        break;
      case PersonTargetLockEvent::kHandover:
        RCLCPP_INFO(get_logger(), "target handover id=%d -> id=%d dist=%.2f m absent=%.2f s",
                    lock_result.previous_track_id, lock_result.nearest.person.track_id,
                    lock_result.handover_distance_m, lock_result.absent_s);
        break;
      case PersonTargetLockEvent::kChallengerSwitch:
        RCLCPP_INFO(get_logger(), "target switch id=%d -> id=%d reason=challenger",
                    lock_result.previous_track_id, lock_result.nearest.person.track_id);
        break;
      case PersonTargetLockEvent::kLostRelock:
        RCLCPP_INFO(get_logger(), "target switch id=%d -> id=%d reason=lost_relock absent=%.2f s",
                    lock_result.previous_track_id, lock_result.nearest.person.track_id,
                    lock_result.absent_s);
        break;
      case PersonTargetLockEvent::kLost:
        RCLCPP_INFO(get_logger(), "target lost id=%d absent=%.2f s", lock_result.previous_track_id,
                    lock_result.absent_s);
        break;
      case PersonTargetLockEvent::kNone:
        break;
    }
  }

  void apply_lock_and_publish(const std_msgs::msg::Header& header,
                              const std::vector<PersonObservation>& observations) {
    const double now_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const PersonTargetLockUpdateResult lock_result =
        update_person_target_lock(lock_state_, lock_config_, now_s, observations);
    log_lock_event(lock_result);

    mentorpi_msgs::msg::PersonArray persons;
    persons.header = header;
    for (const PersonObservation& obs : lock_result.persons) {
      persons.persons.push_back(observation_to_hypothesis(obs));
    }

    mentorpi_msgs::msg::NearestPerson nearest;
    nearest.valid = lock_result.nearest.valid;
    nearest.coasting = lock_result.nearest.coasting;
    if (nearest.valid) {
      nearest.person = observation_to_hypothesis(lock_result.nearest.person);
    }

    warn_if_persons_stale(persons);
    persons_pub_->publish(persons);
    nearest_pub_->publish(nearest);
  }

  void publish_timeout_empty() {
    std_msgs::msg::Header header;
    header.stamp = now();
    apply_lock_and_publish(header, {});
  }

  bool lookup_base_yaw(const rclcpp::Time& stamp, double* yaw_rad) const {
    if (yaw_rad == nullptr) {
      return false;
    }
    try {
      const geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp);
      *yaw_rad = tf2::getYaw(tf.transform.rotation);
      return std::isfinite(*yaw_rad);
    } catch (const tf2::TransformException&) {
      return false;
    }
  }

  // false when the compensated bbox leaves the image (observation skipped); true otherwise.
  bool ego_compensate_bbox(const BboxPx& box, const vision_msgs::msg::Detection2D& det,
                           BboxPx* out) {
    if (out == nullptr) {
      return false;
    }
    *out = box;
    if (!ego_compensation_enabled_ || !intrinsics_valid(intrinsics_) || last_cloud_ == nullptr) {
      return true;
    }

    builtin_interfaces::msg::Time bbox_stamp_msg = det.header.stamp;
    if (header_stamp_is_zero(bbox_stamp_msg)) {
      bbox_stamp_msg = last_detections_->header.stamp;
    }
    const rclcpp::Time t_bbox(bbox_stamp_msg, get_clock()->get_clock_type());
    const rclcpp::Time t_cloud(last_cloud_->header.stamp, get_clock()->get_clock_type());
    const double age_s = (t_cloud - t_bbox).seconds();
    if (std::abs(age_s) > ego_max_age_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "ego compensation skipped: age out of window");
      return true;
    }

    double yaw_bbox = 0.0;
    double yaw_cloud = 0.0;
    if (!lookup_base_yaw(t_bbox, &yaw_bbox) || !lookup_base_yaw(t_cloud, &yaw_cloud)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ego compensation skipped: no TF");
      return true;
    }

    const double dpsi = yaw_cloud - yaw_bbox;
    if (std::abs(dpsi) > ego_max_yaw_rad_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "ego compensation skipped: yaw too large");
      return true;
    }

    const double center_before = bbox_center_u(box);
    const BboxPx rotated = rotate_bbox_yaw(box, intrinsics_, dpsi);
    BboxPx clipped;
    if (!bbox_clip_to_image(rotated, intrinsics_, &clipped)) {
      return false;
    }
    const double center_after = bbox_center_u(rotated);
    const double shift_px = std::abs(center_after - center_before);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ego yaw compensation dpsi=%.1f deg shift=%.0f px age=%.2f s",
                         dpsi * (180.0 / 3.14159265358979323846), shift_px, age_s);
    *out = clipped;
    return true;
  }

  bool transform_to_base(const Xyz& depth_xyz, const std::string& cloud_frame,
                         geometry_msgs::msg::PointStamped* out) {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = cloud_frame.empty() ? depth_frame_ : cloud_frame;
    in.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    in.point.x = depth_xyz.x;
    in.point.y = depth_xyz.y;
    in.point.z = depth_xyz.z;
    try {
      *out = tf_buffer_->transform(in, base_frame_);
      return std::isfinite(out->point.x) && std::isfinite(out->point.y);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no TF %s -> %s: %s",
                           in.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void publish_from_detections() {
    const std_msgs::msg::Header header = last_detections_->header;
    if (last_detections_->detections.empty()) {
      apply_lock_and_publish(header, {});
      return;
    }

    const auto now_st = std::chrono::steady_clock::now();
    const bool cloud_fresh = is_fresh(have_cloud_, last_cloud_stamp_, now_st, points_timeout_);
    if (!cloud_fresh || !have_rgb_ || last_cloud_ == nullptr) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "missing rgb or stale points; persons empty (have_rgb=%d cloud_fresh=%d)",
          static_cast<int>(have_rgb_), static_cast<int>(cloud_fresh));
      apply_lock_and_publish(header, {});
      return;
    }

    // D6.4: no intrinsics, no target. Guessing a focal length from the FOV would
    // hide a regression of the transverse sign behind a plausible number.
    if (!intrinsics_valid(intrinsics_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no camera_info yet; cannot match pixels to cloud, persons empty");
      apply_lock_and_publish(header, {});
      return;
    }

    CloudXyzView cloud_view;
    if (!parse_cloud_xyz(*last_cloud_, &cloud_view)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cloud %ux%u has no float32 xyz payload; persons empty",
                           last_cloud_->width, last_cloud_->height);
      apply_lock_and_publish(header, {});
      return;
    }

    std::vector<PersonObservation> observations;
    const std::string cloud_frame = last_cloud_->header.frame_id;
    const auto& detections = last_detections_->detections;
    for (const auto& det : detections) {
      if (!is_person(det)) {
        continue;
      }
      std::int32_t track_id = 0;
      if (!parse_detection_track_id(det.id, &track_id)) {
        continue;
      }
      BboxPx box;
      if (!bbox_from_detection(bbox_center_x(det), bbox_center_y(det), det.bbox.size_x,
                               det.bbox.size_y, &box)) {
        continue;
      }
      BboxPx search_box;
      if (!ego_compensate_bbox(box, det, &search_box)) {
        continue;
      }
      Xyz depth_xyz;
      if (!nearest_cluster_in_bbox(cloud_view, intrinsics_, search_box, kClusterSlabM,
                                   kClusterMinPoints, &depth_xyz)) {
        continue;
      }
      geometry_msgs::msg::PointStamped base_pt;
      if (!transform_to_base(depth_xyz, cloud_frame, &base_pt)) {
        continue;
      }
      const PersonXyRange geom = xy_range_in_base(base_pt.point.x, base_pt.point.y);
      if (!std::isfinite(geom.range)) {
        continue;
      }
      PersonObservation obs;
      obs.track_id = track_id;
      obs.x = geom.x;
      obs.y = geom.y;
      obs.range = geom.range;
      obs.confidence = person_score(det);
      observations.push_back(obs);
    }

    if (observations.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "detections=%zu but no valid depth/TF; nearest empty",
                           detections.size());
    }
    apply_lock_and_publish(header, observations);
  }

  void on_timer() {
    const auto now_st = std::chrono::steady_clock::now();
    if (!detections_fresh(now_st)) {
      publish_timeout_empty();
    }
  }

  std::string base_frame_;
  std::string depth_frame_;
  std::string odom_frame_{"odom"};
  bool ego_compensation_enabled_{true};
  double ego_max_age_s_{1.0};
  double ego_max_yaw_rad_{0.6};
  std::string ethernet_ping_host_;
  std::string ethernet_ip_;
  std::string wifi_ap_ip_;
  std::chrono::duration<double> ethernet_ping_period_{2.0};
  int ethernet_ping_fail_threshold_{3};
  std::chrono::milliseconds detections_timeout_{1000};
  std::chrono::milliseconds points_timeout_{700};
  int64_t points_timeout_mac_ms_{700};
  int64_t points_timeout_offline_ms_{1000};
  std::string detections_source_{"mac"};
  bool publish_overlay_{false};
  PersonTargetLockConfig lock_config_;
  PersonTargetLockState lock_state_;

  vision_msgs::msg::Detection2DArray::ConstSharedPtr last_detections_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_cloud_;
  std::chrono::steady_clock::time_point last_detections_stamp_{};
  std::chrono::steady_clock::time_point last_cloud_stamp_{};
  bool have_detections_{false};
  bool have_cloud_{false};
  bool have_rgb_{false};
  CameraIntrinsics intrinsics_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr onboard_detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<mentorpi_msgs::msg::PersonArray>::SharedPtr persons_pub_;
  rclcpp::Publisher<mentorpi_msgs::msg::NearestPerson>::SharedPtr nearest_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dds_peer_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  std::atomic<bool> running_{false};
  std::thread ping_thread_;
};

}  // namespace mentorpi_perception

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_perception::PersonPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
