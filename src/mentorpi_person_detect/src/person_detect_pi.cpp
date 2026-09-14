#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <utility>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "mentorpi_person_detect/byte_track.hpp"
#include "mentorpi_person_detect/yolo11n_detector.hpp"

namespace mentorpi_person_detect {

namespace {

constexpr const char* kPersonClass = "person";

std::string resolve_package_path(const std::string& relative) {
  if (!relative.empty() && relative.front() == '/') {
    return relative;
  }
  const std::string share = ament_index_cpp::get_package_share_directory("mentorpi_person_detect");
  if (relative.empty()) {
    return share;
  }
  return share + "/" + relative;
}

void fill_bbox_center(vision_msgs::msg::BoundingBox2D& bbox, double cx, double cy) {
  bbox.center.position.x = cx;
  bbox.center.position.y = cy;
  bbox.center.theta = 0.0;
}

cv::Mat image_to_bgr(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
    const auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    return cv_ptr->image;
  }
  if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
    const auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat bgr;
    cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  const auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  return cv_ptr->image;
}

}  // namespace

class PersonDetectPiNode : public rclcpp::Node {
 public:
  PersonDetectPiNode() : Node("person_detect_pi") {
    enabled_ = declare_parameter<bool>("enabled", false);
    const std::string image_topic =
        declare_parameter<std::string>("image_topic", "/aurora/rgb/image_raw");
    const std::string detections_topic =
        declare_parameter<std::string>("detections_topic", "/perception/detections_2d_onboard");
    const std::string weights_rel =
        declare_parameter<std::string>("weights", "models/yolo11n.ncnn.param");
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.25);
    const int num_threads = declare_parameter<int>("num_threads", 2);
    const int infer_period_ms = declare_parameter<int>("infer_period_ms", 500);

    const ByteTrackConfig track_defaults;
    ByteTrackConfig track_cfg;
    track_cfg.track_high_thresh = static_cast<float>(
        declare_parameter<double>("track_high_thresh", track_defaults.track_high_thresh));
    track_cfg.track_low_thresh = static_cast<float>(
        declare_parameter<double>("track_low_thresh", track_defaults.track_low_thresh));
    track_cfg.new_track_thresh = static_cast<float>(
        declare_parameter<double>("new_track_thresh", track_defaults.new_track_thresh));
    track_cfg.match_thresh =
        static_cast<float>(declare_parameter<double>("match_thresh", track_defaults.match_thresh));
    track_cfg.fuse_score = declare_parameter<bool>("fuse_score", track_defaults.fuse_score);
    track_cfg.track_buffer_s =
        declare_parameter<double>("track_buffer_s", track_defaults.track_buffer_s);

    if (confidence_threshold_ < 0.0 || confidence_threshold_ > 1.0) {
      throw std::invalid_argument("confidence_threshold must be in [0, 1]");
    }
    if (num_threads < 1) {
      throw std::invalid_argument("num_threads must be >= 1");
    }
    if (infer_period_ms < 0) {
      throw std::invalid_argument("infer_period_ms must be >= 0");
    }
    if (track_cfg.track_high_thresh < 0.0f || track_cfg.track_high_thresh > 1.0f) {
      throw std::invalid_argument("track_high_thresh must be in [0, 1]");
    }
    if (track_cfg.track_low_thresh < 0.0f || track_cfg.track_low_thresh > 1.0f) {
      throw std::invalid_argument("track_low_thresh must be in [0, 1]");
    }
    if (track_cfg.new_track_thresh < 0.0f || track_cfg.new_track_thresh > 1.0f) {
      throw std::invalid_argument("new_track_thresh must be in [0, 1]");
    }
    if (track_cfg.match_thresh < 0.0f || track_cfg.match_thresh > 1.0f) {
      throw std::invalid_argument("match_thresh must be in [0, 1]");
    }
    if (track_cfg.track_low_thresh > track_cfg.track_high_thresh) {
      throw std::invalid_argument("track_low_thresh must be <= track_high_thresh");
    }
    if (!(track_cfg.match_thresh > 0.0f)) {
      throw std::invalid_argument("match_thresh must be > 0");
    }
    if (!(track_cfg.track_buffer_s > 0.0)) {
      throw std::invalid_argument("track_buffer_s must be > 0");
    }
    infer_period_ = std::chrono::milliseconds(infer_period_ms);

    // NCNN/OpenMP otherwise oversubscribe the 4 Pi cores and starve Aurora points2
    // (person_perception then drops the target: cloud_fresh=0, nearest empty).
    const std::string omp_threads = std::to_string(num_threads);
    ::setenv("OMP_NUM_THREADS", omp_threads.c_str(), 0);
    ::setenv("NCNN_NUM_THREADS", omp_threads.c_str(), 0);

    const std::string weights_path = resolve_package_path(weights_rel);

    Yolo11nConfig yolo_cfg;
    yolo_cfg.num_threads = num_threads;
    if (!detector_.load(weights_path, yolo_cfg)) {
      throw std::runtime_error("failed to load NCNN weights: " + weights_path);
    }

    tracker_ = std::make_unique<ByteTracker>(track_cfg);

    const auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto sensor_qos = rclcpp::SensorDataQoS();

    detections_pub_ =
        create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic, pub_qos);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, sensor_qos,
        std::bind(&PersonDetectPiNode::on_image, this, std::placeholders::_1));

    param_cb_ = add_on_set_parameters_callback(
        std::bind(&PersonDetectPiNode::on_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "person_detect_pi started enabled=%s image=%s out=%s weights=%s conf=%.2f "
                "num_threads=%d infer_period_ms=%d track_high_thresh=%.2f track_low_thresh=%.2f "
                "new_track_thresh=%.2f match_thresh=%.2f fuse_score=%s track_buffer_s=%.1f",
                enabled_.load() ? "true" : "false", image_topic.c_str(), detections_topic.c_str(),
                weights_path.c_str(), confidence_threshold_, num_threads, infer_period_ms,
                track_cfg.track_high_thresh, track_cfg.track_low_thresh, track_cfg.new_track_thresh,
                track_cfg.match_thresh, track_cfg.fuse_score ? "true" : "false",
                track_cfg.track_buffer_s);
  }

 private:
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool next_enabled = enabled_.load();
    bool have_enabled = false;
    for (const auto& p : params) {
      if (p.get_name() != "enabled") {
        continue;
      }
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "enabled must be bool";
        return result;
      }
      next_enabled = p.as_bool();
      have_enabled = true;
    }
    if (have_enabled && next_enabled != enabled_.load()) {
      enabled_.store(next_enabled);
      RCLCPP_INFO(get_logger(), "enabled=%s", enabled_.load() ? "true" : "false");
      if (!enabled_.load()) {
        tracker_->reset();
        have_last_ = false;
        did_infer_ = false;
        last_inference_header_ = std_msgs::msg::Header{};
        last_out_ = vision_msgs::msg::Detection2DArray{};
      }
    }
    return result;
  }

  void publish_empty(const std_msgs::msg::Header& header) {
    vision_msgs::msg::Detection2DArray out;
    out.header = header;
    detections_pub_->publish(out);
  }

  void publish_last(const std_msgs::msg::Header& header) {
    last_out_.header = header;
    for (auto& det : last_out_.detections) {
      det.header = last_inference_header_;
    }
    detections_pub_->publish(last_out_);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!enabled_.load()) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (infer_period_.count() > 0 && did_infer_ && (now - last_infer_) < infer_period_) {
      if (have_last_) {
        publish_last(msg->header);
      }
      return;
    }
    last_infer_ = now;
    did_infer_ = true;

    cv::Mat bgr;
    try {
      bgr = image_to_bgr(msg);
    } catch (const cv_bridge::Exception& exc) {
      RCLCPP_ERROR(get_logger(), "RGB convert failed: %s", exc.what());
      have_last_ = false;
      publish_empty(msg->header);
      return;
    }
    if (bgr.empty()) {
      have_last_ = false;
      publish_empty(msg->header);
      return;
    }

    std::vector<DetectionBox> detections;
    try {
      detections = detector_.detect(bgr, static_cast<float>(confidence_threshold_));
    } catch (const std::exception& exc) {
      RCLCPP_ERROR(get_logger(), "YOLO11n detect failed: %s", exc.what());
      have_last_ = false;
      publish_empty(msg->header);
      return;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "YOLO11n detect failed: unknown error");
      have_last_ = false;
      publish_empty(msg->header);
      return;
    }
    RCLCPP_INFO_ONCE(get_logger(), "first YOLO11n frame rgb=%dx%d boxes=%zu", bgr.cols, bgr.rows,
                     detections.size());
    if (!detections.empty()) {
      const auto& box = detections.front();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "yolo person boxes=%zu first=%.0fx%.0f @%.0f,%.0f score=%.2f", detections.size(),
          static_cast<double>(box.x2 - box.x1), static_cast<double>(box.y2 - box.y1),
          static_cast<double>((box.x1 + box.x2) * 0.5f),
          static_cast<double>((box.y1 + box.y2) * 0.5f), static_cast<double>(box.score));
    }

    std::vector<TrackedBox> tracks;
    try {
      tracks = tracker_->update(detections, rclcpp::Time(msg->header.stamp).seconds());
    } catch (const std::exception& exc) {
      RCLCPP_ERROR(get_logger(), "ByteTrack failed: %s", exc.what());
      have_last_ = false;
      publish_empty(msg->header);
      return;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "ByteTrack failed: unknown error");
      have_last_ = false;
      publish_empty(msg->header);
      return;
    }

    last_inference_header_ = msg->header;
    last_out_.header = msg->header;
    last_out_.detections.clear();
    for (const auto& track : tracks) {
      if (track.track_id <= 0) {
        continue;
      }
      vision_msgs::msg::Detection2D det;
      det.header = last_inference_header_;
      det.id = std::to_string(track.track_id);
      const double cx = (track.x1 + track.x2) * 0.5;
      const double cy = (track.y1 + track.y2) * 0.5;
      fill_bbox_center(det.bbox, cx, cy);
      det.bbox.size_x = static_cast<double>(std::max(0.0f, track.x2 - track.x1));
      det.bbox.size_y = static_cast<double>(std::max(0.0f, track.y2 - track.y1));
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = kPersonClass;
      hyp.hypothesis.score = track.score;
      det.results.push_back(hyp);
      last_out_.detections.push_back(det);
    }
    have_last_ = true;
    detections_pub_->publish(last_out_);
  }

  std::atomic<bool> enabled_{false};
  double confidence_threshold_{0.25};
  std::chrono::milliseconds infer_period_{500};
  std::chrono::steady_clock::time_point last_infer_{};
  bool did_infer_{false};
  bool have_last_{false};
  std_msgs::msg::Header last_inference_header_{};
  vision_msgs::msg::Detection2DArray last_out_;
  Yolo11nDetector detector_;
  std::unique_ptr<ByteTracker> tracker_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace mentorpi_person_detect

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_person_detect::PersonDetectPiNode>());
  rclcpp::shutdown();
  return 0;
}
