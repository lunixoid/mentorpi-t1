#pragma once

#include <string>

#include "units.hpp"

namespace viewer {

inline constexpr int kBridgePort = 8765;
inline constexpr const char* kDefaultHost = "192.168.88.56";
inline constexpr const char* kFixedFrame = "odom";
inline constexpr const char* kOdomTopic = "/odom_raw";
// SD007 platform model contract (see robot_model_layer.launch.py).
inline constexpr const char* kRobotDescriptionTopic = "/robot_description";
inline constexpr const char* kModelTfChain = "odom -> base_footprint -> base_link";
inline constexpr const char* kModelSensorFrames = "lidar_frame, imu_link, depth_cam_frame";
// SD006 demo-contour contract (see lidar_layer.launch.py).
inline constexpr const char* kLidarScanTopic = "/scan";
inline constexpr const char* kLidarBaseFrame = "base_footprint";
inline constexpr const char* kLidarFrame = "lidar_frame";
inline constexpr const char* kLidarTfFrames = "base_footprint -> lidar_frame";
// SD008 camera viewer contract (see camera_layer.launch.py, as-built Aurora 930).
inline constexpr const char* kCameraColorTopic = units::kCameraColorTopic;
inline constexpr const char* kCameraColorCompressedTopic = units::kCameraColorCompressedTopic;
inline constexpr const char* kCameraDepthTopic = units::kCameraDepthTopic;
inline constexpr const char* kCameraCloudTopic = units::kCameraCloudTopic;
inline constexpr const char* kCameraBaseFrame = units::kCameraBaseFrame;
inline constexpr const char* kCameraFrame = units::kCameraFrame;
inline constexpr const char* kCameraTfFrames = "base_footprint -> depth_camera_link";
// Cheat-sheet: compressed if the topic is in the graph, else raw; 3D prefers cloud.
inline constexpr const char* kCamera2dHint =
    "/aurora/rgb/image_raw/compressed or /aurora/rgb/image_raw";
inline constexpr const char* kCamera3dHint = "/aurora/points2";
// SD014 D5: prod persons in Foxglove; overlay only after t1ctl debug on.
inline constexpr const char* kPersonsTopic = "/perception/persons";
inline constexpr const char* kNearestPersonTopic = "/perception/nearest_person";
inline constexpr const char* kDetections2dTopic = "/perception/detections_2d";
inline constexpr const char* kPersonsOverlayTopic = "/perception/persons/overlay";
inline constexpr const char* kPersonsOverlayHint =
    "/perception/persons/overlay after t1ctl debug on (bgr8)";
// SD010 IMU viewer contract (see imu_layer.launch.py).
inline constexpr const char* kImuTopic = "/imu";
inline constexpr const char* kImuOdomTopic = "/imu_odom";
inline constexpr const char* kImuTfFrames = "base_footprint -> imu_link";
// Legacy alias: dynamic odom pose for viewer (subset of kModelTfChain).
inline constexpr const char* kTfFrames = "odom -> base_footprint";

enum class DemoContour { Inactive, Active };
enum class Bridge { Inactive, Active };

struct Status {
  DemoContour demo{DemoContour::Inactive};
  Bridge bridge{Bridge::Inactive};
  bool port_listening{false};
  std::string host{kDefaultHost};
  std::string ros_domain_id;
  int port{kBridgePort};
};

Status query();
Status query(units::ProcessRunner& runner);
bool start();
bool start(units::ProcessRunner& runner, Status* out, std::string* detail = nullptr);
bool stop();
bool stop(units::ProcessRunner& runner, Status* out);

bool parse_probe(const std::string& text, Status* status);
std::string detect_host(units::ProcessRunner& runner);

}  // namespace viewer
