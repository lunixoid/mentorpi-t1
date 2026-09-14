#pragma once

#include <functional>
#include <string>
#include <vector>

namespace units {

inline constexpr const char* kOurs = "mentorpi-t1.service";
inline constexpr const char* kStock = "start_node.service";
inline constexpr const char* kOursContainer = "mentorpi-t1";
inline constexpr const char* kStockContainer = "MentorPi";

// One in-container rclpy probe (~2s window). Operator budget ~5s; cold DDS is
// handled inside the Python collector, not via C++ retry of the whole exec.
inline constexpr int kRosProbeAttempts = 1;

enum class Demo { Active, Inactive, Failed };
enum class Stock { Inactive, Active };
enum class Chassis { Inactive, Active };
enum class Mode { Follow, Manual, Forbidden };
enum class ModeCommand { Forbid, Allow, Manual };
enum class RemoteController { Inactive, Active };
inline constexpr const char* kNoControlStatusReason = "no /control/status";
inline constexpr const char* kControlSetModeService = "/control/set_mode";
enum class Lidar { Inactive, Active, Degraded };
enum class Camera { Inactive, Active, Degraded };
enum class Imu { Inactive, Active, Degraded };
enum class Odometry { Inactive, Active, Degraded };
enum class Model { Inactive, Active, Degraded };
enum class Calibration { Factory, File, Unused };
enum class DdsBuffers { Unknown, Active, Degraded };

// Receive buffer from mentorpi_bringup/config/fastdds_camera_frames.xml (SD029).
inline constexpr int kDdsRmemMinBytes = 8388608;
inline constexpr const char* kDdsRmemMaxPath = "/proc/sys/net/core/rmem_max";

// SD006 demo-contour contract (see lidar_layer.launch.py).
inline constexpr const char* kLidarScanTopic = "/scan";
inline constexpr const char* kLidarBaseFrame = "base_footprint";
inline constexpr const char* kLidarFrame = "lidar_frame";

// SD008 camera contract (see camera_layer.launch.py, as-built Aurora 930).
inline constexpr const char* kCameraColorTopic = "/aurora/rgb/image_raw";
inline constexpr const char* kCameraColorCompressedTopic = "/aurora/rgb/image_raw/compressed";
inline constexpr const char* kCameraDepthTopic = "/aurora/depth/image_raw";
inline constexpr const char* kCameraCloudTopic = "/aurora/points2";
inline constexpr const char* kCameraBaseFrame = "base_footprint";
inline constexpr const char* kCameraFrame = "depth_camera_link";

// SD010 IMU contract (see imu_layer.launch.py).
inline constexpr const char* kImuTopic = "/imu";
inline constexpr const char* kImuOdomTopic = "/imu_odom";
inline constexpr const char* kImuBaseFrame = "base_footprint";
inline constexpr const char* kImuFrame = "imu_link";

// SD003/SD005 odometry contract (see odom_publisher).
inline constexpr const char* kOdomRawTopic = "/odom_raw";
inline constexpr const char* kOdomFrame = "odom";
inline constexpr const char* kOdomChildFrame = "base_footprint";

// SD007 platform model contract (see robot_model_layer.launch.py).
inline constexpr const char* kModelDescriptionTopic = "/robot_description";
inline constexpr const char* kModelBaseFrame = "base_footprint";
inline constexpr const char* kModelBaseLinkFrame = "base_link";
inline constexpr const char* kModelImuFrame = "imu_link";
inline constexpr const char* kModelDepthFrame = "depth_cam_frame";

struct Status {
  Demo demo{Demo::Inactive};
  Stock stock{Stock::Inactive};
  Chassis chassis{Chassis::Inactive};
  Mode mode{Mode::Follow};
  // True only when `/control/status` parsed. Default Follow must not be shown
  // as a live mode when this is false (missing topic is not AutoFollow).
  bool have_control{false};
  std::string reason;
  RemoteController remote_controller{RemoteController::Inactive};
  Lidar lidar{Lidar::Inactive};
  // Set when a lidar probe ran; used for degraded diagnostics in UI.
  bool lidar_scan_ok{false};
  bool lidar_tf_ok{false};
  Camera camera{Camera::Inactive};
  // Set when a camera probe ran; used for degraded diagnostics in UI.
  bool camera_color_ok{false};
  bool camera_depth_ok{false};
  bool camera_tf_ok{false};
  Imu imu{Imu::Inactive};
  // Set when an IMU probe ran; used for degraded diagnostics in UI.
  bool imu_msg_ok{false};
  bool imu_odom_ok{false};
  bool imu_tf_ok{false};
  Odometry odometry{Odometry::Inactive};
  // Set when an odometry probe ran; used for degraded diagnostics in UI.
  bool odom_msg_ok{false};
  bool odom_tf_ok{false};
  Model model{Model::Inactive};
  // Set when a model probe ran; used for degraded diagnostics in UI.
  bool model_description_ok{false};
  bool model_tf_base_ok{false};
  bool model_tf_lidar_ok{false};
  bool model_tf_imu_ok{false};
  bool model_tf_depth_ok{false};
  Calibration calibration{Calibration::Factory};
  bool have_calibration{false};
  DdsBuffers dds_buffers{DdsBuffers::Unknown};
  int rmem_max{0};
};

class ProcessRunner {
 public:
  virtual ~ProcessRunner() = default;
  virtual int run(const std::vector<std::string>& args, std::string* stdout_out = nullptr) = 0;
  virtual int run_lines(const std::vector<std::string>& args,
                        const std::function<void(const std::string& line)>& on_line,
                        std::string* stdout_out = nullptr);
  virtual bool have_executable(const char* name) const = 0;
};

bool have_systemctl();
ProcessRunner& process_runner();
Status query();
Status query(ProcessRunner& runner);
Status query_host(ProcessRunner& runner);
void fill_dds_buffers(Status& status, const std::string& path = kDdsRmemMaxPath);
bool start();
bool restart();
bool stock();

struct ModeChange {
  bool ok{false};
  bool have_from{false};
  bool have_to{false};
  Mode from{Mode::Follow};
  Mode to{Mode::Follow};
  std::string reason;
  std::string detail;
};

// Structured stdout of in-container ros_mode.py. Requires T1CTL_MODE_OK=0|1.
bool parse_mode_result(const std::string& text, ModeChange* change);

// Calls SetControlMode via docker exec + ros_mode.py. Never publishes
// `/control/state`. On failure fills `known` with systemd facts only and
// `change.detail` for the error block.
bool set_mode(ModeCommand command, ModeChange* change, Status* known);
bool set_mode(ProcessRunner& runner, ModeCommand command, ModeChange* change, Status* known);

// YAML of `/control/status`. Ignores a source banner before the document.
// Returns true only when both `state` and `remote_controller` parsed.
// `reason` is optional; stored only when mode is Forbidden.
bool parse_control_status_yaml(const std::string& text, Status* status);

struct RosProbeView {
  bool have_chassis{false};
  bool chassis_live{false};
  bool have_control{false};
  bool have_lidar{false};
  bool lidar_live{false};
  bool lidar_scan{false};
  bool lidar_tf{false};
  bool have_camera{false};
  bool camera_live{false};
  bool camera_color{false};
  bool camera_depth{false};
  bool camera_tf{false};
  bool have_imu{false};
  bool imu_live{false};
  bool imu_msg{false};
  bool imu_odom{false};
  bool imu_tf{false};
  bool have_odometry{false};
  bool odometry_live{false};
  bool odom_msg{false};
  bool odom_tf{false};
  bool have_model{false};
  bool model_live{false};
  bool model_description{false};
  bool model_tf_base{false};
  bool model_tf_lidar{false};
  bool model_tf_imu{false};
  bool model_tf_depth{false};
  bool have_calibration{false};
  Calibration calibration{Calibration::Factory};
};

// Unified docker-exec output: `T1CTL_CHASSIS=0|1`, `T1CTL_LIDAR=0|1`,
// `T1CTL_CAMERA=0|1`, `T1CTL_MODEL=0|1`, `T1CTL_CALIB=factory|file|unused`,
// optional YAML.
RosProbeView parse_ros_probe(const std::string& text, Status* control);

}  // namespace units
