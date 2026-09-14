#pragma once

#include <functional>
#include <string>
#include <vector>

#include "units.hpp"

namespace calib {

struct PoseRow {
  double x{0};
  double y{0};
  double z{0};
  double roll{0};
  double pitch{0};
  double yaw{0};
  std::string xyz_source;
  std::string rpy_source;
};

struct Show {
  bool ok{false};
  bool have_source{false};
  units::Calibration source{units::Calibration::Factory};
  std::string reason;
  bool have_residual{false};
  double residual_m{0};
  double residual_deg{0};
  bool have_camera{false};
  bool have_lidar{false};
  bool have_imu{false};
  PoseRow camera;
  PoseRow lidar;
  PoseRow imu;
  std::string detail;
  units::Demo demo{units::Demo::Inactive};
  units::Stock stock{units::Stock::Inactive};
};

struct FieldDelta {
  std::string name;
  std::string before;
  std::string after;
  std::string tag;
};

struct Result {
  bool ok{false};
  bool have_ok{false};
  std::string stage;
  std::vector<FieldDelta> fields;
  bool have_residual_before{false};
  bool have_residual_after{false};
  double residual_before{0};
  double residual_after{0};
  bool have_imu_residual_before{false};
  bool have_imu_residual_after{false};
  double imu_residual_before{0};
  double imu_residual_after{0};
  std::vector<std::string> hints;
  bool have_frames{false};
  int frames{0};
  bool have_cloud_points{false};
  int cloud_points{0};
  bool have_scan_rays{false};
  int scan_rays{0};
  bool have_imu_samples{false};
  int imu_samples{0};
  bool have_travel{false};
  double travel_m{0};
  bool have_turn{false};
  double turn_deg{0};
  bool clear_hints{false};
  std::vector<std::string> observed;
  std::string layer;
  std::string frame;
  std::string host_frame;
  bool frame_copied{false};
  std::string frame_copy_error;
  std::string detail;
  std::string message;
  std::string cause;
  std::string draft;
  units::Demo demo{units::Demo::Inactive};
  units::Stock stock{units::Stock::Inactive};
};

// Host path for the side-stage PNG after `docker cp` (not the bind-mounted config/).
inline constexpr const char* kHostSideFramePath = "/tmp/t1ctl-side_frame.png";

bool parse_show(const std::string& text, Show* show);
bool query(units::ProcessRunner& runner, Show* show);
bool query(Show* show);

bool parse_result_line(const std::string& line, Result* result);
bool parse_result(const std::string& text, Result* result);
bool run_command(units::ProcessRunner& runner, const std::vector<std::string>& subcommand,
                 Result* result,
                 const std::function<void(const std::string& line)>& on_line = nullptr);
bool run_command(const std::vector<std::string>& subcommand, Result* result,
                 const std::function<void(const std::string& line)>& on_line = nullptr);

}  // namespace calib
