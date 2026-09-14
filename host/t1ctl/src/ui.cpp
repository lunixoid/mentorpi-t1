#include "ui.hpp"

#include <unistd.h>

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace ui {
namespace {

constexpr const char* kVersion = T1CTL_VERSION;

constexpr const char* kDim = "\033[2m";
constexpr const char* kBold = "\033[1m";
constexpr const char* kRed = "\033[31m";
constexpr const char* kGreen = "\033[32m";
constexpr const char* kBoldRed = "\033[1;31m";
constexpr const char* kReset = "\033[0m";

bool env_no_color() { return std::getenv("NO_COLOR") != nullptr; }

bool env_dumb_term() {
  const char* term = std::getenv("TERM");
  return term != nullptr && std::strcmp(term, "dumb") == 0;
}

bool stream_color(int fd) { return !env_no_color() && !env_dumb_term() && ::isatty(fd) != 0; }

bool stdout_color() { return stream_color(STDOUT_FILENO); }

bool stderr_color() { return stream_color(STDERR_FILENO); }

std::string pad_right(std::string_view text, std::size_t width) {
  std::string out(text);
  if (out.size() < width) {
    out.append(width - out.size(), ' ');
  }
  return out;
}

void paint(std::ostream& out, bool color, const char* code, std::string_view text) {
  if (color) {
    out << code;
  }
  out << text;
  if (color) {
    out << kReset;
  }
}

const char* demo_text(units::Demo demo) {
  switch (demo) {
    case units::Demo::Active:
      return "active";
    case units::Demo::Inactive:
      return "inactive";
    case units::Demo::Failed:
      return "failed";
  }
  return "inactive";
}

const char* stock_text(units::Stock stock) {
  return stock == units::Stock::Active ? "active" : "inactive";
}

const char* chassis_text(units::Chassis chassis) {
  return chassis == units::Chassis::Active ? "active" : "inactive";
}

void print_demo_value(std::ostream& out, bool color, const units::Status& status) {
  const char* text = demo_text(status.demo);
  switch (status.demo) {
    case units::Demo::Active:
      paint(out, color, kGreen, text);
      break;
    case units::Demo::Failed:
      paint(out, color, kBoldRed, text);
      break;
    case units::Demo::Inactive:
      if (status.stock == units::Stock::Inactive) {
        paint(out, color, kRed, text);
      } else {
        out << text;
      }
      break;
  }
}

const char* mode_text(units::Mode mode) {
  switch (mode) {
    case units::Mode::Manual:
      return "manual";
    case units::Mode::Forbidden:
      return "forbidden";
    case units::Mode::Follow:
      return "follow";
  }
  return "follow";
}

bool status_shows_forbidden(const units::Status& status) {
  return !status.have_control || status.mode == units::Mode::Forbidden;
}

const char* status_reason_text(const units::Status& status) {
  if (!status.have_control) {
    return units::kNoControlStatusReason;
  }
  return status.reason.c_str();
}

const char* remote_controller_text(units::RemoteController remote) {
  return remote == units::RemoteController::Active ? "active" : "inactive";
}

const char* lidar_text(units::Lidar lidar) {
  switch (lidar) {
    case units::Lidar::Active:
      return "active";
    case units::Lidar::Degraded:
      return "degraded";
    case units::Lidar::Inactive:
      return "inactive";
  }
  return "inactive";
}

const char* camera_text(units::Camera camera) {
  switch (camera) {
    case units::Camera::Active:
      return "active";
    case units::Camera::Degraded:
      return "degraded";
    case units::Camera::Inactive:
      return "inactive";
  }
  return "inactive";
}

const char* imu_text(units::Imu imu) {
  switch (imu) {
    case units::Imu::Active:
      return "active";
    case units::Imu::Degraded:
      return "degraded";
    case units::Imu::Inactive:
      return "inactive";
  }
  return "inactive";
}

const char* odometry_text(units::Odometry odometry) {
  switch (odometry) {
    case units::Odometry::Active:
      return "active";
    case units::Odometry::Degraded:
      return "degraded";
    case units::Odometry::Inactive:
      return "inactive";
  }
  return "inactive";
}

const char* model_text(units::Model model) {
  switch (model) {
    case units::Model::Active:
      return "active";
    case units::Model::Degraded:
      return "degraded";
    case units::Model::Inactive:
      return "inactive";
  }
  return "inactive";
}

const char* calibration_text(units::Calibration calibration) {
  switch (calibration) {
    case units::Calibration::File:
      return "file";
    case units::Calibration::Unused:
      return "unused";
    case units::Calibration::Factory:
      return "factory";
  }
  return "factory";
}

void print_calibration_value(std::ostream& out, bool color, units::Calibration calibration) {
  const char* text = calibration_text(calibration);
  switch (calibration) {
    case units::Calibration::File:
      paint(out, color, kGreen, text);
      break;
    case units::Calibration::Unused:
      paint(out, color, kBoldRed, text);
      break;
    case units::Calibration::Factory:
      out << text;
      break;
  }
}

std::string format_fixed(double value, int width, int precision) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setw(width) << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

std::string format_xyz(const calib::PoseRow& row) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << format_fixed(row.x, 6, 3) << "  " << format_fixed(row.y, 6, 3) << "  "
      << format_fixed(row.z, 6, 3) << " m    " << row.xyz_source;
  return out.str();
}

std::string format_rpy(const calib::PoseRow& row) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << format_fixed(row.roll, 6, 1) << "  " << format_fixed(row.pitch, 6, 1) << "  "
      << format_fixed(row.yaw, 6, 1) << " deg  " << row.rpy_source;
  return out.str();
}

std::string format_residual(double metres, double degrees) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(2) << metres << " m / ";
  if (std::fabs(degrees - std::round(degrees)) < 0.05) {
    out << std::setprecision(0) << degrees << " deg";
  } else {
    out << std::setprecision(1) << degrees << " deg";
  }
  return out.str();
}

void print_kv(std::ostream& out, bool color, const units::Status& status) {
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  key("demo");
  print_demo_value(out, color, status);
  out << '\n';
  key("stock");
  out << stock_text(status.stock) << '\n';
  key("chassis");
  if (status.chassis == units::Chassis::Active) {
    paint(out, color, kGreen, chassis_text(status.chassis));
    out << '\n';
  } else {
    out << chassis_text(status.chassis) << '\n';
  }
  key("mode");
  if (status_shows_forbidden(status)) {
    paint(out, color, kBoldRed, "forbidden");
    out << '\n';
    const char* reason = status_reason_text(status);
    if (reason[0] != '\0') {
      key("reason");
      out << reason << '\n';
    }
  } else {
    out << mode_text(status.mode) << '\n';
  }
  key("remote controller");
  if (status.remote_controller == units::RemoteController::Active) {
    paint(out, color, kGreen, remote_controller_text(status.remote_controller));
    out << '\n';
  } else {
    out << remote_controller_text(status.remote_controller) << '\n';
  }
  key("lidar");
  if (status.lidar == units::Lidar::Active) {
    paint(out, color, kGreen, lidar_text(status.lidar));
    out << '\n';
  } else if (status.lidar == units::Lidar::Degraded) {
    paint(out, color, kRed, lidar_text(status.lidar));
    out << '\n';
  } else {
    out << lidar_text(status.lidar) << '\n';
  }
  key("camera");
  if (status.camera == units::Camera::Active) {
    paint(out, color, kGreen, camera_text(status.camera));
    out << '\n';
  } else if (status.camera == units::Camera::Degraded) {
    paint(out, color, kRed, camera_text(status.camera));
    out << '\n';
  } else {
    out << camera_text(status.camera) << '\n';
  }
  key("imu");
  if (status.imu == units::Imu::Active) {
    paint(out, color, kGreen, imu_text(status.imu));
    out << '\n';
  } else if (status.imu == units::Imu::Degraded) {
    paint(out, color, kRed, imu_text(status.imu));
    if (!status.imu_msg_ok) {
      paint(out, color, kDim, " (no /imu)");
    } else if (!status.imu_odom_ok) {
      paint(out, color, kDim, " (no /imu_odom)");
    } else if (!status.imu_tf_ok) {
      paint(out, color, kDim, " (no imu_link TF)");
    }
    out << '\n';
  } else {
    out << imu_text(status.imu) << '\n';
  }
  key("odometry");
  if (status.odometry == units::Odometry::Active) {
    paint(out, color, kGreen, odometry_text(status.odometry));
    out << '\n';
  } else if (status.odometry == units::Odometry::Degraded) {
    paint(out, color, kRed, odometry_text(status.odometry));
    if (!status.odom_msg_ok) {
      paint(out, color, kDim, " (no /odom_raw)");
    } else if (!status.odom_tf_ok) {
      paint(out, color, kDim, " (no odom TF)");
    }
    out << '\n';
  } else {
    out << odometry_text(status.odometry) << '\n';
  }
  key("platform model");
  if (status.model == units::Model::Active) {
    paint(out, color, kGreen, model_text(status.model));
    out << '\n';
  } else if (status.model == units::Model::Degraded) {
    paint(out, color, kRed, model_text(status.model));
    out << '\n';
  } else {
    out << model_text(status.model) << '\n';
  }
  if (status.have_calibration) {
    key("calibration");
    print_calibration_value(out, color, status.calibration);
    out << '\n';
  }
  key("dds buffers");
  if (status.dds_buffers == units::DdsBuffers::Active) {
    paint(out, color, kGreen, "active");
    out << '\n';
  } else if (status.dds_buffers == units::DdsBuffers::Degraded) {
    std::ostringstream msg;
    msg << "degraded (rmem_max " << status.rmem_max << " < " << units::kDdsRmemMinBytes
        << ", run make deploy)";
    paint(out, color, kRed, msg.str());
    out << '\n';
  } else {
    out << "unknown\n";
  }
  key("version");
  out << kVersion << '\n';
}

void help_header(std::ostream& out, bool color, const char* title) {
  paint(out, color, kBold, title);
  out << '\n';
}

void help_cmd(std::ostream& out, bool color, const char* name, const char* desc) {
  out << "  " << pad_right(name, 11);
  paint(out, color, kDim, desc);
  out << '\n';
}

void help_status_row(std::ostream& out, bool color, const char* key, const char* value,
                     const char* desc) {
  out << "  " << pad_right(key, 18) << pad_right(value, 15) << ' ';
  paint(out, color, kDim, desc);
  out << '\n';
}

void print_error_hints(std::ostream& out) { out << "\n  t1ctl start\n  t1ctl stock\n"; }

std::string field_key_name(const std::string& name) {
  std::string out;
  out.reserve(name.size());
  for (const char c : name) {
    out.push_back(c == '_' ? ' ' : c);
  }
  return out;
}

bool field_uses_metres(const std::string& name) {
  return (name.size() >= 2 && name.compare(name.size() - 2, 2, "_x") == 0) ||
         (name.size() >= 2 && name.compare(name.size() - 2, 2, "_y") == 0) ||
         (name.size() >= 2 && name.compare(name.size() - 2, 2, "_z") == 0);
}

bool field_uses_degrees(const std::string& name) {
  return name.find("roll") != std::string::npos || name.find("pitch") != std::string::npos ||
         (name.find("yaw") != std::string::npos && name.find("sign") == std::string::npos);
}

bool field_is_sign(const std::string& name) { return name.find("sign") != std::string::npos; }

std::string format_stage_token(const std::string& stage) {
  if (stage.empty()) {
    return stage;
  }
  std::string out = stage;
  out[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
  return out;
}

std::string format_residual_delta(double before, double after) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(3) << before << " -> " << after << " m";
  return out.str();
}

std::string format_imu_residual_delta(double before, double after) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(1) << before << " -> " << after << " deg";
  return out.str();
}

std::string format_field_delta(const calib::FieldDelta& field) {
  if (field_is_sign(field.name)) {
    return field.after;
  }
  std::ostringstream out;
  out.imbue(std::locale::classic());
  if (field_uses_metres(field.name)) {
    out << std::fixed << std::setprecision(3) << field.before << " -> " << field.after << " m";
  } else if (field_uses_degrees(field.name)) {
    out << std::fixed << std::setprecision(1) << field.before << " -> " << field.after << " deg";
  } else {
    out << field.before << " -> " << field.after;
  }
  return out.str();
}

void print_field_delta_row(std::ostream& out, bool color, const calib::FieldDelta& field,
                           bool with_tag) {
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  key(field_key_name(field.name).c_str());
  const std::string value = format_field_delta(field);
  out << value;
  if (with_tag && !field.tag.empty()) {
    const std::size_t width = 18 + value.size();
    const std::size_t target = 40;
    if (width < target) {
      out << std::string(target - width, ' ');
    } else {
      out << "  ";
    }
    out << field.tag;
  }
  out << '\n';
}

std::string format_travel(double metres) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(1) << metres << " m";
  return out.str();
}

std::string format_turn(double degrees) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(0) << degrees << " deg";
  return out.str();
}

class CalibHoldPrinter {
 public:
  void update(const calib::Result& result) {
    if (!result.have_frames && !result.have_cloud_points && !result.have_scan_rays &&
        !result.have_imu_samples && !result.have_travel && !result.have_turn &&
        result.stage.empty() && result.hints.empty() && !result.clear_hints) {
      return;
    }
    if (!result.stage.empty()) {
      stage_ = result.stage;
    }
    if (result.clear_hints) {
      hints_.clear();
    }
    if (!result.hints.empty()) {
      for (const auto& hint : result.hints) {
        if (hints_.empty() || hints_.back() != hint) {
          hints_.push_back(hint);
        }
      }
    }
    if (result.have_frames) {
      frames_ = result.frames;
      have_frames_ = true;
    }
    if (result.have_cloud_points) {
      cloud_points_ = result.cloud_points;
      have_cloud_points_ = true;
    }
    if (result.have_scan_rays) {
      scan_rays_ = result.scan_rays;
      have_scan_rays_ = true;
    }
    if (result.have_imu_samples) {
      imu_samples_ = result.imu_samples;
      have_imu_samples_ = true;
    }
    if (result.have_travel) {
      travel_m_ = result.travel_m;
      have_travel_ = true;
    }
    if (result.have_turn) {
      turn_deg_ = result.turn_deg;
      have_turn_ = true;
    }
    if (!stdout_color()) {
      return;
    }
    redraw();
  }

  void finish() {
    if (!active_ || !stdout_color()) {
      return;
    }
    std::cout << "\033[" << counter_lines_ << "F\033[J";
    std::cout.flush();
    active_ = false;
    counter_lines_ = 0;
  }

 private:
  void redraw() {
    auto& out = std::cout;
    const bool color = stdout_color();
    if (active_) {
      out << "\033[" << counter_lines_ << "F";
    } else {
      active_ = true;
    }
    counter_lines_ = 0;
    auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
    if (!stage_.empty()) {
      key("stage");
      out << stage_ << '\n';
      ++counter_lines_;
    }
    if (!hints_.empty()) {
      for (const auto& hint : hints_) {
        paint(out, color, kDim, "  ");
        out << hint << '\n';
        ++counter_lines_;
      }
    }
    if (have_travel_) {
      key("travel");
      out << format_travel(travel_m_) << '\n';
      ++counter_lines_;
    }
    if (have_turn_) {
      key("turn");
      out << format_turn(turn_deg_) << '\n';
      ++counter_lines_;
    }
    if (have_frames_) {
      key("frames");
      out << frames_ << '\n';
      ++counter_lines_;
    }
    if (have_cloud_points_) {
      key("cloud points");
      out << cloud_points_ << '\n';
      ++counter_lines_;
    }
    if (have_scan_rays_) {
      key("scan rays");
      out << scan_rays_ << '\n';
      ++counter_lines_;
    }
    if (have_imu_samples_) {
      key("imu samples");
      out << imu_samples_ << '\n';
      ++counter_lines_;
    }
    out.flush();
  }

  std::string stage_;
  std::vector<std::string> hints_;
  bool have_frames_{false};
  int frames_{0};
  bool have_cloud_points_{false};
  int cloud_points_{0};
  bool have_scan_rays_{false};
  int scan_rays_{0};
  bool have_imu_samples_{false};
  int imu_samples_{0};
  bool have_travel_{false};
  double travel_m_{0};
  bool have_turn_{false};
  double turn_deg_{0};
  bool active_{false};
  int counter_lines_{0};
};

CalibHoldPrinter& calib_hold_printer() {
  static CalibHoldPrinter printer;
  return printer;
}

}  // namespace

void print_help() {
  const bool color = stdout_color();
  auto& out = std::cout;
  out << "t1ctl  " << kVersion << '\n';
  help_header(out, color, "USAGE");
  out << "  t1ctl [command]\n";
  help_header(out, color, "COMMANDS");
  help_cmd(out, color, "status", "print status (default)");
  help_cmd(out, color, "start", "start demo");
  help_cmd(out, color, "restart", "restart demo");
  help_cmd(out, color, "stock", "restore stock autostart");
  help_cmd(out, color, "mode", "set control mode");
  help_cmd(out, color, "calib", "sensor extrinsics");
  help_header(out, color, "MODE");
  out << "  " << pad_right("mode forbid", 18);
  paint(out, color, kDim, "hold motion (Forbidden)");
  out << '\n';
  out << "  " << pad_right("mode allow", 18);
  paint(out, color, kDim, "release hold, return to follow");
  out << '\n';
  out << "  " << pad_right("mode manual", 18);
  paint(out, color, kDim, "select operator pad");
  out << '\n';
  help_header(out, color, "CALIB");
  out << "  " << pad_right("calib status", 18);
  paint(out, color, kDim, "print live poses and residual");
  out << '\n';
  out << "  " << pad_right("calib corner", 18);
  paint(out, color, kDim, "camera to lidar at a corner");
  out << '\n';
  out << "  " << pad_right("calib drive", 18);
  paint(out, color, kDim, "lidar yaw from a pad run");
  out << '\n';
  out << "  " << pad_right("calib side", 18);
  paint(out, color, kDim, "transverse sign from a side object");
  out << '\n';
  out << "  " << pad_right("", 18);
  paint(out, color, kDim, "--side left|right [--timeout S]");
  out << '\n';
  out << "  " << pad_right("calib lidar", 18);
  paint(out, color, kDim, "enter lidar height and tilt");
  out << '\n';
  out << "  " << pad_right("", 18);
  paint(out, color, kDim, "--height M --pitch DEG --roll DEG");
  out << '\n';
  out << "  " << pad_right("calib camera", 18);
  paint(out, color, kDim, "enter camera height");
  out << '\n';
  out << "  " << pad_right("", 18);
  paint(out, color, kDim, "--height M");
  out << '\n';
  out << "  " << pad_right("calib accept", 18);
  paint(out, color, kDim, "keep last stage as draft");
  out << '\n';
  out << "  " << pad_right("calib reject", 18);
  paint(out, color, kDim, "drop last stage");
  out << '\n';
  out << "  " << pad_right("calib save", 18);
  paint(out, color, kDim, "write file; live after restart");
  out << '\n';
  out << "  " << pad_right("calib abort", 18);
  paint(out, color, kDim, "drop draft; file unchanged");
  out << '\n';
  help_header(out, color, "STATUS");
  help_status_row(out, color, "demo", "active", "mentorpi-t1.service is running");
  help_status_row(out, color, "", "inactive", "mentorpi-t1.service is down");
  help_status_row(out, color, "", "failed", "start or restart failed");
  help_status_row(out, color, "stock", "inactive", "start_node.service is off");
  help_status_row(out, color, "", "active", "start_node.service is on");
  help_status_row(out, color, "chassis", "active", "/vehicle/status echo succeeded");
  help_status_row(out, color, "", "inactive", "container down or no /vehicle/status");
  help_status_row(out, color, "mode", "follow", "person follow is selected");
  help_status_row(out, color, "", "manual", "operator pad is selected");
  help_status_row(out, color, "", "forbidden", "motion held; chassis zeros");
  help_status_row(out, color, "reason", "operator", "host command t1ctl mode forbid");
  help_status_row(out, color, "", "(topic)", "critical contour signal missing");
  help_status_row(out, color, "remote controller", "active", "pad receiver has a link");
  help_status_row(out, color, "", "inactive", "no pad link");
  help_status_row(out, color, "lidar", "active", "/scan echo and lidar_frame TF ready");
  help_status_row(out, color, "", "degraded", "demo up but scan or TF missing");
  help_status_row(out, color, "", "inactive", "container down or probe skipped");
  help_status_row(out, color, "camera", "active", "color, depth, and depth_camera_link TF ready");
  help_status_row(out, color, "", "degraded", "demo up but color, depth, or TF missing");
  help_status_row(out, color, "", "inactive", "container down or probe skipped");
  help_status_row(out, color, "imu", "active", "/imu, /imu_odom, and imu_link TF ready");
  help_status_row(out, color, "", "degraded", "demo up but IMU topic, odom stream, or TF missing");
  help_status_row(out, color, "", "inactive", "container down or probe skipped");
  help_status_row(out, color, "odometry", "active", "/odom_raw and odom TF ready");
  help_status_row(out, color, "", "degraded", "demo up but /odom_raw or TF missing");
  help_status_row(out, color, "", "inactive", "container down or probe skipped");
  help_status_row(out, color, "platform model", "active",
                  "/robot_description and required model TF ready");
  help_status_row(out, color, "", "degraded", "demo up but description or TF missing");
  help_status_row(out, color, "", "inactive", "container down or probe skipped");
  help_status_row(out, color, "calibration", "factory", "poses from the platform model");
  help_status_row(out, color, "", "file", "saved file applied at start");
  help_status_row(out, color, "", "unused", "file present but not applied");
  help_status_row(out, color, "dds buffers", "active",
                  "camera frames get the 8 MB socket buffer (SD029)");
  help_status_row(out, color, "", "degraded", "rmem_max below 8 MB: camera frames drop on the Pi");
  help_status_row(out, color, "version", kVersion, "t1ctl package version");
  help_header(out, color, "FLAGS");
  out << "  -h, --help     ";
  paint(out, color, kDim, "this help");
  out << "    -V, --version  ";
  paint(out, color, kDim, "print version");
  out << '\n';
}

void print_version() { std::cout << "t1ctl " << kVersion << '\n'; }

void print_status(const units::Status& status) {
  const bool color = stdout_color();
  print_kv(std::cout, color, status);
  if (status.demo == units::Demo::Inactive && status.stock == units::Stock::Inactive) {
    std::cout << "\n  t1ctl start\n";
  } else if (status.demo == units::Demo::Failed) {
    std::cout << '\n';
    std::cout.flush();
    paint(std::cerr, stderr_color(), kBoldRed, "error:");
    std::cerr << " demo start failed\n";
    std::cerr.flush();
    print_error_hints(std::cout);
  } else if (status_shows_forbidden(status)) {
    std::cout << "\n  t1ctl mode allow\n";
  } else if (status.have_calibration && status.calibration == units::Calibration::Unused) {
    std::cout << "\n  t1ctl calib\n";
  }
}

void print_started() { std::cout << "Started.\n"; }

void print_restarted() { std::cout << "Restarted.\n"; }

void print_stock() { std::cout << "Restored stock autostart.\n"; }

void print_action_error() {
  std::cout.flush();
  paint(std::cerr, stderr_color(), kBoldRed, "error:");
  std::cerr << " demo start failed\n";
  std::cerr.flush();
  print_error_hints(std::cout);
}

void print_mode_changed(const units::ModeChange& change) {
  const bool color = stdout_color();
  auto& out = std::cout;
  switch (change.to) {
    case units::Mode::Forbidden:
      out << "Motion held.\n";
      break;
    case units::Mode::Follow:
      out << "Motion allowed.\n";
      break;
    case units::Mode::Manual:
      out << "Manual.\n";
      break;
  }
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  key("mode");
  if (change.have_from) {
    out << mode_text(change.from) << " -> ";
  }
  if (change.to == units::Mode::Forbidden) {
    paint(out, color, kBoldRed, "forbidden");
  } else {
    out << mode_text(change.to);
  }
  out << '\n';
  if (change.to == units::Mode::Forbidden && !change.reason.empty()) {
    key("reason");
    out << change.reason << '\n';
  }
}

void print_mode_error(const units::Status& status, const std::string& detail) {
  const bool color = stdout_color();
  auto& out = std::cout;
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  key("demo");
  print_demo_value(out, color, status);
  out << '\n';
  key("stock");
  out << stock_text(status.stock) << '\n';
  out << '\n';
  out.flush();
  paint(std::cerr, stderr_color(), kBoldRed, "error:");
  std::cerr << " mode change failed\n";
  if (!detail.empty()) {
    std::istringstream lines(detail);
    std::string line;
    while (std::getline(lines, line)) {
      if (!line.empty()) {
        std::cerr << "  " << line << '\n';
      }
    }
  }
  std::cerr.flush();
  out << "\n  t1ctl start\n";
}

void print_calib(const calib::Show& show) {
  const bool color = stdout_color();
  auto& out = std::cout;
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  key("source");
  print_calibration_value(out, color, show.source);
  out << '\n';
  if (show.source == units::Calibration::Unused && !show.reason.empty()) {
    key("reason");
    out << show.reason << '\n';
  }
  if (show.have_residual) {
    key("residual");
    out << format_residual(show.residual_m, show.residual_deg) << '\n';
  }
  if (show.have_camera) {
    key("camera xyz");
    out << format_xyz(show.camera) << '\n';
    key("camera rpy");
    out << format_rpy(show.camera) << '\n';
  }
  if (show.have_lidar) {
    key("lidar xyz");
    out << format_xyz(show.lidar) << '\n';
    key("lidar rpy");
    out << format_rpy(show.lidar) << '\n';
  }
  if (show.have_imu) {
    key("imu xyz");
    out << format_xyz(show.imu) << '\n';
    key("imu rpy");
    out << format_rpy(show.imu) << '\n';
  }
  if (show.source == units::Calibration::Factory || show.source == units::Calibration::Unused) {
    out << "\n  t1ctl calib corner\n";
  }
}

void print_calib_error(const calib::Show& show) {
  const bool color = stdout_color();
  auto& out = std::cout;
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  key("demo");
  units::Status demo_status;
  demo_status.demo = show.demo;
  demo_status.stock = show.stock;
  print_demo_value(out, color, demo_status);
  out << '\n';
  key("stock");
  out << stock_text(show.stock) << '\n';
  out << '\n';
  out.flush();
  paint(std::cerr, stderr_color(), kBoldRed, "error:");
  std::cerr << " calib show failed\n";
  if (!show.detail.empty()) {
    std::istringstream lines(show.detail);
    std::string line;
    while (std::getline(lines, line)) {
      if (!line.empty()) {
        std::cerr << "  " << line << '\n';
      }
    }
  }
  std::cerr.flush();
  out << "\n  t1ctl start\n";
}

void print_calib_hold_line(const calib::Result& result) { calib_hold_printer().update(result); }

void print_calib_hold(const calib::Result& result) {
  const bool color = stdout_color();
  auto& out = std::cout;
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  if (!result.stage.empty()) {
    key("stage");
    out << result.stage << '\n';
  }
  if (!result.hints.empty()) {
    for (const auto& hint : result.hints) {
      paint(out, color, kDim, "  ");
      out << hint << '\n';
    }
  }
  if (result.have_travel) {
    key("travel");
    out << format_travel(result.travel_m) << '\n';
  }
  if (result.have_turn) {
    key("turn");
    out << format_turn(result.turn_deg) << '\n';
  }
  if (result.have_frames) {
    key("frames");
    out << result.frames << '\n';
  }
  if (result.have_cloud_points) {
    key("cloud points");
    out << result.cloud_points << '\n';
  }
  if (result.have_scan_rays) {
    key("scan rays");
    out << result.scan_rays << '\n';
  }
  if (result.have_imu_samples) {
    key("imu samples");
    out << result.imu_samples << '\n';
  }
}

void print_calib_hold_finish() { calib_hold_printer().finish(); }

void print_calib_proposal(const calib::Result& result) {
  const bool color = stdout_color();
  auto& out = std::cout;
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  if (!result.stage.empty()) {
    key("stage");
    out << result.stage << '\n';
  }
  if (!result.cause.empty()) {
    key("cause");
    out << result.cause << '\n';
  }
  if (result.have_residual_before && result.have_residual_after) {
    key("residual");
    out << format_residual_delta(result.residual_before, result.residual_after) << '\n';
  }
  if (result.have_imu_residual_before && result.have_imu_residual_after) {
    key("imu residual");
    out << format_imu_residual_delta(result.imu_residual_before, result.imu_residual_after) << '\n';
  }
  for (const auto& field : result.fields) {
    print_field_delta_row(out, color, field, false);
  }
  if (!result.detail.empty()) {
    std::istringstream lines(result.detail);
    std::string line;
    while (std::getline(lines, line)) {
      if (line.empty()) {
        continue;
      }
      key("detail");
      out << line << '\n';
    }
  }
  out << "\n  t1ctl calib accept\n  t1ctl calib reject\n";
}

void print_calib_side(const calib::Result& result) {
  const bool color = stdout_color();
  auto& out = std::cout;
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  if (!result.stage.empty()) {
    key("stage");
    out << result.stage << '\n';
  }
  for (const auto& item : result.observed) {
    std::string kind = item;
    std::string value;
    const auto space = item.find(' ');
    if (space != std::string::npos) {
      kind = item.substr(0, space);
      value = item.substr(space + 1);
    }
    key(kind.c_str());
    out << value << '\n';
  }
  if (!result.layer.empty()) {
    key("layer");
    out << result.layer << '\n';
  }
  if (!result.frame_copy_error.empty()) {
    key("frame");
    out << result.frame_copy_error << '\n';
  } else if (!result.host_frame.empty()) {
    key("frame");
    out << result.host_frame << '\n';
  } else if (!result.frame.empty()) {
    key("frame");
    out << result.frame << '\n';
  }
  for (const auto& field : result.fields) {
    print_field_delta_row(out, color, field, false);
  }
  if (!result.fields.empty()) {
    out << "\n  t1ctl calib accept\n  t1ctl calib reject\n";
  }
}

void print_calib_save(const calib::Result& result) {
  const bool color = stdout_color();
  auto& out = std::cout;
  for (const auto& field : result.fields) {
    print_field_delta_row(out, color, field, true);
  }
  out << "Calibration saved.\n";
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  key("live");
  out << "after restart\n";
  out << "\n  t1ctl restart\n";
}

void print_calib_stage_error(const calib::Result& result) {
  print_calib_hold_finish();
  const bool color = stdout_color();
  auto& out = std::cout;
  auto key = [&](const char* name) { paint(out, color, kDim, pad_right(name, 18)); };
  key("demo");
  units::Status demo_status;
  demo_status.demo = result.demo;
  demo_status.stock = result.stock;
  print_demo_value(out, color, demo_status);
  out << '\n';
  key("stock");
  out << stock_text(result.stock) << '\n';
  out << '\n';
  out.flush();
  const std::string stage = result.stage.empty() ? "calib" : result.stage;
  paint(std::cerr, stderr_color(), kBoldRed, "error:");
  std::cerr << " calib " << stage << " failed\n";
  if (!result.detail.empty()) {
    std::istringstream lines(result.detail);
    std::string line;
    while (std::getline(lines, line)) {
      if (!line.empty()) {
        std::cerr << "  " << line << '\n';
      }
    }
  }
  std::cerr.flush();
  if (result.demo == units::Demo::Inactive) {
    out << "\n  t1ctl start\n";
  } else if (!result.stage.empty()) {
    out << "\n  t1ctl calib " << result.stage << '\n';
  } else {
    out << "\n  t1ctl calib corner\n";
  }
}

void print_calib_stage_phrase(const calib::Result& result, bool kept) {
  const std::string stage = format_stage_token(result.stage);
  if (kept) {
    std::cout << stage << " kept.\n";
  } else {
    std::cout << stage << " dropped.\n";
  }
}

}  // namespace ui
