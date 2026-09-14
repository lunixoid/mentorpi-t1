#include "calib.hpp"

#include <cctype>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace calib {
namespace {

void trim_ws(std::string& text) {
  std::size_t start = 0;
  while (start < text.size()) {
    const char c = text[start];
    if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
      break;
    }
    ++start;
  }
  if (start > 0) {
    text.erase(0, start);
  }
  while (!text.empty()) {
    const char c = text.back();
    if (c != '\n' && c != '\r' && c != ' ' && c != '\t') {
      break;
    }
    text.pop_back();
  }
}

bool parse_source_token(const std::string& text, units::Calibration* source) {
  if (text == "factory") {
    *source = units::Calibration::Factory;
    return true;
  }
  if (text == "file") {
    *source = units::Calibration::File;
    return true;
  }
  if (text == "unused") {
    *source = units::Calibration::Unused;
    return true;
  }
  return false;
}

bool container_running(units::ProcessRunner& runner, const char* name) {
  std::string out;
  if (runner.run({"docker", "inspect", "-f", "{{.State.Running}}", name}, &out) != 0) {
    return false;
  }
  trim_ws(out);
  return out == "true";
}

void append_detail(std::string* detail, const std::string& line) {
  if (detail == nullptr || line.empty()) {
    return;
  }
  if (!detail->empty()) {
    *detail += '\n';
  }
  *detail += line;
}

bool parse_pose_line(const std::string& line, Show* show) {
  if (line.rfind("pose:", 0) != 0) {
    return false;
  }
  std::istringstream ss(line.substr(5));
  std::string sensor;
  std::string kind;
  double a = 0;
  double b = 0;
  double c = 0;
  std::string source;
  if (!(ss >> sensor >> kind >> a >> b >> c >> source)) {
    return false;
  }
  PoseRow* row = nullptr;
  bool* have = nullptr;
  if (sensor == "camera") {
    row = &show->camera;
    have = &show->have_camera;
  } else if (sensor == "lidar") {
    row = &show->lidar;
    have = &show->have_lidar;
  } else if (sensor == "imu") {
    row = &show->imu;
    have = &show->have_imu;
  } else {
    return false;
  }
  if (kind == "xyz") {
    row->x = a;
    row->y = b;
    row->z = c;
    row->xyz_source = source;
    *have = true;
    return true;
  }
  if (kind == "rpy") {
    row->roll = a;
    row->pitch = b;
    row->yaw = c;
    row->rpy_source = source;
    *have = true;
    return true;
  }
  return false;
}

std::vector<std::string> docker_bash_args(const char* script) {
  return {"docker",    "exec", "-u",  "ubuntu", "-w", "/home/ubuntu", units::kOursContainer,
          "/bin/bash", "-c",   script};
}

constexpr const char* kCalibPrefix =
    "export PYTHONUNBUFFERED=1; "
    "source /home/ubuntu/ros2_ws/.hiwonderrc >/dev/null 2>/dev/null; "
    "export ROS_LOCALHOST_ONLY=0; "
    "source /home/ubuntu/mentorpi_t1_ws/install/setup.bash >/dev/null 2>/dev/null; "
    "ros2 run mentorpi_calibration calib";

constexpr const char* kCalibShowScript =
    "export PYTHONUNBUFFERED=1; "
    "source /home/ubuntu/ros2_ws/.hiwonderrc >/dev/null 2>/dev/null; "
    "export ROS_LOCALHOST_ONLY=0; "
    "source /home/ubuntu/mentorpi_t1_ws/install/setup.bash >/dev/null 2>/dev/null; "
    "ros2 run mentorpi_calibration calib show";

std::string shell_single_quote(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 2);
  out.push_back('\'');
  for (const char c : text) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out.push_back(c);
    }
  }
  out.push_back('\'');
  return out;
}

bool needs_shell_quote(const std::string& text) {
  if (text.empty()) {
    return true;
  }
  for (const char c : text) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == '_')) {
      return true;
    }
  }
  return false;
}

std::string shell_arg(const std::string& text) {
  if (!needs_shell_quote(text)) {
    return text;
  }
  return shell_single_quote(text);
}

std::string calib_script(const std::vector<std::string>& subcommand) {
  std::ostringstream ss;
  ss << kCalibPrefix;
  for (const auto& arg : subcommand) {
    ss << ' ' << shell_arg(arg);
  }
  return ss.str();
}

void fill_host(units::ProcessRunner& runner, Result* result) {
  const units::Status host = units::query_host(runner);
  result->demo = host.demo;
  result->stock = host.stock;
}

void fill_host(units::ProcessRunner& runner, Show* show) {
  const units::Status host = units::query_host(runner);
  show->demo = host.demo;
  show->stock = host.stock;
}

bool parse_int_token(const std::string& text, int* value) {
  if (text.empty()) {
    return false;
  }
  char* end = nullptr;
  const long n = std::strtol(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  *value = static_cast<int>(n);
  return true;
}

bool parse_double_token(const std::string& text, double* value) {
  if (text.empty()) {
    return false;
  }
  char* end = nullptr;
  const double n = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0') {
    return false;
  }
  *value = n;
  return true;
}

void copy_side_frame(units::ProcessRunner& runner, Result* result) {
  if (result == nullptr || result->frame.empty()) {
    return;
  }
  // config/ is bind-mounted on the host; copy to /tmp so docker cp does not write
  // the PNG onto the same inode.
  result->host_frame = kHostSideFramePath;
  const std::string source = std::string(units::kOursContainer) + ":" + result->frame;
  std::string out;
  const int code = runner.run({"docker", "cp", source, result->host_frame}, &out);
  if (code == 0) {
    result->frame_copied = true;
    result->frame_copy_error.clear();
    return;
  }
  result->frame_copied = false;
  result->host_frame.clear();
  result->frame_copy_error = "not found in container";
}

bool parse_field_line(const std::string& line, FieldDelta* field) {
  if (line.rfind("field:", 0) != 0) {
    return false;
  }
  std::istringstream ss(line.substr(6));
  std::string name;
  std::string before;
  std::string after;
  std::string tag;
  if (!(ss >> name >> before >> after)) {
    return false;
  }
  ss >> tag;
  field->name = name;
  field->before = before;
  field->after = after;
  field->tag = tag;
  return true;
}

}  // namespace

bool parse_result_line(const std::string& line, Result* result) {
  if (result == nullptr) {
    return false;
  }
  std::string trimmed = line;
  trim_ws(trimmed);
  if (trimmed.empty()) {
    return false;
  }
  bool parsed = false;
  if (trimmed == "T1CTL_CALIB_OK=1") {
    result->ok = true;
    result->have_ok = true;
    parsed = true;
  } else if (trimmed == "T1CTL_CALIB_OK=0") {
    result->ok = false;
    result->have_ok = true;
    parsed = true;
  } else if (trimmed.rfind("stage:", 0) == 0) {
    result->stage = trimmed.substr(6);
    trim_ws(result->stage);
    parsed = true;
  } else if (trimmed.rfind("hint:", 0) == 0) {
    std::string value = trimmed.substr(5);
    trim_ws(value);
    if (value.empty()) {
      result->clear_hints = true;
      result->hints.clear();
    } else {
      result->hints.push_back(value);
    }
    parsed = true;
  } else if (trimmed.rfind("travel:", 0) == 0) {
    double value = 0;
    if (parse_double_token(trimmed.substr(7), &value)) {
      result->have_travel = true;
      result->travel_m = value;
      parsed = true;
    }
  } else if (trimmed.rfind("turn:", 0) == 0) {
    double value = 0;
    if (parse_double_token(trimmed.substr(5), &value)) {
      result->have_turn = true;
      result->turn_deg = value;
      parsed = true;
    }
  } else if (trimmed.rfind("frames:", 0) == 0) {
    int value = 0;
    if (parse_int_token(trimmed.substr(7), &value)) {
      result->have_frames = true;
      result->frames = value;
      parsed = true;
    }
  } else if (trimmed.rfind("cloud_points:", 0) == 0) {
    int value = 0;
    if (parse_int_token(trimmed.substr(13), &value)) {
      result->have_cloud_points = true;
      result->cloud_points = value;
      parsed = true;
    }
  } else if (trimmed.rfind("scan_rays:", 0) == 0) {
    int value = 0;
    if (parse_int_token(trimmed.substr(10), &value)) {
      result->have_scan_rays = true;
      result->scan_rays = value;
      parsed = true;
    }
  } else if (trimmed.rfind("imu_samples:", 0) == 0) {
    int value = 0;
    if (parse_int_token(trimmed.substr(12), &value)) {
      result->have_imu_samples = true;
      result->imu_samples = value;
      parsed = true;
    }
  } else if (trimmed.rfind("residual_before:", 0) == 0) {
    double value = 0;
    if (parse_double_token(trimmed.substr(16), &value)) {
      result->have_residual_before = true;
      result->residual_before = value;
      parsed = true;
    }
  } else if (trimmed.rfind("residual_after:", 0) == 0) {
    double value = 0;
    if (parse_double_token(trimmed.substr(15), &value)) {
      result->have_residual_after = true;
      result->residual_after = value;
      parsed = true;
    }
  } else if (trimmed.rfind("imu_residual_before:", 0) == 0) {
    double value = 0;
    if (parse_double_token(trimmed.substr(20), &value)) {
      result->have_imu_residual_before = true;
      result->imu_residual_before = value;
      parsed = true;
    }
  } else if (trimmed.rfind("imu_residual_after:", 0) == 0) {
    double value = 0;
    if (parse_double_token(trimmed.substr(19), &value)) {
      result->have_imu_residual_after = true;
      result->imu_residual_after = value;
      parsed = true;
    }
  } else if (trimmed.rfind("field:", 0) == 0) {
    FieldDelta field;
    if (parse_field_line(trimmed, &field)) {
      result->fields.push_back(field);
      parsed = true;
    }
  } else if (trimmed.rfind("detail:", 0) == 0) {
    std::string value = trimmed.substr(7);
    trim_ws(value);
    append_detail(&result->detail, value);
    parsed = true;
  } else if (trimmed.rfind("reason:", 0) == 0) {
    result->message = trimmed.substr(7);
    trim_ws(result->message);
    parsed = true;
  } else if (trimmed.rfind("cause:", 0) == 0) {
    result->cause = trimmed.substr(6);
    trim_ws(result->cause);
    parsed = true;
  } else if (trimmed.rfind("draft:", 0) == 0) {
    result->draft = trimmed.substr(6);
    trim_ws(result->draft);
    parsed = true;
  } else if (trimmed.rfind("observed:", 0) == 0) {
    std::string value = trimmed.substr(9);
    trim_ws(value);
    if (!value.empty()) {
      result->observed.push_back(value);
    }
    parsed = true;
  } else if (trimmed.rfind("layer:", 0) == 0) {
    result->layer = trimmed.substr(6);
    trim_ws(result->layer);
    parsed = true;
  } else if (trimmed.rfind("frame:", 0) == 0) {
    result->frame = trimmed.substr(6);
    trim_ws(result->frame);
    parsed = true;
  }
  return parsed;
}

bool parse_result(const std::string& text, Result* result) {
  if (result == nullptr) {
    return false;
  }
  Result parsed;
  parsed.demo = result->demo;
  parsed.stock = result->stock;
  bool have_ok = false;
  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    parse_result_line(line, &parsed);
    if (parsed.have_ok) {
      have_ok = true;
    }
  }
  if (!have_ok) {
    return false;
  }
  *result = parsed;
  return true;
}

bool parse_show(const std::string& text, Show* show) {
  if (show == nullptr) {
    return false;
  }
  Show parsed;
  parsed.demo = show->demo;
  parsed.stock = show->stock;
  bool have_ok = false;
  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    trim_ws(line);
    if (line.empty()) {
      continue;
    }
    if (line == "T1CTL_CALIB_OK=1") {
      parsed.ok = true;
      have_ok = true;
      continue;
    }
    if (line == "T1CTL_CALIB_OK=0") {
      parsed.ok = false;
      have_ok = true;
      continue;
    }
    if (line.rfind("source:", 0) == 0) {
      std::string value = line.substr(7);
      trim_ws(value);
      if (parse_source_token(value, &parsed.source)) {
        parsed.have_source = true;
      }
      continue;
    }
    if (line.rfind("reason:", 0) == 0) {
      parsed.reason = line.substr(7);
      trim_ws(parsed.reason);
      continue;
    }
    if (line.rfind("residual:", 0) == 0) {
      std::istringstream rs(line.substr(9));
      double metres = 0;
      double degrees = 0;
      if (rs >> metres >> degrees) {
        parsed.have_residual = true;
        parsed.residual_m = metres;
        parsed.residual_deg = degrees;
      }
      continue;
    }
    if (line.rfind("detail:", 0) == 0) {
      std::string value = line.substr(7);
      trim_ws(value);
      append_detail(&parsed.detail, value);
      continue;
    }
    parse_pose_line(line, &parsed);
  }
  if (!have_ok) {
    return false;
  }
  *show = parsed;
  return true;
}

bool run_command(units::ProcessRunner& runner, const std::vector<std::string>& subcommand,
                 Result* result, const std::function<void(const std::string& line)>& on_line) {
  if (result == nullptr) {
    return false;
  }
  *result = Result{};
  fill_host(runner, result);
  if (!container_running(runner, units::kOursContainer)) {
    result->ok = false;
    result->have_ok = false;
    result->detail = "container mentorpi-t1 is not running";
    return false;
  }
  const std::string script = calib_script(subcommand);
  std::string out;
  auto line_handler = [&](const std::string& line) {
    parse_result_line(line, result);
    if (on_line) {
      on_line(line);
    }
  };
  runner.run_lines(docker_bash_args(script.c_str()), line_handler, &out);
  Result parsed = *result;
  if (!parse_result(out, &parsed)) {
    result->ok = false;
    if (result->detail.empty()) {
      result->detail = "calib helper failed";
    }
    return false;
  }
  parsed.demo = result->demo;
  parsed.stock = result->stock;
  *result = parsed;
  if (!result->ok) {
    if (result->detail.empty() && !result->message.empty()) {
      result->detail = result->message;
    }
    return false;
  }
  if (result->stage == "side") {
    copy_side_frame(runner, result);
  }
  return true;
}

bool run_command(const std::vector<std::string>& subcommand, Result* result,
                 const std::function<void(const std::string& line)>& on_line) {
  return run_command(units::process_runner(), subcommand, result, on_line);
}

bool query(units::ProcessRunner& runner, Show* show) {
  if (show == nullptr) {
    return false;
  }
  *show = Show{};
  fill_host(runner, show);
  if (!container_running(runner, units::kOursContainer)) {
    show->ok = false;
    show->detail = "container mentorpi-t1 is not running";
    return false;
  }
  std::string out;
  runner.run(docker_bash_args(kCalibShowScript), &out);
  Show parsed = *show;
  if (!parse_show(out, &parsed) || !parsed.ok || !parsed.have_source) {
    show->ok = false;
    if (parsed.detail.empty()) {
      show->detail = "calib helper failed";
    } else {
      show->detail = parsed.detail;
    }
    show->have_camera = false;
    show->have_lidar = false;
    show->have_imu = false;
    return false;
  }
  *show = parsed;
  return true;
}

bool query(Show* show) { return query(units::process_runner(), show); }

}  // namespace calib
