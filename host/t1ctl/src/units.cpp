#include "units.hpp"

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

#include "ros_mode_embed.hpp"
#include "ros_probe_embed.hpp"

namespace units {
namespace {

bool posix_have_executable(const char* name) {
  if (name == nullptr || name[0] == '\0') {
    return false;
  }
  if (std::strchr(name, '/') != nullptr) {
    return ::access(name, X_OK) == 0;
  }
  const char* path = std::getenv("PATH");
  if (path == nullptr) {
    return false;
  }
  std::stringstream ss(path);
  std::string dir;
  while (std::getline(ss, dir, ':')) {
    if (dir.empty()) {
      dir = ".";
    }
    const std::string candidate = dir + "/" + name;
    if (::access(candidate.c_str(), X_OK) == 0) {
      return true;
    }
  }
  return false;
}

void trim_trailing_ws(std::string& text) {
  while (!text.empty()) {
    const char c = text.back();
    if (c != '\n' && c != '\r' && c != ' ' && c != '\t') {
      break;
    }
    text.pop_back();
  }
}

void emit_complete_lines(std::string* buffer,
                         const std::function<void(const std::string& line)>& on_line) {
  if (buffer == nullptr) {
    return;
  }
  std::size_t pos = 0;
  while (pos < buffer->size()) {
    const std::size_t end = buffer->find('\n', pos);
    if (end == std::string::npos) {
      break;
    }
    std::string line = buffer->substr(pos, end - pos);
    trim_trailing_ws(line);
    if (on_line) {
      on_line(line);
    }
    pos = end + 1;
  }
  if (pos > 0) {
    buffer->erase(0, pos);
  }
}

int posix_run(const std::vector<std::string>& args, std::string* stdout_out) {
  if (args.empty()) {
    return 127;
  }
  int pipefd[2] = {-1, -1};
  if (stdout_out != nullptr && ::pipe(pipefd) != 0) {
    return 127;
  }
  const pid_t pid = ::fork();
  if (pid < 0) {
    if (pipefd[0] >= 0) {
      ::close(pipefd[0]);
      ::close(pipefd[1]);
    }
    return 127;
  }
  if (pid == 0) {
    if (stdout_out != nullptr) {
      ::close(pipefd[0]);
      ::dup2(pipefd[1], STDOUT_FILENO);
      ::close(pipefd[1]);
    } else {
      const int devnull = ::open("/dev/null", O_WRONLY);
      if (devnull >= 0) {
        ::dup2(devnull, STDOUT_FILENO);
        ::close(devnull);
      }
    }
    const int devnull_err = ::open("/dev/null", O_WRONLY);
    if (devnull_err >= 0) {
      ::dup2(devnull_err, STDERR_FILENO);
      ::close(devnull_err);
    }
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (const auto& arg : args) {
      argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);
    ::execvp(argv[0], argv.data());
    _exit(127);
  }
  if (stdout_out != nullptr) {
    ::close(pipefd[1]);
    stdout_out->clear();
    char buf[256];
    ssize_t n = 0;
    while ((n = ::read(pipefd[0], buf, sizeof(buf))) > 0) {
      stdout_out->append(buf, static_cast<std::size_t>(n));
    }
    ::close(pipefd[0]);
  }
  int st = 0;
  while (::waitpid(pid, &st, 0) < 0) {
    if (errno != EINTR) {
      return 127;
    }
  }
  if (WIFEXITED(st)) {
    return WEXITSTATUS(st);
  }
  return 127;
}

int posix_run_lines(const std::vector<std::string>& args,
                    const std::function<void(const std::string& line)>& on_line,
                    std::string* stdout_out) {
  if (args.empty()) {
    return 127;
  }
  int pipefd[2] = {-1, -1};
  if (::pipe(pipefd) != 0) {
    return 127;
  }
  const pid_t pid = ::fork();
  if (pid < 0) {
    ::close(pipefd[0]);
    ::close(pipefd[1]);
    return 127;
  }
  if (pid == 0) {
    ::close(pipefd[0]);
    ::dup2(pipefd[1], STDOUT_FILENO);
    ::close(pipefd[1]);
    const int devnull_err = ::open("/dev/null", O_WRONLY);
    if (devnull_err >= 0) {
      ::dup2(devnull_err, STDERR_FILENO);
      ::close(devnull_err);
    }
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (const auto& arg : args) {
      argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);
    ::execvp(argv[0], argv.data());
    _exit(127);
  }
  ::close(pipefd[1]);
  std::string captured;
  std::string line_buffer;
  char buf[256];
  ssize_t n = 0;
  while ((n = ::read(pipefd[0], buf, sizeof(buf))) > 0) {
    const std::string chunk(buf, static_cast<std::size_t>(n));
    captured += chunk;
    line_buffer += chunk;
    emit_complete_lines(&line_buffer, on_line);
  }
  ::close(pipefd[0]);
  if (!line_buffer.empty()) {
    trim_trailing_ws(line_buffer);
    if (!line_buffer.empty() && on_line) {
      on_line(line_buffer);
    }
    line_buffer.clear();
  }
  if (stdout_out != nullptr) {
    *stdout_out = captured;
  }
  int st = 0;
  while (::waitpid(pid, &st, 0) < 0) {
    if (errno != EINTR) {
      return 127;
    }
  }
  if (WIFEXITED(st)) {
    return WEXITSTATUS(st);
  }
  return 127;
}

class PosixProcessRunner final : public ProcessRunner {
 public:
  int run(const std::vector<std::string>& args, std::string* stdout_out) override {
    return posix_run(args, stdout_out);
  }
  int run_lines(const std::vector<std::string>& args,
                const std::function<void(const std::string& line)>& on_line,
                std::string* stdout_out) override {
    return posix_run_lines(args, on_line, stdout_out);
  }
  bool have_executable(const char* name) const override { return posix_have_executable(name); }
};

PosixProcessRunner& posix_runner() {
  static PosixProcessRunner runner;
  return runner;
}

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
  trim_trailing_ws(text);
}

bool parse_int_exact(const std::string& text, int* n) {
  if (text.empty()) {
    return false;
  }
  std::size_t idx = 0;
  try {
    *n = std::stoi(text, &idx);
  } catch (...) {
    return false;
  }
  return idx == text.size();
}

bool parse_bool_token(const std::string& text, bool* value) {
  if (text == "true" || text == "True" || text == "TRUE" || text == "1") {
    *value = true;
    return true;
  }
  if (text == "false" || text == "False" || text == "FALSE" || text == "0") {
    *value = false;
    return true;
  }
  return false;
}

constexpr int kControlStateForbidden = 0;
constexpr int kControlStateManual = 1;
constexpr int kControlStateAutoFollow = 2;

bool parse_mode_token(const std::string& state, Mode* mode) {
  if (state == "MANUAL") {
    *mode = Mode::Manual;
    return true;
  }
  if (state == "AUTO_FOLLOW") {
    *mode = Mode::Follow;
    return true;
  }
  if (state == "FORBIDDEN") {
    *mode = Mode::Forbidden;
    return true;
  }
  int n = 0;
  if (!parse_int_exact(state, &n)) {
    return false;
  }
  switch (n) {
    case kControlStateManual:
      *mode = Mode::Manual;
      return true;
    case kControlStateForbidden:
      *mode = Mode::Forbidden;
      return true;
    case kControlStateAutoFollow:
      *mode = Mode::Follow;
      return true;
    default:
      return false;
  }
}

bool container_running(ProcessRunner& runner, const char* name) {
  std::string out;
  if (runner.run({"docker", "inspect", "-f", "{{.State.Running}}", name}, &out) != 0) {
    return false;
  }
  trim_trailing_ws(out);
  return out == "true";
}

std::string shell_single_quote(const char* text) {
  std::string out;
  out.reserve(std::strlen(text) + 2);
  out.push_back('\'');
  for (const char* p = text; *p != '\0'; ++p) {
    if (*p == '\'') {
      out += "'\\''";
    } else {
      out.push_back(*p);
    }
  }
  out.push_back('\'');
  return out;
}

std::string ros_probe_script() {
  static const std::string script =
      std::string(
          "export PYTHONUNBUFFERED=1; "
          "source /home/ubuntu/ros2_ws/.hiwonderrc >/dev/null 2>/dev/null; "
          "export ROS_LOCALHOST_ONLY=0; "
          "source /home/ubuntu/mentorpi_t1_ws/install/setup.bash >/dev/null 2>/dev/null; "
          "python3 -c ") +
      shell_single_quote(kRosProbePy);
  return script;
}

std::vector<std::string> ros_probe_args() {
  return {"docker",       "exec",         "-u",        "ubuntu", "-w",
          "/home/ubuntu", kOursContainer, "/bin/bash", "-c",     ros_probe_script()};
}

uint8_t mode_command_target(ModeCommand command) {
  switch (command) {
    case ModeCommand::Forbid:
      return static_cast<uint8_t>(kControlStateForbidden);
    case ModeCommand::Manual:
      return static_cast<uint8_t>(kControlStateManual);
    case ModeCommand::Allow:
      return static_cast<uint8_t>(kControlStateAutoFollow);
  }
  return static_cast<uint8_t>(kControlStateAutoFollow);
}

std::string ros_mode_script(uint8_t target) {
  return std::string(
             "export PYTHONUNBUFFERED=1; "
             "export T1CTL_MODE_TARGET=") +
         std::to_string(static_cast<int>(target)) +
         "; source /home/ubuntu/ros2_ws/.hiwonderrc >/dev/null 2>/dev/null; "
         "export ROS_LOCALHOST_ONLY=0; "
         "source /home/ubuntu/mentorpi_t1_ws/install/setup.bash >/dev/null 2>/dev/null; "
         "python3 -c " +
         shell_single_quote(kRosModePy);
}

std::vector<std::string> ros_mode_args(uint8_t target) {
  return {"docker",       "exec",         "-u",        "ubuntu", "-w",
          "/home/ubuntu", kOursContainer, "/bin/bash", "-c",     ros_mode_script(target)};
}

void append_detail_line(std::string* detail, const std::string& line) {
  if (detail == nullptr || line.empty()) {
    return;
  }
  if (!detail->empty()) {
    *detail += '\n';
  }
  *detail += line;
}

void ensure_motion_not_allowed(std::string* detail) {
  if (detail == nullptr) {
    return;
  }
  if (detail->find("motion is not allowed") == std::string::npos) {
    append_detail_line(detail, "motion is not allowed");
  }
}

void fail_mode_change(ModeChange* change, const std::string& detail) {
  if (change == nullptr) {
    return;
  }
  change->ok = false;
  change->detail = detail;
  ensure_motion_not_allowed(&change->detail);
}

void fill_systemd(ProcessRunner& runner, Status& status) {
  if (!runner.have_executable("systemctl")) {
    return;
  }
  const bool ours_failed =
      runner.run({"sudo", "systemctl", "is-failed", "--quiet", kOurs}, nullptr) == 0;
  const bool ours_active =
      runner.run({"sudo", "systemctl", "is-active", "--quiet", kOurs}, nullptr) == 0;
  const bool stock_active =
      runner.run({"sudo", "systemctl", "is-active", "--quiet", kStock}, nullptr) == 0;
  status.stock = stock_active ? Stock::Active : Stock::Inactive;
  if (ours_failed) {
    status.demo = Demo::Failed;
  } else if (ours_active) {
    status.demo = Demo::Active;
  } else {
    status.demo = Demo::Inactive;
  }
}

void fill_ros(ProcessRunner& runner, Status& status) {
  for (int attempt = 0; attempt < kRosProbeAttempts; ++attempt) {
    std::string out;
    runner.run(ros_probe_args(), &out);
    Status parsed;
    const RosProbeView view = parse_ros_probe(out, &parsed);

    if (view.have_chassis) {
      status.chassis = view.chassis_live ? Chassis::Active : Chassis::Inactive;
    }
    if (view.have_control) {
      status.mode = parsed.mode;
      status.remote_controller = parsed.remote_controller;
      status.reason = parsed.reason;
      status.have_control = true;
    }
    if (view.have_lidar) {
      status.lidar = view.lidar_live ? Lidar::Active : Lidar::Degraded;
      status.lidar_scan_ok = view.lidar_scan;
      status.lidar_tf_ok = view.lidar_tf;
    }
    if (view.have_camera) {
      status.camera = view.camera_live ? Camera::Active : Camera::Degraded;
      status.camera_color_ok = view.camera_color;
      status.camera_depth_ok = view.camera_depth;
      status.camera_tf_ok = view.camera_tf;
    }
    if (view.have_imu) {
      status.imu = view.imu_live ? Imu::Active : Imu::Degraded;
      status.imu_msg_ok = view.imu_msg;
      status.imu_odom_ok = view.imu_odom;
      status.imu_tf_ok = view.imu_tf;
    }
    if (view.have_odometry) {
      status.odometry = view.odometry_live ? Odometry::Active : Odometry::Degraded;
      status.odom_msg_ok = view.odom_msg;
      status.odom_tf_ok = view.odom_tf;
    }
    if (view.have_model) {
      status.model = view.model_live ? Model::Active : Model::Degraded;
      status.model_description_ok = view.model_description;
      status.model_tf_base_ok = view.model_tf_base;
      status.model_tf_lidar_ok = view.model_tf_lidar;
      status.model_tf_imu_ok = view.model_tf_imu;
      status.model_tf_depth_ok = view.model_tf_depth;
    }
    if (view.have_calibration) {
      status.have_calibration = true;
      status.calibration = view.calibration;
    }
  }
}

int systemctl(std::initializer_list<const char*> verb_and_unit) {
  std::vector<std::string> args;
  args.reserve(2 + verb_and_unit.size());
  args.emplace_back("sudo");
  args.emplace_back("systemctl");
  for (const char* part : verb_and_unit) {
    args.emplace_back(part);
  }
  return posix_run(args, nullptr);
}

bool is_active(const char* unit) { return systemctl({"is-active", "--quiet", unit}) == 0; }

bool stop_disable(const char* unit) {
  if (systemctl({"stop", unit}) != 0) {
    return false;
  }
  return systemctl({"disable", unit}) == 0;
}

bool enable_then(const char* unit, const char* action) {
  if (systemctl({"enable", unit}) != 0) {
    return false;
  }
  return systemctl({action, unit}) == 0;
}

bool docker_start(const char* container) {
  return posix_run({"docker", "start", container}, nullptr) == 0;
}

// docker stop exits non-zero if the container is already down; that is OK.
void docker_stop(const char* container) {
  posix_run({"docker", "stop", "-t", "1", container}, nullptr);
}

bool enter_demo(const char* action) {
  if (!stop_disable(kStock)) {
    return false;
  }
  docker_stop(kStockContainer);
  if (!enable_then(kOurs, action)) {
    return false;
  }
  return is_active(kOurs);
}

}  // namespace

bool parse_control_status_yaml(const std::string& text, Status* status) {
  if (status == nullptr) {
    return false;
  }
  bool have_state = false;
  bool have_remote = false;
  Mode mode = Mode::Follow;
  bool remote_on = false;
  std::string reason;

  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    trim_ws(line);
    if (!have_state && line.rfind("state:", 0) == 0) {
      std::string value = line.substr(6);
      trim_ws(value);
      if (parse_mode_token(value, &mode)) {
        have_state = true;
      }
    }
    if (!have_remote && line.rfind("remote_controller:", 0) == 0) {
      std::string value = line.substr(18);
      trim_ws(value);
      if (parse_bool_token(value, &remote_on)) {
        have_remote = true;
      }
    }
    if (line.rfind("reason:", 0) == 0) {
      reason = line.substr(7);
      trim_ws(reason);
    }
  }

  if (!have_state || !have_remote) {
    return false;
  }
  status->mode = mode;
  status->remote_controller = remote_on ? RemoteController::Active : RemoteController::Inactive;
  status->reason = (mode == Mode::Forbidden) ? reason : std::string();
  status->have_control = true;
  return true;
}

RosProbeView parse_ros_probe(const std::string& text, Status* control) {
  RosProbeView view;
  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    trim_ws(line);
    if (line == "T1CTL_CHASSIS=1") {
      view.have_chassis = true;
      view.chassis_live = true;
    } else if (line == "T1CTL_CHASSIS=0") {
      view.have_chassis = true;
      view.chassis_live = false;
    } else if (line == "T1CTL_LIDAR=1") {
      view.have_lidar = true;
      view.lidar_live = true;
    } else if (line == "T1CTL_LIDAR=0") {
      view.have_lidar = true;
      view.lidar_live = false;
    } else if (line == "T1CTL_LIDAR_SCAN=1") {
      view.lidar_scan = true;
    } else if (line == "T1CTL_LIDAR_TF=1") {
      view.lidar_tf = true;
    } else if (line == "T1CTL_CAMERA=1") {
      view.have_camera = true;
      view.camera_live = true;
    } else if (line == "T1CTL_CAMERA=0") {
      view.have_camera = true;
      view.camera_live = false;
    } else if (line == "T1CTL_CAMERA_COLOR=1") {
      view.camera_color = true;
    } else if (line == "T1CTL_CAMERA_DEPTH=1") {
      view.camera_depth = true;
    } else if (line == "T1CTL_CAMERA_TF=1") {
      view.camera_tf = true;
    } else if (line == "T1CTL_IMU=1") {
      view.have_imu = true;
      view.imu_live = true;
    } else if (line == "T1CTL_IMU=0") {
      view.have_imu = true;
      view.imu_live = false;
    } else if (line == "T1CTL_IMU_MSG=1") {
      view.imu_msg = true;
    } else if (line == "T1CTL_IMU_ODOM=1") {
      view.imu_odom = true;
    } else if (line == "T1CTL_IMU_TF=1") {
      view.imu_tf = true;
    } else if (line == "T1CTL_ODOM=1") {
      view.have_odometry = true;
      view.odometry_live = true;
    } else if (line == "T1CTL_ODOM=0") {
      view.have_odometry = true;
      view.odometry_live = false;
    } else if (line == "T1CTL_ODOM_MSG=1") {
      view.odom_msg = true;
    } else if (line == "T1CTL_ODOM_TF=1") {
      view.odom_tf = true;
    } else if (line == "T1CTL_MODEL=1") {
      view.have_model = true;
      view.model_live = true;
    } else if (line == "T1CTL_MODEL=0") {
      view.have_model = true;
      view.model_live = false;
    } else if (line == "T1CTL_MODEL_DESCRIPTION=1") {
      view.model_description = true;
    } else if (line == "T1CTL_MODEL_TF_BASE=1") {
      view.model_tf_base = true;
    } else if (line == "T1CTL_MODEL_TF_LIDAR=1") {
      view.model_tf_lidar = true;
    } else if (line == "T1CTL_MODEL_TF_IMU=1") {
      view.model_tf_imu = true;
    } else if (line == "T1CTL_MODEL_TF_DEPTH=1") {
      view.model_tf_depth = true;
    } else if (line == "T1CTL_CALIB=factory") {
      view.have_calibration = true;
      view.calibration = Calibration::Factory;
    } else if (line == "T1CTL_CALIB=file") {
      view.have_calibration = true;
      view.calibration = Calibration::File;
    } else if (line == "T1CTL_CALIB=unused") {
      view.have_calibration = true;
      view.calibration = Calibration::Unused;
    }
  }
  if (control != nullptr && parse_control_status_yaml(text, control)) {
    view.have_control = true;
  }
  return view;
}

bool parse_mode_result(const std::string& text, ModeChange* change) {
  if (change == nullptr) {
    return false;
  }
  ModeChange parsed;
  bool have_ok = false;
  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    trim_ws(line);
    if (line == "T1CTL_MODE_OK=1") {
      parsed.ok = true;
      have_ok = true;
    } else if (line == "T1CTL_MODE_OK=0") {
      parsed.ok = false;
      have_ok = true;
    } else if (line.rfind("from:", 0) == 0) {
      std::string value = line.substr(5);
      trim_ws(value);
      if (parse_mode_token(value, &parsed.from)) {
        parsed.have_from = true;
      }
    } else if (line.rfind("to:", 0) == 0) {
      std::string value = line.substr(3);
      trim_ws(value);
      if (parse_mode_token(value, &parsed.to)) {
        parsed.have_to = true;
      }
    } else if (line.rfind("reason:", 0) == 0) {
      parsed.reason = line.substr(7);
      trim_ws(parsed.reason);
    } else if (line.rfind("detail:", 0) == 0) {
      std::string value = line.substr(7);
      trim_ws(value);
      append_detail_line(&parsed.detail, value);
    }
  }
  if (!have_ok) {
    return false;
  }
  if (parsed.ok && parsed.to == Mode::Forbidden) {
    if (parsed.reason.empty()) {
      parsed.reason = "operator";
    }
  } else if (parsed.to != Mode::Forbidden) {
    parsed.reason.clear();
  }
  *change = parsed;
  return true;
}

bool have_systemctl() { return posix_have_executable("systemctl"); }

ProcessRunner& process_runner() { return posix_runner(); }

Status query() { return query(posix_runner()); }

void fill_dds_buffers(Status& status, const std::string& path) {
  status.dds_buffers = DdsBuffers::Unknown;
  status.rmem_max = 0;
  std::ifstream in(path);
  if (!in) {
    return;
  }
  std::string line;
  if (!std::getline(in, line)) {
    return;
  }
  trim_ws(line);
  int value = 0;
  if (!parse_int_exact(line, &value)) {
    return;
  }
  status.rmem_max = value;
  status.dds_buffers = value >= kDdsRmemMinBytes ? DdsBuffers::Active : DdsBuffers::Degraded;
}

Status query_host(ProcessRunner& runner) {
  Status status;
  fill_systemd(runner, status);
  fill_dds_buffers(status);
  return status;
}

Status query(ProcessRunner& runner) {
  Status status = query_host(runner);
  // Probe the live container, not the systemd Active bit: is-active can lag or
  // be missing on a host without systemctl while mentorpi-t1 is still running.
  if (container_running(runner, kOursContainer)) {
    fill_ros(runner, status);
  }
  return status;
}

bool set_mode(ModeCommand command, ModeChange* change, Status* known) {
  return set_mode(posix_runner(), command, change, known);
}

bool set_mode(ProcessRunner& runner, ModeCommand command, ModeChange* change, Status* known) {
  if (change == nullptr) {
    return false;
  }
  *change = ModeChange{};
  Status local;
  Status* status = known != nullptr ? known : &local;
  *status = Status{};
  fill_systemd(runner, *status);

  if (!container_running(runner, kOursContainer)) {
    fail_mode_change(change,
                     "container mentorpi-t1 is not running\n"
                     "/control/state has no publisher");
    return false;
  }

  std::string out;
  runner.run(ros_mode_args(mode_command_target(command)), &out);
  ModeChange parsed;
  if (!parse_mode_result(out, &parsed) || !parsed.ok || !parsed.have_to) {
    if (parsed.detail.empty()) {
      fail_mode_change(change, "mode helper failed");
    } else {
      fail_mode_change(change, parsed.detail);
    }
    return false;
  }
  *change = parsed;
  return true;
}

bool start() {
  if (!have_systemctl()) {
    return false;
  }
  return enter_demo("start");
}

bool restart() {
  if (!have_systemctl()) {
    return false;
  }
  return enter_demo("restart");
}

bool stock() {
  if (!have_systemctl()) {
    return false;
  }
  if (!stop_disable(kOurs)) {
    return false;
  }
  docker_stop(kOursContainer);
  if (!docker_start(kStockContainer)) {
    return false;
  }
  if (!enable_then(kStock, "start")) {
    return false;
  }
  return is_active(kStock);
}

int ProcessRunner::run_lines(const std::vector<std::string>& args,
                             const std::function<void(const std::string& line)>& on_line,
                             std::string* stdout_out) {
  std::string captured;
  std::string* out = stdout_out != nullptr ? stdout_out : &captured;
  const int code = run(args, out);
  std::istringstream ss(*out);
  std::string line;
  while (std::getline(ss, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (on_line) {
      on_line(line);
    }
  }
  return code;
}

}  // namespace units
