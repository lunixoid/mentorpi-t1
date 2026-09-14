#include "viewer.hpp"

#include <sstream>
#include <string>
#include <vector>

namespace viewer {
namespace {

bool container_running(units::ProcessRunner& runner, const char* name) {
  std::string out;
  if (runner.run({"docker", "inspect", "-f", "{{.State.Running}}", name}, &out) != 0) {
    return false;
  }
  while (!out.empty()) {
    const char c = out.back();
    if (c != '\n' && c != '\r' && c != ' ' && c != '\t') {
      break;
    }
    out.pop_back();
  }
  return out == "true";
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
  while (!text.empty()) {
    const char c = text.back();
    if (c != '\n' && c != '\r' && c != ' ' && c != '\t') {
      break;
    }
    text.pop_back();
  }
}

bool parse_kv_line(const std::string& line, const char* key, std::string* value) {
  const std::string prefix = std::string(key) + "=";
  if (line.rfind(prefix, 0) != 0) {
    return false;
  }
  *value = line.substr(prefix.size());
  trim_ws(*value);
  return true;
}

bool parse_flag_line(const std::string& line, const char* key, bool* value) {
  std::string token;
  if (!parse_kv_line(line, key, &token)) {
    return false;
  }
  *value = token == "1";
  return true;
}

std::vector<std::string> docker_bash_args(const char* script) {
  return {"docker",    "exec", "-u",  "ubuntu", "-w", "/home/ubuntu", units::kOursContainer,
          "/bin/bash", "-c",   script};
}

constexpr const char* kViewerProbeScript =
    "export PYTHONUNBUFFERED=1; "
    "source /home/ubuntu/ros2_ws/.hiwonderrc >/dev/null 2>/dev/null; "
    "export ROS_LOCALHOST_ONLY=0; "
    "source /home/ubuntu/mentorpi_t1_ws/install/setup.bash >/dev/null 2>/dev/null; "
    "bridge=0; "
    "if pgrep -x foxglove_bridge >/dev/null 2>&1; then bridge=1; fi; "
    "port=0; "
    "if ss -ltn 2>/dev/null | grep -q ':8765'; then port=1; fi; "
    "printf 'T1CTL_BRIDGE=%s\\n' \"$bridge\"; "
    "printf 'T1CTL_PORT=%s\\n' \"$port\"; "
    "printf 'T1CTL_ROS_DOMAIN_ID=%s\\n' \"${ROS_DOMAIN_ID:-}\"; ";

constexpr const char* kViewerStartScript =
    "export PYTHONUNBUFFERED=1; "
    "source /home/ubuntu/ros2_ws/.hiwonderrc >/dev/null 2>/dev/null; "
    "export ROS_LOCALHOST_ONLY=0; "
    "source /home/ubuntu/mentorpi_t1_ws/install/setup.bash >/dev/null 2>/dev/null; "
    "if pgrep -x foxglove_bridge >/dev/null 2>&1; then "
    "printf 'T1CTL_STARTED=already\\n'; exit 0; "
    "fi; "
    "launch_log=/tmp/t1ctl-foxglove-launch.log; "
    ": >\"$launch_log\"; "
    "nohup ros2 launch mentorpi_bringup foxglove_bridge.launch.py >>\"$launch_log\" 2>&1 & "
    "for i in $(seq 1 20); do "
    "if pgrep -x foxglove_bridge >/dev/null 2>&1 && "
    "ss -ltn 2>/dev/null | grep -q ':8765'; then "
    "rm -f \"$launch_log\"; "
    "printf 'T1CTL_STARTED=ok\\n'; exit 0; "
    "fi; "
    "sleep 0.25; "
    "done; "
    "if [ -s \"$launch_log\" ]; then "
    "tail -n 15 \"$launch_log\" | sed 's/^/T1CTL_DIAG=/'; "
    "fi; "
    "printf 'T1CTL_STARTED=timeout\\n'; exit 1";

constexpr const char* kViewerStopScript =
    "if ! pgrep -x foxglove_bridge >/dev/null 2>&1; then "
    "printf 'T1CTL_STOPPED=already\\n'; exit 0; "
    "fi; "
    "pkill -x foxglove_bridge >/dev/null 2>&1 || true; "
    "for i in $(seq 1 20); do "
    "if ! pgrep -x foxglove_bridge >/dev/null 2>&1; then "
    "printf 'T1CTL_STOPPED=ok\\n'; exit 0; "
    "fi; "
    "sleep 0.25; "
    "done; "
    "printf 'T1CTL_STOPPED=timeout\\n'; exit 1";

void fill_from_probe(units::ProcessRunner& runner, Status& status) {
  std::string out;
  runner.run(docker_bash_args(kViewerProbeScript), &out);
  parse_probe(out, &status);
}

bool bridge_ready(const Status& status) {
  return status.bridge == Bridge::Active && status.port_listening;
}

void append_detail_line(std::string* detail, const std::string& line) {
  if (detail == nullptr || line.empty()) {
    return;
  }
  if (!detail->empty()) {
    detail->push_back('\n');
  }
  detail->append(line);
}

void parse_start_output(const std::string& text, std::string* started, std::string* detail) {
  if (started != nullptr) {
    started->clear();
  }
  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    trim_ws(line);
    if (line.empty()) {
      continue;
    }
    std::string token;
    if (started != nullptr && parse_kv_line(line, "T1CTL_STARTED", &token)) {
      *started = token;
      continue;
    }
    if (parse_kv_line(line, "T1CTL_DIAG", &token)) {
      append_detail_line(detail, token);
    }
  }
}

std::string default_start_failure_detail(const std::string& started) {
  if (started == "timeout") {
    return "bridge did not become ready within 5s";
  }
  return {};
}

}  // namespace

bool parse_probe(const std::string& text, Status* status) {
  if (status == nullptr) {
    return false;
  }
  bool have_bridge = false;
  bool have_port = false;
  bool have_domain = false;
  bool bridge_on = false;
  bool port_on = false;
  std::string domain;

  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    trim_ws(line);
    if (!have_bridge && parse_flag_line(line, "T1CTL_BRIDGE", &bridge_on)) {
      have_bridge = true;
    }
    if (!have_port && parse_flag_line(line, "T1CTL_PORT", &port_on)) {
      have_port = true;
    }
    if (!have_domain && parse_kv_line(line, "T1CTL_ROS_DOMAIN_ID", &domain)) {
      have_domain = true;
    }
  }

  if (!have_bridge || !have_port) {
    return false;
  }
  status->bridge = bridge_on ? Bridge::Active : Bridge::Inactive;
  status->port_listening = port_on;
  if (have_domain) {
    status->ros_domain_id = domain;
  }
  return true;
}

std::string detect_host(units::ProcessRunner& runner) {
  std::string out;
  if (runner.run({"hostname", "-I"}, &out) != 0) {
    return kDefaultHost;
  }
  trim_ws(out);
  const std::size_t end = out.find_first_of(" \t");
  if (end != std::string::npos) {
    out.erase(end);
  }
  if (out.empty()) {
    return kDefaultHost;
  }
  return out;
}

Status query() { return viewer::query(units::process_runner()); }

Status query(units::ProcessRunner& runner) {
  Status status;
  status.host = detect_host(runner);
  status.port = kBridgePort;
  if (!container_running(runner, units::kOursContainer)) {
    return status;
  }
  status.demo = DemoContour::Active;
  fill_from_probe(runner, status);
  return status;
}

bool start() {
  Status status;
  return start(units::process_runner(), &status);
}

bool start(units::ProcessRunner& runner, Status* out, std::string* detail) {
  if (out == nullptr) {
    return false;
  }
  if (detail != nullptr) {
    detail->clear();
  }
  *out = viewer::query(runner);
  if (out->demo != DemoContour::Active) {
    if (detail != nullptr) {
      *detail = "demo contour is inactive; run: t1ctl start";
    }
    return false;
  }
  if (bridge_ready(*out)) {
    return true;
  }
  std::string script_out;
  const int code = runner.run(docker_bash_args(kViewerStartScript), &script_out);
  std::string started;
  parse_start_output(script_out, &started, detail);
  if (detail != nullptr && detail->empty()) {
    const std::string fallback = default_start_failure_detail(started);
    if (!fallback.empty()) {
      *detail = fallback;
    }
  }
  *out = viewer::query(runner);
  if (code == 0 && bridge_ready(*out)) {
    return true;
  }
  if (detail != nullptr && detail->empty()) {
    *detail = "bridge did not become ready";
  }
  return false;
}

bool stop() {
  Status status;
  return stop(units::process_runner(), &status);
}

bool stop(units::ProcessRunner& runner, Status* out) {
  if (out == nullptr) {
    return false;
  }
  *out = viewer::query(runner);
  if (out->demo != DemoContour::Active) {
    return out->bridge == Bridge::Inactive && !out->port_listening;
  }
  if (out->bridge == Bridge::Inactive && !out->port_listening) {
    return true;
  }
  const int code = runner.run(docker_bash_args(kViewerStopScript), nullptr);
  *out = viewer::query(runner);
  return code == 0 && out->bridge == Bridge::Inactive && !out->port_listening;
}

}  // namespace viewer
