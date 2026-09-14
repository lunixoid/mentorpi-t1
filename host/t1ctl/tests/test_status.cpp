#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "calib.hpp"
#include "ui.hpp"
#include "units.hpp"
#include "viewer.hpp"

namespace {

int g_fails = 0;

void check(bool cond, const char* expr, const char* file, int line) {
  if (!cond) {
    std::cerr << "FAIL " << file << ":" << line << " " << expr << '\n';
    ++g_fails;
  }
}

#define CHECK(cond) check(static_cast<bool>(cond), #cond, __FILE__, __LINE__)

class MockRunner final : public units::ProcessRunner {
 public:
  bool systemctl_on_path = true;
  bool demo_active = true;
  bool demo_failed = false;
  bool stock_active = false;
  bool container_up = true;
  std::vector<int> probe_codes;
  std::vector<std::string> probe_outs;
  std::vector<int> viewer_codes;
  std::vector<std::string> viewer_outs;
  std::vector<int> mode_codes;
  std::vector<std::string> mode_outs;
  std::vector<int> debug_codes;
  std::vector<std::string> debug_outs;
  std::vector<int> detect_codes;
  std::vector<std::string> detect_outs;
  std::vector<int> calib_codes;
  std::vector<std::string> calib_outs;
  std::vector<int> docker_cp_codes;
  std::string warm_out;
  int warm_code = 0;
  std::vector<std::vector<std::string>> calls;

  bool have_executable(const char* name) const override {
    return systemctl_on_path && std::string(name) == "systemctl";
  }

  int run(const std::vector<std::string>& args, std::string* out) override {
    calls.push_back(args);
    if (args.empty()) {
      return 127;
    }
    if (args[0] == "hostname" && args.size() >= 2 && args[1] == "-I") {
      if (out != nullptr) {
        *out = "192.168.2.2 192.168.2.3\n";
      }
      return 0;
    }
    if (args[0] == "docker" && args.size() >= 2 && args[1] == "inspect") {
      if (out != nullptr) {
        *out = container_up ? "true\n" : "false\n";
      }
      return 0;
    }
    if (args[0] == "docker" && args.size() >= 2 && args[1] == "cp") {
      const std::size_t i = docker_cp_used_;
      ++docker_cp_used_;
      if (out != nullptr) {
        *out = "";
      }
      if (i < docker_cp_codes.size()) {
        return docker_cp_codes[i];
      }
      return 0;
    }
    if (args[0] == "docker" && args.size() >= 2 && args[1] == "exec") {
      const char* script = args.back().c_str();
      const bool viewer_script = std::strstr(script, "T1CTL_BRIDGE=") != nullptr ||
                                 std::strstr(script, "T1CTL_STARTED=") != nullptr ||
                                 std::strstr(script, "T1CTL_STOPPED=") != nullptr;
      if (viewer_script) {
        const std::size_t i = viewer_used_;
        ++viewer_used_;
        if (i < viewer_outs.size()) {
          if (out != nullptr) {
            *out = viewer_outs[i];
          }
          if (i < viewer_codes.size()) {
            return viewer_codes[i];
          }
          return 0;
        }
      }
      const bool mode_script = std::strstr(script, "T1CTL_MODE_OK=") != nullptr;
      if (mode_script) {
        const std::size_t i = mode_used_;
        ++mode_used_;
        if (i < mode_outs.size()) {
          if (out != nullptr) {
            *out = mode_outs[i];
          }
          if (i < mode_codes.size()) {
            return mode_codes[i];
          }
          return 0;
        }
        if (out != nullptr) {
          *out = warm_out;
        }
        return warm_code;
      }
      const bool debug_script = std::strstr(script, "T1CTL_DEBUG_ACTION=") != nullptr;
      if (debug_script) {
        const std::size_t i = debug_used_;
        ++debug_used_;
        if (i < debug_outs.size()) {
          if (out != nullptr) {
            *out = debug_outs[i];
          }
          if (i < debug_codes.size()) {
            return debug_codes[i];
          }
          return 0;
        }
        if (out != nullptr) {
          *out = warm_out;
        }
        return warm_code;
      }
      const bool detect_script = std::strstr(script, "T1CTL_DETECT_ACTION=") != nullptr;
      if (detect_script) {
        const std::size_t i = detect_used_;
        ++detect_used_;
        if (i < detect_outs.size()) {
          if (out != nullptr) {
            *out = detect_outs[i];
          }
          if (i < detect_codes.size()) {
            return detect_codes[i];
          }
          return 0;
        }
        if (out != nullptr) {
          *out = warm_out;
        }
        return warm_code;
      }
      const bool calib_script = std::strstr(script, "ros2 run mentorpi_calibration") != nullptr;
      if (calib_script) {
        const std::size_t i = calib_used_;
        ++calib_used_;
        if (i < calib_outs.size()) {
          if (out != nullptr) {
            *out = calib_outs[i];
          }
          if (i < calib_codes.size()) {
            return calib_codes[i];
          }
          return 0;
        }
        if (out != nullptr) {
          *out = warm_out;
        }
        return warm_code;
      }
      const std::size_t i = probe_used_;
      ++probe_used_;
      if (i < probe_outs.size()) {
        if (out != nullptr) {
          *out = probe_outs[i];
        }
        if (i < probe_codes.size()) {
          return probe_codes[i];
        }
        return 0;
      }
      if (out != nullptr) {
        *out = warm_out;
      }
      return warm_code;
    }
    if (args.size() >= 5 && args[0] == "sudo" && args[1] == "systemctl") {
      const std::string verb = args[2];
      const std::string unit = args.back();
      if (verb == "is-failed") {
        return (unit == units::kOurs && demo_failed) ? 0 : 1;
      }
      if (verb == "is-active") {
        if (unit == units::kOurs) {
          return demo_active ? 0 : 1;
        }
        if (unit == units::kStock) {
          return stock_active ? 0 : 1;
        }
      }
      return 1;
    }
    return 127;
  }

  std::size_t probe_used() const { return probe_used_; }
  std::size_t viewer_used() const { return viewer_used_; }
  std::size_t mode_used() const { return mode_used_; }
  std::size_t debug_used() const { return debug_used_; }
  std::size_t detect_used() const { return detect_used_; }
  std::size_t calib_used() const { return calib_used_; }
  std::size_t docker_cp_used() const { return docker_cp_used_; }

 private:
  std::size_t probe_used_{0};
  std::size_t viewer_used_{0};
  std::size_t mode_used_{0};
  std::size_t debug_used_{0};
  std::size_t detect_used_{0};
  std::size_t calib_used_{0};
  std::size_t docker_cp_used_{0};
};

int count_exec(const MockRunner& runner) {
  int n = 0;
  for (const auto& args : runner.calls) {
    if (args.size() >= 2 && args[0] == "docker" && args[1] == "exec") {
      ++n;
    }
  }
  return n;
}

const char* last_probe_script(const MockRunner& runner) {
  for (auto it = runner.calls.rbegin(); it != runner.calls.rend(); ++it) {
    if (it->size() >= 2 && (*it)[0] == "docker" && (*it)[1] == "exec") {
      return it->back().c_str();
    }
  }
  return "";
}

const char* last_mode_script(const MockRunner& runner) {
  for (auto it = runner.calls.rbegin(); it != runner.calls.rend(); ++it) {
    if (it->size() >= 2 && (*it)[0] == "docker" && (*it)[1] == "exec" &&
        std::strstr(it->back().c_str(), "T1CTL_MODE_OK=") != nullptr) {
      return it->back().c_str();
    }
  }
  return "";
}

const char* last_debug_script(const MockRunner& runner) {
  for (auto it = runner.calls.rbegin(); it != runner.calls.rend(); ++it) {
    if (it->size() >= 2 && (*it)[0] == "docker" && (*it)[1] == "exec" &&
        std::strstr(it->back().c_str(), "T1CTL_DEBUG_ACTION=") != nullptr) {
      return it->back().c_str();
    }
  }
  return "";
}

const char* last_detect_script(const MockRunner& runner) {
  for (auto it = runner.calls.rbegin(); it != runner.calls.rend(); ++it) {
    if (it->size() >= 2 && (*it)[0] == "docker" && (*it)[1] == "exec" &&
        std::strstr(it->back().c_str(), "T1CTL_DETECT_ACTION=") != nullptr) {
      return it->back().c_str();
    }
  }
  return "";
}

constexpr const char* kLiveYaml =
    "T1CTL_CHASSIS=1\n"
    "T1CTL_LIDAR=1\n"
    "T1CTL_LIDAR_SCAN=1\n"
    "T1CTL_LIDAR_TF=1\n"
    "T1CTL_CAMERA=1\n"
    "T1CTL_CAMERA_COLOR=1\n"
    "T1CTL_CAMERA_DEPTH=1\n"
    "T1CTL_CAMERA_TF=1\n"
    "T1CTL_IMU=1\n"
    "T1CTL_IMU_MSG=1\n"
    "T1CTL_IMU_ODOM=1\n"
    "T1CTL_IMU_TF=1\n"
    "T1CTL_ODOM=1\n"
    "T1CTL_ODOM_MSG=1\n"
    "T1CTL_ODOM_TF=1\n"
    "T1CTL_MODEL=1\n"
    "T1CTL_MODEL_DESCRIPTION=1\n"
    "T1CTL_MODEL_TF_BASE=1\n"
    "T1CTL_MODEL_TF_LIDAR=1\n"
    "T1CTL_MODEL_TF_IMU=1\n"
    "T1CTL_MODEL_TF_DEPTH=1\n"
    "T1CTL_CALIB=factory\n"
    "state: 2\n"
    "remote_controller: true\n";

constexpr const char* kTimeoutOut = "";
constexpr const char* kChassisDead =
    "T1CTL_CHASSIS=0\n"
    "T1CTL_LIDAR=0\n"
    "T1CTL_LIDAR_SCAN=0\n"
    "T1CTL_LIDAR_TF=0\n"
    "T1CTL_CAMERA=0\n"
    "T1CTL_CAMERA_COLOR=0\n"
    "T1CTL_CAMERA_DEPTH=0\n"
    "T1CTL_CAMERA_TF=0\n"
    "T1CTL_IMU=0\n"
    "T1CTL_IMU_MSG=0\n"
    "T1CTL_IMU_ODOM=0\n"
    "T1CTL_IMU_TF=0\n"
    "T1CTL_ODOM=0\n"
    "T1CTL_ODOM_MSG=0\n"
    "T1CTL_ODOM_TF=0\n"
    "T1CTL_MODEL=0\n"
    "T1CTL_MODEL_DESCRIPTION=0\n"
    "T1CTL_MODEL_TF_BASE=0\n"
    "T1CTL_MODEL_TF_LIDAR=0\n"
    "T1CTL_MODEL_TF_IMU=0\n"
    "T1CTL_MODEL_TF_DEPTH=0\n";

void test_banner_before_yaml() {
  const std::string text =
      "\n"
      "****************************************\n"
      "Welcome to Hiwonder Ubuntu 22.04\n"
      "state: not-yaml\n"
      "ROS_DOMAIN_ID=7\n"
      "****************************************\n"
      "state: 2\n"
      "remote_controller: true\n"
      "---\n";
  units::Status status;
  status.remote_controller = units::RemoteController::Inactive;
  CHECK(units::parse_control_status_yaml(text, &status));
  CHECK(status.have_control);
  CHECK(status.mode == units::Mode::Follow);
  CHECK(status.reason.empty());
  CHECK(status.remote_controller == units::RemoteController::Active);

  units::Status from_probe;
  const units::RosProbeView view =
      units::parse_ros_probe(std::string("Welcome to Hiwonder\n") + kLiveYaml, &from_probe);
  CHECK(view.have_chassis);
  CHECK(view.chassis_live);
  CHECK(view.have_control);
  CHECK(from_probe.have_control);
  CHECK(from_probe.mode == units::Mode::Follow);
  CHECK(from_probe.remote_controller == units::RemoteController::Active);
}

void test_lidar_probe_parse() {
  units::Status status;
  const units::RosProbeView active =
      units::parse_ros_probe("T1CTL_LIDAR=1\nT1CTL_LIDAR_SCAN=1\nT1CTL_LIDAR_TF=1\n", &status);
  CHECK(active.have_lidar);
  CHECK(active.lidar_live);
  CHECK(active.lidar_scan);
  CHECK(active.lidar_tf);

  const units::RosProbeView degraded =
      units::parse_ros_probe("T1CTL_LIDAR=0\nT1CTL_LIDAR_SCAN=1\nT1CTL_LIDAR_TF=0\n", &status);
  CHECK(degraded.have_lidar);
  CHECK(!degraded.lidar_live);
  CHECK(degraded.lidar_scan);
  CHECK(!degraded.lidar_tf);
}

void test_camera_probe_parse() {
  units::Status status;
  const units::RosProbeView active = units::parse_ros_probe(
      "T1CTL_CAMERA=1\n"
      "T1CTL_CAMERA_COLOR=1\n"
      "T1CTL_CAMERA_DEPTH=1\n"
      "T1CTL_CAMERA_TF=1\n",
      &status);
  CHECK(active.have_camera);
  CHECK(active.camera_live);
  CHECK(active.camera_color);
  CHECK(active.camera_depth);
  CHECK(active.camera_tf);

  const units::RosProbeView degraded = units::parse_ros_probe(
      "T1CTL_CAMERA=0\n"
      "T1CTL_CAMERA_COLOR=1\n"
      "T1CTL_CAMERA_DEPTH=0\n"
      "T1CTL_CAMERA_TF=1\n",
      &status);
  CHECK(degraded.have_camera);
  CHECK(!degraded.camera_live);
  CHECK(degraded.camera_color);
  CHECK(!degraded.camera_depth);
  CHECK(degraded.camera_tf);

  const units::RosProbeView missing_color = units::parse_ros_probe(
      "T1CTL_CAMERA=0\n"
      "T1CTL_CAMERA_DEPTH=1\n"
      "T1CTL_CAMERA_TF=1\n",
      &status);
  CHECK(missing_color.have_camera);
  CHECK(!missing_color.camera_live);
  CHECK(!missing_color.camera_color);
  CHECK(missing_color.camera_depth);
  CHECK(missing_color.camera_tf);

  const units::RosProbeView missing_depth = units::parse_ros_probe(
      "T1CTL_CAMERA=0\n"
      "T1CTL_CAMERA_COLOR=1\n"
      "T1CTL_CAMERA_TF=1\n",
      &status);
  CHECK(missing_depth.have_camera);
  CHECK(!missing_depth.camera_live);
  CHECK(missing_depth.camera_color);
  CHECK(!missing_depth.camera_depth);
  CHECK(missing_depth.camera_tf);

  const units::RosProbeView missing_tf = units::parse_ros_probe(
      "T1CTL_CAMERA=0\n"
      "T1CTL_CAMERA_COLOR=1\n"
      "T1CTL_CAMERA_DEPTH=1\n",
      &status);
  CHECK(missing_tf.have_camera);
  CHECK(!missing_tf.camera_live);
  CHECK(missing_tf.camera_color);
  CHECK(missing_tf.camera_depth);
  CHECK(!missing_tf.camera_tf);
}

void test_imu_probe_parse() {
  units::Status status;
  const units::RosProbeView active = units::parse_ros_probe(
      "T1CTL_IMU=1\n"
      "T1CTL_IMU_MSG=1\n"
      "T1CTL_IMU_ODOM=1\n"
      "T1CTL_IMU_TF=1\n",
      &status);
  CHECK(active.have_imu);
  CHECK(active.imu_live);
  CHECK(active.imu_msg);
  CHECK(active.imu_odom);
  CHECK(active.imu_tf);

  const units::RosProbeView degraded = units::parse_ros_probe(
      "T1CTL_IMU=0\n"
      "T1CTL_IMU_MSG=1\n"
      "T1CTL_IMU_ODOM=0\n"
      "T1CTL_IMU_TF=1\n",
      &status);
  CHECK(degraded.have_imu);
  CHECK(!degraded.imu_live);
  CHECK(degraded.imu_msg);
  CHECK(!degraded.imu_odom);
  CHECK(degraded.imu_tf);

  const units::RosProbeView missing_msg = units::parse_ros_probe(
      "T1CTL_IMU=0\n"
      "T1CTL_IMU_ODOM=1\n"
      "T1CTL_IMU_TF=1\n",
      &status);
  CHECK(missing_msg.have_imu);
  CHECK(!missing_msg.imu_live);
  CHECK(!missing_msg.imu_msg);
  CHECK(missing_msg.imu_odom);
  CHECK(missing_msg.imu_tf);

  const units::RosProbeView missing_odom = units::parse_ros_probe(
      "T1CTL_IMU=0\n"
      "T1CTL_IMU_MSG=1\n"
      "T1CTL_IMU_TF=1\n",
      &status);
  CHECK(missing_odom.have_imu);
  CHECK(!missing_odom.imu_live);
  CHECK(missing_odom.imu_msg);
  CHECK(!missing_odom.imu_odom);
  CHECK(missing_odom.imu_tf);

  const units::RosProbeView missing_tf = units::parse_ros_probe(
      "T1CTL_IMU=0\n"
      "T1CTL_IMU_MSG=1\n"
      "T1CTL_IMU_ODOM=1\n",
      &status);
  CHECK(missing_tf.have_imu);
  CHECK(!missing_tf.imu_live);
  CHECK(missing_tf.imu_msg);
  CHECK(missing_tf.imu_odom);
  CHECK(!missing_tf.imu_tf);
}

void test_odometry_probe_parse() {
  units::Status status;
  const units::RosProbeView active = units::parse_ros_probe(
      "T1CTL_ODOM=1\n"
      "T1CTL_ODOM_MSG=1\n"
      "T1CTL_ODOM_TF=1\n",
      &status);
  CHECK(active.have_odometry);
  CHECK(active.odometry_live);
  CHECK(active.odom_msg);
  CHECK(active.odom_tf);

  const units::RosProbeView degraded = units::parse_ros_probe(
      "T1CTL_ODOM=0\n"
      "T1CTL_ODOM_MSG=1\n"
      "T1CTL_ODOM_TF=0\n",
      &status);
  CHECK(degraded.have_odometry);
  CHECK(!degraded.odometry_live);
  CHECK(degraded.odom_msg);
  CHECK(!degraded.odom_tf);

  const units::RosProbeView missing_msg = units::parse_ros_probe(
      "T1CTL_ODOM=0\n"
      "T1CTL_ODOM_TF=1\n",
      &status);
  CHECK(missing_msg.have_odometry);
  CHECK(!missing_msg.odometry_live);
  CHECK(!missing_msg.odom_msg);
  CHECK(missing_msg.odom_tf);

  const units::RosProbeView missing_tf = units::parse_ros_probe(
      "T1CTL_ODOM=0\n"
      "T1CTL_ODOM_MSG=1\n",
      &status);
  CHECK(missing_tf.have_odometry);
  CHECK(!missing_tf.odometry_live);
  CHECK(missing_tf.odom_msg);
  CHECK(!missing_tf.odom_tf);
}

void test_model_probe_parse() {
  units::Status status;
  const units::RosProbeView active = units::parse_ros_probe(
      "T1CTL_MODEL=1\n"
      "T1CTL_MODEL_DESCRIPTION=1\n"
      "T1CTL_MODEL_TF_BASE=1\n"
      "T1CTL_MODEL_TF_LIDAR=1\n"
      "T1CTL_MODEL_TF_IMU=1\n"
      "T1CTL_MODEL_TF_DEPTH=1\n",
      &status);
  CHECK(active.have_model);
  CHECK(active.model_live);
  CHECK(active.model_description);
  CHECK(active.model_tf_base);
  CHECK(active.model_tf_lidar);
  CHECK(active.model_tf_imu);
  CHECK(active.model_tf_depth);

  const units::RosProbeView degraded = units::parse_ros_probe(
      "T1CTL_MODEL=0\n"
      "T1CTL_MODEL_DESCRIPTION=1\n"
      "T1CTL_MODEL_TF_BASE=1\n"
      "T1CTL_MODEL_TF_LIDAR=0\n"
      "T1CTL_MODEL_TF_IMU=1\n"
      "T1CTL_MODEL_TF_DEPTH=0\n",
      &status);
  CHECK(degraded.have_model);
  CHECK(!degraded.model_live);
  CHECK(degraded.model_description);
  CHECK(degraded.model_tf_base);
  CHECK(!degraded.model_tf_lidar);
  CHECK(degraded.model_tf_imu);
  CHECK(!degraded.model_tf_depth);

  const units::RosProbeView missing_description = units::parse_ros_probe(
      "T1CTL_MODEL=0\n"
      "T1CTL_MODEL_TF_BASE=1\n"
      "T1CTL_MODEL_TF_LIDAR=1\n"
      "T1CTL_MODEL_TF_IMU=1\n"
      "T1CTL_MODEL_TF_DEPTH=1\n",
      &status);
  CHECK(missing_description.have_model);
  CHECK(!missing_description.model_live);
  CHECK(!missing_description.model_description);
  CHECK(missing_description.model_tf_base);
  CHECK(missing_description.model_tf_lidar);
  CHECK(missing_description.model_tf_imu);
  CHECK(missing_description.model_tf_depth);

  const units::RosProbeView missing_base = units::parse_ros_probe(
      "T1CTL_MODEL=0\n"
      "T1CTL_MODEL_DESCRIPTION=1\n"
      "T1CTL_MODEL_TF_LIDAR=1\n"
      "T1CTL_MODEL_TF_IMU=1\n"
      "T1CTL_MODEL_TF_DEPTH=1\n",
      &status);
  CHECK(missing_base.have_model);
  CHECK(!missing_base.model_live);
  CHECK(missing_base.model_description);
  CHECK(!missing_base.model_tf_base);

  const units::RosProbeView missing_imu = units::parse_ros_probe(
      "T1CTL_MODEL=0\n"
      "T1CTL_MODEL_DESCRIPTION=1\n"
      "T1CTL_MODEL_TF_BASE=1\n"
      "T1CTL_MODEL_TF_LIDAR=1\n"
      "T1CTL_MODEL_TF_DEPTH=1\n",
      &status);
  CHECK(missing_imu.have_model);
  CHECK(!missing_imu.model_live);
  CHECK(missing_imu.model_description);
  CHECK(missing_imu.model_tf_base);
  CHECK(missing_imu.model_tf_lidar);
  CHECK(!missing_imu.model_tf_imu);
  CHECK(missing_imu.model_tf_depth);

  const units::RosProbeView missing_lidar = units::parse_ros_probe(
      "T1CTL_MODEL=0\n"
      "T1CTL_MODEL_DESCRIPTION=1\n"
      "T1CTL_MODEL_TF_BASE=1\n"
      "T1CTL_MODEL_TF_IMU=1\n"
      "T1CTL_MODEL_TF_DEPTH=1\n",
      &status);
  CHECK(missing_lidar.have_model);
  CHECK(!missing_lidar.model_live);
  CHECK(missing_lidar.model_description);
  CHECK(missing_lidar.model_tf_base);
  CHECK(!missing_lidar.model_tf_lidar);
  CHECK(missing_lidar.model_tf_imu);
  CHECK(missing_lidar.model_tf_depth);

  const units::RosProbeView missing_depth = units::parse_ros_probe(
      "T1CTL_MODEL=0\n"
      "T1CTL_MODEL_DESCRIPTION=1\n"
      "T1CTL_MODEL_TF_BASE=1\n"
      "T1CTL_MODEL_TF_LIDAR=1\n"
      "T1CTL_MODEL_TF_IMU=1\n",
      &status);
  CHECK(missing_depth.have_model);
  CHECK(!missing_depth.model_live);
  CHECK(missing_depth.model_description);
  CHECK(missing_depth.model_tf_base);
  CHECK(missing_depth.model_tf_lidar);
  CHECK(missing_depth.model_tf_imu);
  CHECK(!missing_depth.model_tf_depth);
}

void test_true_false_parse() {
  units::Status status;
  CHECK(units::parse_control_status_yaml("state: 2\nremote_controller: true\n", &status));
  CHECK(status.have_control);
  CHECK(status.mode == units::Mode::Follow);
  CHECK(status.reason.empty());
  CHECK(status.remote_controller == units::RemoteController::Active);
  CHECK(units::parse_control_status_yaml("state: 1\nremote_controller: True\n", &status));
  CHECK(status.mode == units::Mode::Manual);
  CHECK(status.reason.empty());
  CHECK(status.remote_controller == units::RemoteController::Active);
  CHECK(units::parse_control_status_yaml("state: 2\nremote_controller: 1\n", &status));
  CHECK(status.remote_controller == units::RemoteController::Active);

  CHECK(units::parse_control_status_yaml("state: 1\nremote_controller: false\n", &status));
  CHECK(status.mode == units::Mode::Manual);
  CHECK(status.remote_controller == units::RemoteController::Inactive);
  CHECK(units::parse_control_status_yaml("state: 2\nremote_controller: False\n", &status));
  CHECK(status.remote_controller == units::RemoteController::Inactive);
  CHECK(units::parse_control_status_yaml("state: 2\nremote_controller: 0\n", &status));
  CHECK(status.remote_controller == units::RemoteController::Inactive);

  units::Status untouched;
  untouched.remote_controller = units::RemoteController::Active;
  untouched.mode = units::Mode::Manual;
  untouched.have_control = true;
  untouched.reason = "operator";
  CHECK(!units::parse_control_status_yaml("banner only\n", &untouched));
  CHECK(untouched.remote_controller == units::RemoteController::Active);
  CHECK(untouched.mode == units::Mode::Manual);
  CHECK(untouched.have_control);
  CHECK(untouched.reason == "operator");
  CHECK(!units::parse_control_status_yaml("state: 2\nremote_controller: maybe\n", &untouched));
  CHECK(untouched.remote_controller == units::RemoteController::Active);
}

void test_forbidden_and_reason_parse() {
  units::Status status;
  CHECK(units::parse_control_status_yaml("state: 0\nremote_controller: true\nreason: operator\n",
                                         &status));
  CHECK(status.have_control);
  CHECK(status.mode == units::Mode::Forbidden);
  CHECK(status.reason == "operator");
  CHECK(status.remote_controller == units::RemoteController::Active);

  CHECK(units::parse_control_status_yaml(
      "state: 0\nremote_controller: false\nreason: no /pnc/desired_twist\n", &status));
  CHECK(status.mode == units::Mode::Forbidden);
  CHECK(status.reason == "no /pnc/desired_twist");

  CHECK(units::parse_control_status_yaml(
      "state: FORBIDDEN\nremote_controller: true\nreason: operator\n", &status));
  CHECK(status.mode == units::Mode::Forbidden);
  CHECK(status.reason == "operator");

  CHECK(units::parse_control_status_yaml("state: 2\nremote_controller: true\nreason: operator\n",
                                         &status));
  CHECK(status.have_control);
  CHECK(status.mode == units::Mode::Follow);
  CHECK(status.reason.empty());

  units::Status unknown;
  unknown.mode = units::Mode::Manual;
  unknown.have_control = true;
  CHECK(!units::parse_control_status_yaml("state: 9\nremote_controller: true\n", &unknown));
  CHECK(unknown.mode == units::Mode::Manual);
  CHECK(unknown.have_control);

  units::Status from_probe;
  const units::RosProbeView view = units::parse_ros_probe(
      "T1CTL_CHASSIS=1\nstate: 0\nremote_controller: true\nreason: operator\n", &from_probe);
  CHECK(view.have_control);
  CHECK(from_probe.have_control);
  CHECK(from_probe.mode == units::Mode::Forbidden);
  CHECK(from_probe.reason == "operator");
}

void test_timeout_then_retry() {
  MockRunner runner;
  runner.probe_outs = {kTimeoutOut};
  runner.probe_codes = {124};
  const units::Status status = units::query(runner);
  CHECK(status.demo == units::Demo::Active);
  CHECK(status.chassis == units::Chassis::Inactive);
  CHECK(!status.have_control);
  CHECK(status.remote_controller == units::RemoteController::Inactive);
  CHECK(status.lidar == units::Lidar::Inactive);
  CHECK(status.camera == units::Camera::Inactive);
  CHECK(status.imu == units::Imu::Inactive);
  CHECK(status.odometry == units::Odometry::Inactive);
  CHECK(status.model == units::Model::Inactive);
  CHECK(count_exec(runner) == 1);
  CHECK(runner.probe_used() == 1);

  const std::string script = last_probe_script(runner);
  CHECK(script.find("python3") != std::string::npos);
  CHECK(script.find("rclpy") != std::string::npos);
  CHECK(script.find("/vehicle/status") != std::string::npos);
  CHECK(script.find("/scan") != std::string::npos);
  CHECK(script.find("lidar_frame") != std::string::npos);
  CHECK(script.find("/aurora/rgb/image_raw") != std::string::npos);
  CHECK(script.find("/aurora/rgb/image_raw/compressed") != std::string::npos);
  CHECK(script.find("/aurora/depth/image_raw") != std::string::npos);
  CHECK(script.find("/aurora/points2") != std::string::npos);
  CHECK(script.find("depth_camera_link") != std::string::npos);
  CHECK(script.find("/imu") != std::string::npos);
  CHECK(script.find("/imu_odom") != std::string::npos);
  CHECK(script.find("/odom_raw") != std::string::npos);
  CHECK(script.find("T1CTL_IMU=") != std::string::npos);
  CHECK(script.find("T1CTL_IMU_MSG=") != std::string::npos);
  CHECK(script.find("T1CTL_IMU_ODOM=") != std::string::npos);
  CHECK(script.find("T1CTL_IMU_TF=") != std::string::npos);
  CHECK(script.find("T1CTL_ODOM=") != std::string::npos);
  CHECK(script.find("T1CTL_ODOM_MSG=") != std::string::npos);
  CHECK(script.find("T1CTL_ODOM_TF=") != std::string::npos);
  CHECK(script.find("T1CTL_CAMERA=") != std::string::npos);
  CHECK(script.find("T1CTL_CAMERA_COLOR=") != std::string::npos);
  CHECK(script.find("T1CTL_CAMERA_DEPTH=") != std::string::npos);
  CHECK(script.find("T1CTL_CAMERA_TF=") != std::string::npos);
  CHECK(script.find("/robot_description") != std::string::npos);
  CHECK(script.find("base_link") != std::string::npos);
  CHECK(script.find("imu_link") != std::string::npos);
  CHECK(script.find("depth_cam_frame") != std::string::npos);
  CHECK(script.find("T1CTL_MODEL=") != std::string::npos);
  CHECK(script.find("T1CTL_MODEL_DESCRIPTION=") != std::string::npos);
  CHECK(script.find("T1CTL_MODEL_TF_BASE=") != std::string::npos);
  CHECK(script.find("T1CTL_MODEL_TF_LIDAR=") != std::string::npos);
  CHECK(script.find("T1CTL_MODEL_TF_IMU=") != std::string::npos);
  CHECK(script.find("T1CTL_MODEL_TF_DEPTH=") != std::string::npos);
  CHECK(script.find("/control/status") != std::string::npos);
  CHECK(script.find("msg.reason") != std::string::npos);
  CHECK(script.find(".hiwonderrc >/dev/null") != std::string::npos);
  CHECK(script.find("ROS_LOCALHOST_ONLY=0") != std::string::npos);
  CHECK(script.find("PYTHONUNBUFFERED=1") != std::string::npos);
  CHECK(script.find("ros2 daemon start") == std::string::npos);
  CHECK(script.find("timeout 2 ros2 topic echo") == std::string::npos);
  CHECK(script.find("tf2_echo") == std::string::npos);
  CHECK(script.find("timeout 2 ros2 topic list") == std::string::npos);
}

void test_parsed_false_is_not_a_timeout() {
  MockRunner runner;
  runner.probe_outs = {
      "T1CTL_CHASSIS=1\n"
      "T1CTL_LIDAR=0\n"
      "T1CTL_LIDAR_SCAN=0\n"
      "T1CTL_LIDAR_TF=1\n"
      "state: 1\n"
      "remote_controller: false\n"};
  runner.warm_out = runner.probe_outs[0];
  const units::Status status = units::query(runner);
  CHECK(status.chassis == units::Chassis::Active);
  CHECK(status.have_control);
  CHECK(status.mode == units::Mode::Manual);
  CHECK(status.reason.empty());
  CHECK(status.remote_controller == units::RemoteController::Inactive);
  CHECK(status.lidar == units::Lidar::Degraded);
  CHECK(!status.lidar_scan_ok);
  CHECK(status.lidar_tf_ok);
  CHECK(status.camera == units::Camera::Inactive);
  CHECK(status.model == units::Model::Inactive);
  CHECK(count_exec(runner) == 1);
}

void test_model_active_query() {
  MockRunner runner;
  runner.probe_outs = {kLiveYaml};
  runner.warm_out = kLiveYaml;
  const units::Status status = units::query(runner);
  CHECK(status.model == units::Model::Active);
  CHECK(status.model_description_ok);
  CHECK(status.model_tf_base_ok);
  CHECK(status.model_tf_lidar_ok);
  CHECK(status.model_tf_imu_ok);
  CHECK(status.model_tf_depth_ok);
  CHECK(count_exec(runner) == 1);
}

void test_imu_active_query() {
  MockRunner runner;
  runner.probe_outs = {kLiveYaml};
  runner.warm_out = kLiveYaml;
  const units::Status status = units::query(runner);
  CHECK(status.imu == units::Imu::Active);
  CHECK(status.imu_msg_ok);
  CHECK(status.imu_odom_ok);
  CHECK(status.imu_tf_ok);
  CHECK(count_exec(runner) == 1);
}

void test_odometry_active_query() {
  MockRunner runner;
  runner.probe_outs = {kLiveYaml};
  runner.warm_out = kLiveYaml;
  const units::Status status = units::query(runner);
  CHECK(status.odometry == units::Odometry::Active);
  CHECK(status.odom_msg_ok);
  CHECK(status.odom_tf_ok);
  CHECK(count_exec(runner) == 1);
}

void test_imu_degraded_query() {
  MockRunner runner;
  runner.probe_outs = {
      "T1CTL_CHASSIS=1\n"
      "T1CTL_LIDAR=1\n"
      "T1CTL_LIDAR_SCAN=1\n"
      "T1CTL_LIDAR_TF=1\n"
      "T1CTL_IMU=0\n"
      "T1CTL_IMU_MSG=0\n"
      "T1CTL_IMU_ODOM=1\n"
      "T1CTL_IMU_TF=1\n"
      "T1CTL_ODOM=1\n"
      "T1CTL_ODOM_MSG=1\n"
      "T1CTL_ODOM_TF=1\n"
      "state: 2\n"
      "remote_controller: false\n"};
  runner.warm_out = runner.probe_outs[0];
  const units::Status status = units::query(runner);
  CHECK(status.imu == units::Imu::Degraded);
  CHECK(!status.imu_msg_ok);
  CHECK(status.imu_odom_ok);
  CHECK(status.imu_tf_ok);
  CHECK(count_exec(runner) == 1);
}

void test_odometry_degraded_query() {
  MockRunner runner;
  runner.probe_outs = {
      "T1CTL_CHASSIS=1\n"
      "T1CTL_LIDAR=1\n"
      "T1CTL_LIDAR_SCAN=1\n"
      "T1CTL_LIDAR_TF=1\n"
      "T1CTL_IMU=1\n"
      "T1CTL_IMU_MSG=1\n"
      "T1CTL_IMU_ODOM=1\n"
      "T1CTL_IMU_TF=1\n"
      "T1CTL_ODOM=0\n"
      "T1CTL_ODOM_MSG=1\n"
      "T1CTL_ODOM_TF=0\n"
      "state: 2\n"
      "remote_controller: false\n"};
  runner.warm_out = runner.probe_outs[0];
  const units::Status status = units::query(runner);
  CHECK(status.odometry == units::Odometry::Degraded);
  CHECK(status.odom_msg_ok);
  CHECK(!status.odom_tf_ok);
  CHECK(count_exec(runner) == 1);
}

void test_camera_active_query() {
  MockRunner runner;
  runner.probe_outs = {kLiveYaml};
  runner.warm_out = kLiveYaml;
  const units::Status status = units::query(runner);
  CHECK(status.camera == units::Camera::Active);
  CHECK(status.camera_color_ok);
  CHECK(status.camera_depth_ok);
  CHECK(status.camera_tf_ok);
  CHECK(count_exec(runner) == 1);
}

void test_camera_degraded_query() {
  MockRunner runner;
  runner.probe_outs = {
      "T1CTL_CHASSIS=1\n"
      "T1CTL_LIDAR=1\n"
      "T1CTL_LIDAR_SCAN=1\n"
      "T1CTL_LIDAR_TF=1\n"
      "T1CTL_CAMERA=0\n"
      "T1CTL_CAMERA_COLOR=0\n"
      "T1CTL_CAMERA_DEPTH=1\n"
      "T1CTL_CAMERA_TF=0\n"
      "T1CTL_MODEL=1\n"
      "T1CTL_MODEL_DESCRIPTION=1\n"
      "T1CTL_MODEL_TF_BASE=1\n"
      "T1CTL_MODEL_TF_LIDAR=1\n"
      "T1CTL_MODEL_TF_IMU=1\n"
      "T1CTL_MODEL_TF_DEPTH=1\n"
      "state: 2\n"
      "remote_controller: false\n"};
  runner.warm_out = runner.probe_outs[0];
  const units::Status status = units::query(runner);
  CHECK(status.camera == units::Camera::Degraded);
  CHECK(!status.camera_color_ok);
  CHECK(status.camera_depth_ok);
  CHECK(!status.camera_tf_ok);
  CHECK(count_exec(runner) == 1);
}

void test_model_degraded_query() {
  MockRunner runner;
  runner.probe_outs = {
      "T1CTL_CHASSIS=1\n"
      "T1CTL_LIDAR=1\n"
      "T1CTL_LIDAR_SCAN=1\n"
      "T1CTL_LIDAR_TF=1\n"
      "T1CTL_MODEL=0\n"
      "T1CTL_MODEL_DESCRIPTION=0\n"
      "T1CTL_MODEL_TF_BASE=1\n"
      "T1CTL_MODEL_TF_LIDAR=1\n"
      "T1CTL_MODEL_TF_IMU=0\n"
      "T1CTL_MODEL_TF_DEPTH=1\n"
      "state: 2\n"
      "remote_controller: false\n"};
  runner.warm_out = runner.probe_outs[0];
  const units::Status status = units::query(runner);
  CHECK(status.model == units::Model::Degraded);
  CHECK(!status.model_description_ok);
  CHECK(status.model_tf_base_ok);
  CHECK(status.model_tf_lidar_ok);
  CHECK(!status.model_tf_imu_ok);
  CHECK(status.model_tf_depth_ok);
  CHECK(count_exec(runner) == 1);
}

void test_demo_down_defaults() {
  MockRunner runner;
  runner.demo_active = false;
  runner.container_up = false;
  runner.probe_outs = {kLiveYaml};
  const units::Status status = units::query(runner);
  CHECK(status.demo == units::Demo::Inactive);
  CHECK(status.stock == units::Stock::Inactive);
  CHECK(status.chassis == units::Chassis::Inactive);
  CHECK(!status.have_control);
  CHECK(status.remote_controller == units::RemoteController::Inactive);
  CHECK(status.lidar == units::Lidar::Inactive);
  CHECK(status.camera == units::Camera::Inactive);
  CHECK(status.imu == units::Imu::Inactive);
  CHECK(status.odometry == units::Odometry::Inactive);
  CHECK(!status.have_calibration);
  CHECK(count_exec(runner) == 0);
}

void test_live_container_despite_inactive_unit() {
  MockRunner runner;
  runner.demo_active = false;
  runner.container_up = true;
  runner.probe_outs = {kLiveYaml};
  const units::Status status = units::query(runner);
  CHECK(status.demo == units::Demo::Inactive);
  CHECK(status.chassis == units::Chassis::Active);
  CHECK(status.remote_controller == units::RemoteController::Active);
  CHECK(count_exec(runner) == 1);
}

void test_live_probe_all_active() {
  MockRunner runner;
  runner.probe_outs = {kLiveYaml};
  const units::Status status = units::query(runner);
  CHECK(status.chassis == units::Chassis::Active);
  CHECK(status.have_control);
  CHECK(status.mode == units::Mode::Follow);
  CHECK(status.reason.empty());
  CHECK(status.remote_controller == units::RemoteController::Active);
  CHECK(status.lidar == units::Lidar::Active);
  CHECK(status.camera == units::Camera::Active);
  CHECK(status.imu == units::Imu::Active);
  CHECK(status.odometry == units::Odometry::Active);
  CHECK(status.model == units::Model::Active);
  CHECK(status.have_calibration);
  CHECK(status.calibration == units::Calibration::Factory);
  CHECK(count_exec(runner) == 1);
}

void test_cold_then_warm_queries() {
  MockRunner runner;
  runner.probe_outs = {kChassisDead, kLiveYaml};

  const units::Status cold = units::query(runner);
  CHECK(cold.chassis == units::Chassis::Inactive);
  CHECK(!cold.have_control);
  CHECK(cold.remote_controller == units::RemoteController::Inactive);
  CHECK(count_exec(runner) == 1);

  const units::Status warm = units::query(runner);
  CHECK(warm.chassis == units::Chassis::Active);
  CHECK(warm.have_control);
  CHECK(warm.mode == units::Mode::Follow);
  CHECK(warm.remote_controller == units::RemoteController::Active);
  CHECK(count_exec(runner) == 2);
  CHECK(runner.probe_used() == 2);
}

void test_exhausted_timeout_stays_default() {
  MockRunner runner;
  runner.probe_outs = {kTimeoutOut};
  runner.probe_codes = {124};
  const units::Status status = units::query(runner);
  CHECK(status.chassis == units::Chassis::Inactive);
  CHECK(!status.have_control);
  CHECK(status.remote_controller == units::RemoteController::Inactive);
  CHECK(status.lidar == units::Lidar::Inactive);
  CHECK(status.camera == units::Camera::Inactive);
  CHECK(status.imu == units::Imu::Inactive);
  CHECK(status.odometry == units::Odometry::Inactive);
  CHECK(status.model == units::Model::Inactive);
  CHECK(count_exec(runner) == units::kRosProbeAttempts);
}

const char* last_viewer_probe_script(const MockRunner& runner) {
  for (auto it = runner.calls.rbegin(); it != runner.calls.rend(); ++it) {
    if (it->size() >= 2 && (*it)[0] == "docker" && (*it)[1] == "exec") {
      const char* script = it->back().c_str();
      if (std::strstr(script, "T1CTL_BRIDGE=") != nullptr) {
        return script;
      }
    }
  }
  return "";
}

constexpr const char* kViewerLive = "T1CTL_BRIDGE=1\nT1CTL_PORT=1\nT1CTL_ROS_DOMAIN_ID=1\n";
constexpr const char* kViewerDown = "T1CTL_BRIDGE=0\nT1CTL_PORT=0\nT1CTL_ROS_DOMAIN_ID=1\n";

void test_viewer_constants() {
  CHECK(std::strcmp(viewer::kFixedFrame, "odom") == 0);
  CHECK(std::strcmp(viewer::kRobotDescriptionTopic, "/robot_description") == 0);
  CHECK(std::strcmp(viewer::kModelTfChain, "odom -> base_footprint -> base_link") == 0);
  CHECK(std::strstr(viewer::kModelSensorFrames, "lidar_frame") != nullptr);
  CHECK(std::strstr(viewer::kModelSensorFrames, "imu_link") != nullptr);
  CHECK(std::strstr(viewer::kModelSensorFrames, "depth_cam_frame") != nullptr);
  CHECK(std::strcmp(viewer::kOdomTopic, "/odom_raw") == 0);
  CHECK(std::strcmp(viewer::kLidarScanTopic, "/scan") == 0);
  CHECK(std::strcmp(viewer::kCameraColorTopic, "/aurora/rgb/image_raw") == 0);
  CHECK(std::strcmp(viewer::kCameraColorCompressedTopic, "/aurora/rgb/image_raw/compressed") == 0);
  CHECK(std::strcmp(viewer::kCameraCloudTopic, "/aurora/points2") == 0);
  CHECK(std::strcmp(viewer::kCameraDepthTopic, "/aurora/depth/image_raw") == 0);
  CHECK(std::strcmp(viewer::kCameraFrame, "depth_camera_link") == 0);
  CHECK(std::strcmp(viewer::kCameraTfFrames, "base_footprint -> depth_camera_link") == 0);
  CHECK(std::strstr(viewer::kCamera2dHint, "/aurora/rgb/image_raw/compressed") != nullptr);
  CHECK(std::strstr(viewer::kCamera2dHint, "/aurora/rgb/image_raw") != nullptr);
  CHECK(std::strcmp(viewer::kCamera3dHint, "/aurora/points2") == 0);
  CHECK(std::strcmp(viewer::kPersonsTopic, "/perception/persons") == 0);
  CHECK(std::strcmp(viewer::kNearestPersonTopic, "/perception/nearest_person") == 0);
  CHECK(std::strcmp(viewer::kDetections2dTopic, "/perception/detections_2d") == 0);
  CHECK(std::strcmp(viewer::kPersonsOverlayTopic, "/perception/persons/overlay") == 0);
  CHECK(std::strstr(viewer::kPersonsOverlayHint, "t1ctl debug on") != nullptr);
  CHECK(std::strstr(viewer::kPersonsOverlayHint, "bgr8") != nullptr);
  CHECK(std::strcmp(viewer::kImuTopic, "/imu") == 0);
  CHECK(std::strcmp(viewer::kImuOdomTopic, "/imu_odom") == 0);
  CHECK(std::strcmp(viewer::kImuTfFrames, "base_footprint -> imu_link") == 0);
}

void test_viewer_probe_parse() {
  viewer::Status status;
  CHECK(viewer::parse_probe(kViewerLive, &status));
  CHECK(status.bridge == viewer::Bridge::Active);
  CHECK(status.port_listening);
  CHECK(status.ros_domain_id == "1");

  CHECK(viewer::parse_probe(kViewerDown, &status));
  CHECK(status.bridge == viewer::Bridge::Inactive);
  CHECK(!status.port_listening);
  CHECK(status.ros_domain_id == "1");
}

void test_viewer_status_query() {
  MockRunner runner;
  runner.viewer_outs = {kViewerLive};
  const viewer::Status status = viewer::query(runner);
  CHECK(status.demo == viewer::DemoContour::Active);
  CHECK(status.bridge == viewer::Bridge::Active);
  CHECK(status.port_listening);
  CHECK(status.host == "192.168.2.2");
  CHECK(status.ros_domain_id == "1");
  CHECK(runner.viewer_used() == 1);
}

void test_viewer_probe_script_no_echo() {
  MockRunner runner;
  runner.viewer_outs = {kViewerLive};
  (void)viewer::query(runner);
  const std::string script = last_viewer_probe_script(runner);
  CHECK(script.find("pgrep -x foxglove_bridge") != std::string::npos);
  CHECK(script.find("T1CTL_BRIDGE=") != std::string::npos);
  CHECK(script.find(".hiwonderrc") != std::string::npos);
  CHECK(script.find("ROS_LOCALHOST_ONLY=0") != std::string::npos);
  CHECK(script.find("topic echo") == std::string::npos);
  CHECK(script.find("tf2_echo") == std::string::npos);
  CHECK(script.find("ros2 daemon start") == std::string::npos);
}

void test_viewer_start_without_systemctl() {
  MockRunner runner;
  runner.viewer_outs = {kViewerDown, kViewerDown, kViewerLive};
  runner.viewer_codes = {0, 0, 0};
  viewer::Status status;
  CHECK(viewer::start(runner, &status));
  CHECK(status.bridge == viewer::Bridge::Active);
  CHECK(status.port_listening);
  bool saw_systemctl = false;
  for (const auto& args : runner.calls) {
    if (!args.empty() && args[0] == "sudo") {
      saw_systemctl = true;
    }
  }
  CHECK(!saw_systemctl);
}

void test_viewer_stop_without_systemctl() {
  MockRunner runner;
  runner.viewer_outs = {kViewerLive, "", kViewerDown};
  runner.viewer_codes = {0, 0};
  viewer::Status status;
  CHECK(viewer::stop(runner, &status));
  CHECK(status.bridge == viewer::Bridge::Inactive);
  CHECK(!status.port_listening);
  bool saw_systemctl = false;
  for (const auto& args : runner.calls) {
    if (!args.empty() && args[0] == "sudo") {
      saw_systemctl = true;
    }
  }
  CHECK(!saw_systemctl);
}

void test_viewer_demo_down() {
  MockRunner runner;
  runner.container_up = false;
  viewer::Status status;
  CHECK(!viewer::start(runner, &status));
  CHECK(status.demo == viewer::DemoContour::Inactive);
  CHECK(runner.viewer_used() == 0);
}

void test_viewer_start_failure_shows_diag() {
  MockRunner runner;
  runner.viewer_outs = {
      kViewerDown,
      "T1CTL_STARTED=timeout\n"
      "T1CTL_DIAG=package 'foxglove_bridge' not found\n"
      "T1CTL_DIAG=executable 'foxglove_bridge' not found on the libexec directory\n",
      kViewerDown};
  runner.viewer_codes = {0, 1, 0};
  viewer::Status status;
  std::string detail;
  CHECK(!viewer::start(runner, &status, &detail));
  CHECK(status.bridge == viewer::Bridge::Inactive);
  CHECK(!status.port_listening);
  CHECK(detail.find("foxglove_bridge") != std::string::npos);
}

void test_viewer_start_demo_inactive_detail() {
  MockRunner runner;
  runner.container_up = false;
  viewer::Status status;
  std::string detail;
  CHECK(!viewer::start(runner, &status, &detail));
  CHECK(status.demo == viewer::DemoContour::Inactive);
  CHECK(detail.find("t1ctl start") != std::string::npos);
  CHECK(runner.viewer_used() == 0);
}

class StreamCapture {
 public:
  explicit StreamCapture(std::ostream& stream) : stream_(stream), old_(stream.rdbuf()) {
    stream_.rdbuf(buffer_.rdbuf());
  }
  ~StreamCapture() { stream_.rdbuf(old_); }
  std::string str() const { return buffer_.str(); }

 private:
  std::ostream& stream_;
  std::stringstream buffer_;
  std::streambuf* old_;
};

std::string strip_ansi(const std::string& in) {
  std::string out;
  out.reserve(in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    if (in[i] == '\033' && i + 1 < in.size() && in[i + 1] == '[') {
      i += 2;
      while (i < in.size() && in[i] != 'm') {
        ++i;
      }
      continue;
    }
    out.push_back(in[i]);
  }
  return out;
}

std::string kv_text(const char* key, const char* value) {
  std::string out(key);
  if (out.size() < 18) {
    out.append(18 - out.size(), ' ');
  }
  out += value;
  return out;
}

units::Status make_degraded_status() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.stock = units::Stock::Inactive;
  status.chassis = units::Chassis::Active;
  status.have_control = true;
  status.mode = units::Mode::Follow;
  status.lidar = units::Lidar::Degraded;
  status.lidar_scan_ok = false;
  status.lidar_tf_ok = false;
  status.camera = units::Camera::Degraded;
  status.camera_color_ok = false;
  status.camera_depth_ok = false;
  status.camera_tf_ok = false;
  status.model = units::Model::Degraded;
  status.model_description_ok = false;
  status.model_tf_base_ok = false;
  status.model_tf_lidar_ok = false;
  status.model_tf_imu_ok = false;
  status.model_tf_depth_ok = false;
  status.imu = units::Imu::Degraded;
  status.imu_msg_ok = false;
  status.imu_odom_ok = false;
  status.imu_tf_ok = false;
  status.odometry = units::Odometry::Degraded;
  status.odom_msg_ok = false;
  status.odom_tf_ok = false;
  return status;
}

void test_print_viewer_status_no_hints() {
  viewer::Status status;
  CHECK(viewer::parse_probe(kViewerLive, &status));
  status.demo = viewer::DemoContour::Active;
  StreamCapture capture(std::cout);
  ui::print_viewer_status(status);
  const std::string out = capture.str();
  CHECK(out.find("/scan") != std::string::npos);
  CHECK(out.find("/aurora/rgb/image_raw/compressed") != std::string::npos);
  CHECK(out.find("/aurora/points2") != std::string::npos);
  CHECK(out.find("persons overlay") != std::string::npos);
  CHECK(out.find("/perception/persons") != std::string::npos);
  CHECK(out.find("/perception/nearest_person") != std::string::npos);
  CHECK(out.find("/perception/detections_2d") != std::string::npos);
  CHECK(out.find("/perception/persons/overlay") != std::string::npos);
  CHECK(out.find("t1ctl debug on") != std::string::npos);
  CHECK(out.find("RGB with person boxes; Mac detect script for boxes") == std::string::npos);
  CHECK(out.find("Read-only: no command topics or services from Foxglove") != std::string::npos);
  CHECK(out.find("depth_camera_link") != std::string::npos);
  CHECK(out.find("/imu") != std::string::npos);
  CHECK(out.find("/imu_odom") != std::string::npos);
  CHECK(out.find("base_footprint -> imu_link") != std::string::npos);
  CHECK(out.find("orientation.x/y/z/w") != std::string::npos);
  CHECK(out.find("linear_acceleration.x/y/z") != std::string::npos);
  CHECK(out.find("no LaserScan") == std::string::npos);
  CHECK(out.find("no robot_description") == std::string::npos);
  CHECK(out.find("no color on") == std::string::npos);
}

void test_print_status_degraded_no_hints() {
  const units::Status status = make_degraded_status();
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = capture.str();
  CHECK(out.find("degraded") != std::string::npos);
  CHECK(out.find("no LaserScan") == std::string::npos);
  CHECK(out.find("no color on") == std::string::npos);
  CHECK(out.find("no robot_description") == std::string::npos);
  CHECK(out.find("docker logs") == std::string::npos);
}

void test_print_status_active_sensors() {
  units::Status status = make_degraded_status();
  status.lidar = units::Lidar::Active;
  status.camera = units::Camera::Active;
  status.imu = units::Imu::Active;
  status.odometry = units::Odometry::Active;
  status.model = units::Model::Active;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = capture.str();
  CHECK(out.find("lidar") != std::string::npos);
  CHECK(out.find("camera") != std::string::npos);
  CHECK(out.find("platform model") != std::string::npos);
  CHECK(out.find("active") != std::string::npos);
  CHECK(out.find("degraded") == std::string::npos);
}

void test_print_started_no_kv() {
  StreamCapture capture(std::cout);
  ui::print_started();
  const std::string out = capture.str();
  CHECK(out == "Started.\n");
  CHECK(out.find("demo") == std::string::npos);
  CHECK(out.find("chassis") == std::string::npos);
}

void test_print_restarted_no_kv() {
  StreamCapture capture(std::cout);
  ui::print_restarted();
  CHECK(capture.str() == "Restarted.\n");
}

void test_print_stock_no_kv() {
  StreamCapture capture(std::cout);
  ui::print_stock();
  CHECK(capture.str() == "Restored stock autostart.\n");
}

void test_print_action_error_no_kv() {
  StreamCapture cout_capture(std::cout);
  StreamCapture cerr_capture(std::cerr);
  ui::print_action_error();
  const std::string cout_out = cout_capture.str();
  const std::string cerr_out = cerr_capture.str();
  CHECK(cout_out.find("demo") == std::string::npos);
  CHECK(cout_out.find("chassis") == std::string::npos);
  CHECK(cout_out.find("t1ctl start") != std::string::npos);
  CHECK(cout_out.find("t1ctl stock") != std::string::npos);
  CHECK(cerr_out.find("demo start failed") != std::string::npos);
}

void test_print_version() {
  StreamCapture capture(std::cout);
  ui::print_version();
  const std::string out = capture.str();
  CHECK(std::string(T1CTL_VERSION) == "1.9.0");
  CHECK(out == "t1ctl " + std::string(T1CTL_VERSION) + "\n");
  CHECK(out.find("1.9.0") != std::string::npos);
}

void test_print_status_version_kv() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.stock = units::Stock::Inactive;
  status.have_control = true;
  status.mode = units::Mode::Follow;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find("version") != std::string::npos);
  CHECK(out.find("1.9.0") != std::string::npos);
  CHECK(out.find(kv_text("mode", "follow")) != std::string::npos);
  CHECK(out.find(kv_text("dds buffers", "unknown")) != std::string::npos);
  CHECK(out.find("reason") == std::string::npos);
  CHECK(out.find("forbidden") == std::string::npos);
  CHECK(out.find("calibration") == std::string::npos);
  CHECK(out.find("FOXGLOVE DESKTOP") == std::string::npos);
}

void test_print_status_imu_degraded_hint() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.have_control = true;
  status.mode = units::Mode::Follow;
  status.imu = units::Imu::Degraded;
  status.imu_msg_ok = false;
  status.imu_odom_ok = true;
  status.imu_tf_ok = true;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = capture.str();
  CHECK(out.find("no /imu") != std::string::npos);
}

void test_print_status_odometry_degraded_hint() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.have_control = true;
  status.mode = units::Mode::Follow;
  status.odometry = units::Odometry::Degraded;
  status.odom_msg_ok = false;
  status.odom_tf_ok = false;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = capture.str();
  CHECK(out.find("no /odom_raw") != std::string::npos);
}

void test_print_status_forbidden_operator() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.stock = units::Stock::Inactive;
  status.chassis = units::Chassis::Active;
  status.have_control = true;
  status.mode = units::Mode::Forbidden;
  status.reason = "operator";
  status.remote_controller = units::RemoteController::Active;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("mode", "forbidden")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "operator")) != std::string::npos);
  CHECK(out.find(kv_text("mode", "follow")) == std::string::npos);
  CHECK(out.find("  t1ctl mode allow\n") != std::string::npos);
}

void test_print_status_forbidden_topic_reason() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.stock = units::Stock::Inactive;
  status.have_control = true;
  status.mode = units::Mode::Forbidden;
  status.reason = "no /pnc/desired_twist";
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("mode", "forbidden")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "no /pnc/desired_twist")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "operator")) == std::string::npos);
}

void test_print_status_missing_control_not_follow() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.stock = units::Stock::Inactive;
  status.chassis = units::Chassis::Active;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(!status.have_control);
  CHECK(out.find(kv_text("mode", "follow")) == std::string::npos);
  CHECK(out.find(kv_text("mode", "forbidden")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "no /control/status")) != std::string::npos);
  CHECK(out.find("  t1ctl mode allow\n") != std::string::npos);
}

void test_print_status_demo_down_not_follow() {
  units::Status status;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("mode", "follow")) == std::string::npos);
  CHECK(out.find(kv_text("mode", "forbidden")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "no /control/status")) != std::string::npos);
  CHECK(out.find("  t1ctl start\n") != std::string::npos);
  CHECK(out.find("t1ctl mode allow") == std::string::npos);
}

void test_print_help_forbidden_dictionary() {
  StreamCapture capture(std::cout);
  ui::print_help();
  const std::string out = capture.str();
  CHECK(out.find("forbidden") != std::string::npos);
  CHECK(out.find("motion held; chassis zeros") != std::string::npos);
  CHECK(out.find("reason") != std::string::npos);
  CHECK(out.find("host command t1ctl mode forbid") != std::string::npos);
  CHECK(out.find("critical contour signal missing") != std::string::npos);
}

void test_forbidden_query() {
  MockRunner runner;
  runner.probe_outs = {
      "T1CTL_CHASSIS=1\n"
      "state: 0\n"
      "remote_controller: true\n"
      "reason: operator\n"};
  const units::Status status = units::query(runner);
  CHECK(status.have_control);
  CHECK(status.mode == units::Mode::Forbidden);
  CHECK(status.reason == "operator");
  CHECK(status.chassis == units::Chassis::Active);
}

void test_missing_control_query() {
  MockRunner runner;
  runner.probe_outs = {kChassisDead};
  const units::Status status = units::query(runner);
  CHECK(!status.have_control);
  CHECK(status.demo == units::Demo::Active);
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("mode", "follow")) == std::string::npos);
  CHECK(out.find(kv_text("reason", "no /control/status")) != std::string::npos);
}

void test_parse_mode_result_success() {
  units::ModeChange change;
  CHECK(units::parse_mode_result("T1CTL_MODE_OK=1\nfrom: 2\nto: 0\nreason: operator\n", &change));
  CHECK(change.ok);
  CHECK(change.have_from);
  CHECK(change.have_to);
  CHECK(change.from == units::Mode::Follow);
  CHECK(change.to == units::Mode::Forbidden);
  CHECK(change.reason == "operator");

  CHECK(units::parse_mode_result("Welcome to Hiwonder\nT1CTL_MODE_OK=1\nfrom: 0\nto: 2\nreason:\n",
                                 &change));
  CHECK(change.ok);
  CHECK(change.from == units::Mode::Forbidden);
  CHECK(change.to == units::Mode::Follow);
  CHECK(change.reason.empty());

  CHECK(units::parse_mode_result("T1CTL_MODE_OK=1\nfrom: 2\nto: 1\n", &change));
  CHECK(change.ok);
  CHECK(change.to == units::Mode::Manual);
  CHECK(change.reason.empty());

  CHECK(units::parse_mode_result(
      "T1CTL_MODE_OK=1\nfrom: AUTO_FOLLOW\nto: FORBIDDEN\nreason: operator\n", &change));
  CHECK(change.from == units::Mode::Follow);
  CHECK(change.to == units::Mode::Forbidden);
}

void test_parse_mode_result_failure() {
  units::ModeChange change;
  CHECK(units::parse_mode_result(
      "T1CTL_MODE_OK=0\ndetail: service /control/set_mode is not available\n", &change));
  CHECK(!change.ok);
  CHECK(change.detail.find("service /control/set_mode is not available") != std::string::npos);

  CHECK(
      units::parse_mode_result("T1CTL_MODE_OK=0\n"
                               "detail: service /control/set_mode is not available\n"
                               "detail: extra\n",
                               &change));
  CHECK(change.detail.find("service /control/set_mode is not available") != std::string::npos);
  CHECK(change.detail.find("extra") != std::string::npos);

  units::ModeChange untouched;
  untouched.ok = true;
  untouched.to = units::Mode::Manual;
  CHECK(!units::parse_mode_result("banner only\n", &untouched));
  CHECK(untouched.ok);
  CHECK(untouched.to == units::Mode::Manual);
}

void test_set_mode_forbid_allow_manual() {
  MockRunner runner;
  runner.mode_outs = {"T1CTL_MODE_OK=1\nfrom: 2\nto: 0\nreason: operator\n",
                      "T1CTL_MODE_OK=1\nfrom: 0\nto: 2\nreason:\n",
                      "T1CTL_MODE_OK=1\nfrom: 2\nto: 1\nreason:\n"};
  units::ModeChange change;
  units::Status known;
  CHECK(units::set_mode(runner, units::ModeCommand::Forbid, &change, &known));
  CHECK(change.ok);
  CHECK(change.from == units::Mode::Follow);
  CHECK(change.to == units::Mode::Forbidden);
  CHECK(change.reason == "operator");
  CHECK(known.demo == units::Demo::Active);
  CHECK(runner.mode_used() == 1);
  CHECK(runner.probe_used() == 0);

  const std::string script = last_mode_script(runner);
  CHECK(script.find("python3") != std::string::npos);
  CHECK(script.find("T1CTL_MODE_TARGET=0") != std::string::npos);
  CHECK(script.find("SetControlMode") != std::string::npos);
  CHECK(script.find(units::kControlSetModeService) != std::string::npos);
  CHECK(script.find("create_client") != std::string::npos);
  CHECK(script.find("create_publisher") == std::string::npos);
  CHECK(script.find("topic pub") == std::string::npos);

  CHECK(units::set_mode(runner, units::ModeCommand::Allow, &change, &known));
  CHECK(change.from == units::Mode::Forbidden);
  CHECK(change.to == units::Mode::Follow);
  CHECK(change.reason.empty());
  CHECK(std::string(last_mode_script(runner)).find("T1CTL_MODE_TARGET=2") != std::string::npos);

  CHECK(units::set_mode(runner, units::ModeCommand::Manual, &change, &known));
  CHECK(change.to == units::Mode::Manual);
  CHECK(std::string(last_mode_script(runner)).find("T1CTL_MODE_TARGET=1") != std::string::npos);
}

void test_set_mode_container_down() {
  MockRunner runner;
  runner.container_up = false;
  runner.demo_active = false;
  units::ModeChange change;
  units::Status known;
  CHECK(!units::set_mode(runner, units::ModeCommand::Allow, &change, &known));
  CHECK(!change.ok);
  CHECK(runner.mode_used() == 0);
  CHECK(runner.probe_used() == 0);
  CHECK(known.demo == units::Demo::Inactive);
  CHECK(known.stock == units::Stock::Inactive);
  CHECK(!known.have_control);
  CHECK(change.detail.find("container mentorpi-t1 is not running") != std::string::npos);
  CHECK(change.detail.find("/control/state has no publisher") != std::string::npos);
  CHECK(change.detail.find("motion is not allowed") != std::string::npos);
}

void test_set_mode_service_missing() {
  MockRunner runner;
  runner.mode_outs = {"T1CTL_MODE_OK=0\ndetail: service /control/set_mode is not available\n"};
  runner.mode_codes = {1};
  units::ModeChange change;
  units::Status known;
  CHECK(!units::set_mode(runner, units::ModeCommand::Forbid, &change, &known));
  CHECK(!change.ok);
  CHECK(runner.mode_used() == 1);
  CHECK(change.detail.find("service /control/set_mode is not available") != std::string::npos);
  CHECK(change.detail.find("motion is not allowed") != std::string::npos);
}

void test_set_mode_helper_empty_output() {
  MockRunner runner;
  runner.mode_outs = {""};
  units::ModeChange change;
  units::Status known;
  CHECK(!units::set_mode(runner, units::ModeCommand::Forbid, &change, &known));
  CHECK(!change.ok);
  CHECK(change.detail.find("mode helper failed") != std::string::npos);
  CHECK(change.detail.find("motion is not allowed") != std::string::npos);
}

void test_print_mode_forbid() {
  units::ModeChange change;
  change.ok = true;
  change.have_from = true;
  change.have_to = true;
  change.from = units::Mode::Follow;
  change.to = units::Mode::Forbidden;
  change.reason = "operator";
  StreamCapture capture(std::cout);
  ui::print_mode_changed(change);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find("Motion held.\n") != std::string::npos);
  CHECK(out.find(kv_text("mode", "follow -> forbidden")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "operator")) != std::string::npos);
  CHECK(out.find("demo") == std::string::npos);
}

void test_print_mode_allow() {
  units::ModeChange change;
  change.ok = true;
  change.have_from = true;
  change.have_to = true;
  change.from = units::Mode::Forbidden;
  change.to = units::Mode::Follow;
  StreamCapture capture(std::cout);
  ui::print_mode_changed(change);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find("Motion allowed.\n") != std::string::npos);
  CHECK(out.find(kv_text("mode", "forbidden -> follow")) != std::string::npos);
  CHECK(out.find("reason") == std::string::npos);
}

void test_print_mode_manual() {
  units::ModeChange change;
  change.ok = true;
  change.have_from = true;
  change.have_to = true;
  change.from = units::Mode::Follow;
  change.to = units::Mode::Manual;
  StreamCapture capture(std::cout);
  ui::print_mode_changed(change);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find("Manual.\n") != std::string::npos);
  CHECK(out.find(kv_text("mode", "follow -> manual")) != std::string::npos);
  CHECK(out.find("reason") == std::string::npos);
}

void test_print_mode_error_container_down() {
  units::Status status;
  StreamCapture cout_capture(std::cout);
  StreamCapture cerr_capture(std::cerr);
  ui::print_mode_error(status,
                       "container mentorpi-t1 is not running\n"
                       "/control/state has no publisher\n"
                       "motion is not allowed");
  const std::string cout_out = strip_ansi(cout_capture.str());
  const std::string cerr_out = strip_ansi(cerr_capture.str());
  CHECK(cout_out.find(kv_text("demo", "inactive")) != std::string::npos);
  CHECK(cout_out.find(kv_text("stock", "inactive")) != std::string::npos);
  CHECK(cout_out.find(kv_text("mode", "follow")) == std::string::npos);
  CHECK(cout_out.find("forbidden") == std::string::npos);
  CHECK(cout_out.find("  t1ctl start\n") != std::string::npos);
  CHECK(cerr_out.find("error: mode change failed\n") != std::string::npos);
  CHECK(cerr_out.find("  container mentorpi-t1 is not running\n") != std::string::npos);
  CHECK(cerr_out.find("  /control/state has no publisher\n") != std::string::npos);
  CHECK(cerr_out.find("  motion is not allowed\n") != std::string::npos);
}

void test_print_help_mode_section() {
  StreamCapture capture(std::cout);
  ui::print_help();
  const std::string out = capture.str();
  CHECK(out.find("set control mode") != std::string::npos);
  CHECK(out.find("MODE") != std::string::npos);
  CHECK(out.find("hold motion (Forbidden)") != std::string::npos);
  CHECK(out.find("release hold, return to follow") != std::string::npos);
  CHECK(out.find("select operator pad") != std::string::npos);
}

void test_parse_debug_result_success() {
  units::DebugChange change;
  CHECK(units::parse_debug_result("T1CTL_DEBUG_OK=1\noverlay: on\n", &change));
  CHECK(change.ok);
  CHECK(change.have_overlay);
  CHECK(change.overlay_on);

  CHECK(
      units::parse_debug_result("Welcome to Hiwonder\nT1CTL_DEBUG_OK=1\noverlay: off\n", &change));
  CHECK(change.ok);
  CHECK(change.have_overlay);
  CHECK(!change.overlay_on);
}

void test_parse_debug_result_failure() {
  units::DebugChange change;
  CHECK(units::parse_debug_result(
      "T1CTL_DEBUG_OK=0\ndetail: node /person_perception is not available\n", &change));
  CHECK(!change.ok);
  CHECK(change.detail.find("node /person_perception is not available") != std::string::npos);

  units::DebugChange untouched;
  untouched.ok = true;
  untouched.overlay_on = true;
  CHECK(!units::parse_debug_result("banner only\n", &untouched));
  CHECK(untouched.ok);
  CHECK(untouched.overlay_on);

  CHECK(!units::parse_debug_result("T1CTL_DEBUG_OK=1\n", &change));
}

void test_run_debug_on_off_status() {
  MockRunner runner;
  runner.debug_outs = {"T1CTL_DEBUG_OK=1\noverlay: on\n", "T1CTL_DEBUG_OK=1\noverlay: off\n",
                       "T1CTL_DEBUG_OK=1\noverlay: off\n"};
  runner.viewer_outs = {kViewerDown, "T1CTL_STARTED=ok\n", kViewerLive, kViewerLive,
                        "",          kViewerDown,          kViewerDown};
  units::DebugChange change;
  units::Status known;
  CHECK(units::run_debug(runner, units::DebugCommand::On, &change, &known));
  CHECK(change.ok);
  CHECK(change.overlay_on);
  CHECK(change.have_bridge);
  CHECK(change.bridge_on);
  CHECK(change.websocket_host == "192.168.2.2");
  CHECK(known.demo == units::Demo::Active);
  CHECK(runner.debug_used() == 1);
  CHECK(runner.viewer_used() == 3);
  CHECK(runner.probe_used() == 0);

  const std::string on_script = last_debug_script(runner);
  CHECK(on_script.find("python3") != std::string::npos);
  CHECK(on_script.find("T1CTL_DEBUG_ACTION=on") != std::string::npos);
  CHECK(on_script.find("get_parameters") != std::string::npos);
  CHECK(on_script.find("set_parameters") != std::string::npos);
  CHECK(on_script.find(units::kPersonPerceptionNode) != std::string::npos);
  CHECK(on_script.find(units::kPublishOverlayParam) != std::string::npos);
  CHECK(on_script.find("PYTHONUNBUFFERED=1") != std::string::npos);
  CHECK(on_script.find("ROS_LOCALHOST_ONLY=0") != std::string::npos);
  CHECK(on_script.find(".hiwonderrc >/dev/null") != std::string::npos);
  CHECK(on_script.find("ros2 param") == std::string::npos);
  CHECK(on_script.find("person_perception.yaml") == std::string::npos);

  StreamCapture on_capture(std::cout);
  ui::print_debug(change);
  const std::string on_out = on_capture.str();
  CHECK(on_out.find("overlay: on\n") != std::string::npos);
  CHECK(on_out.find("bridge: on\n") != std::string::npos);
  CHECK(on_out.find("websocket: ws://192.168.2.2:8765\n") != std::string::npos);

  CHECK(units::run_debug(runner, units::DebugCommand::Off, &change, &known));
  CHECK(change.ok);
  CHECK(!change.overlay_on);
  CHECK(change.have_bridge);
  CHECK(!change.bridge_on);
  CHECK(std::string(last_debug_script(runner)).find("T1CTL_DEBUG_ACTION=off") != std::string::npos);
  CHECK(runner.viewer_used() == 6);

  CHECK(units::run_debug(runner, units::DebugCommand::Status, &change, &known));
  CHECK(change.ok);
  CHECK(!change.overlay_on);
  CHECK(change.have_bridge);
  CHECK(!change.bridge_on);
  CHECK(std::string(last_debug_script(runner)).find("T1CTL_DEBUG_ACTION=status") !=
        std::string::npos);
  CHECK(runner.viewer_used() == 7);
}

void test_run_debug_container_down() {
  MockRunner runner;
  runner.container_up = false;
  runner.demo_active = false;
  units::DebugChange change;
  units::Status known;
  CHECK(!units::run_debug(runner, units::DebugCommand::On, &change, &known));
  CHECK(!change.ok);
  CHECK(runner.debug_used() == 0);
  CHECK(runner.viewer_used() == 0);
  CHECK(known.demo == units::Demo::Inactive);
  CHECK(change.detail.find("container mentorpi-t1 is not running") != std::string::npos);
}

void test_run_debug_node_missing() {
  MockRunner runner;
  runner.debug_outs = {"T1CTL_DEBUG_OK=0\ndetail: node /person_perception is not available\n"};
  runner.debug_codes = {1};
  runner.viewer_outs = {kViewerDown};
  units::DebugChange change;
  units::Status known;
  CHECK(!units::run_debug(runner, units::DebugCommand::Status, &change, &known));
  CHECK(!change.ok);
  CHECK(runner.debug_used() == 1);
  CHECK(runner.viewer_used() == 1);
  CHECK(change.have_bridge);
  CHECK(!change.bridge_on);
  CHECK(change.detail.find("node /person_perception is not available") != std::string::npos);
}

void test_run_debug_overlay_fail_still_starts_bridge() {
  MockRunner runner;
  runner.debug_outs = {""};
  runner.viewer_outs = {kViewerDown, "T1CTL_STARTED=ok\n", kViewerLive};
  units::DebugChange change;
  units::Status known;
  CHECK(!units::run_debug(runner, units::DebugCommand::On, &change, &known));
  CHECK(!change.ok);
  CHECK(change.detail.find("debug helper failed") != std::string::npos);
  CHECK(change.have_bridge);
  CHECK(change.bridge_on);
  CHECK(runner.viewer_used() == 3);
}

void test_print_debug_on_off() {
  units::DebugChange change;
  change.ok = true;
  change.have_overlay = true;
  change.overlay_on = true;
  change.have_bridge = true;
  change.bridge_on = true;
  change.websocket_host = "192.168.88.56";
  StreamCapture capture(std::cout);
  ui::print_debug(change);
  CHECK(capture.str() ==
        "T1CTL_DEBUG_OK=1\noverlay: on\nbridge: on\nwebsocket: ws://192.168.88.56:8765\n");

  change.overlay_on = false;
  change.bridge_on = false;
  StreamCapture off_capture(std::cout);
  ui::print_debug(change);
  CHECK(off_capture.str() == "T1CTL_DEBUG_OK=1\noverlay: off\nbridge: off\n");
}

void test_print_debug_error_node_missing() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.stock = units::Stock::Inactive;
  StreamCapture cout_capture(std::cout);
  StreamCapture cerr_capture(std::cerr);
  ui::print_debug_error(status, "node /person_perception is not available");
  const std::string cout_out = strip_ansi(cout_capture.str());
  const std::string cerr_out = strip_ansi(cerr_capture.str());
  CHECK(cout_out.find("T1CTL_DEBUG_OK=0\n") != std::string::npos);
  CHECK(cout_out.find(kv_text("demo", "active")) != std::string::npos);
  CHECK(cout_out.find("overlay:") == std::string::npos);
  CHECK(cerr_out.find("error: debug failed\n") != std::string::npos);
  CHECK(cerr_out.find("  node /person_perception is not available\n") != std::string::npos);
}

void test_print_help_debug_section() {
  StreamCapture capture(std::cout);
  ui::print_help();
  const std::string out = capture.str();
  CHECK(out.find("persons overlay and Foxglove bridge") != std::string::npos);
  CHECK(out.find("DEBUG") != std::string::npos);
  CHECK(out.find("debug on") != std::string::npos);
  CHECK(out.find("publish persons overlay and start Foxglove bridge") != std::string::npos);
  CHECK(out.find("stop persons overlay and Foxglove bridge") != std::string::npos);
  CHECK(out.find("print overlay and bridge on/off") != std::string::npos);
  CHECK(out.find("viewer start") == std::string::npos);
  CHECK(out.find("viewer stop") == std::string::npos);
  CHECK(out.find("viewer status") == std::string::npos);
  CHECK(out.find("VIEWER") == std::string::npos);
}

void test_parse_detect_result_success() {
  units::DetectChange change;
  CHECK(units::parse_detect_result("T1CTL_DETECT_OK=1\nsource: offline\n", &change));
  CHECK(change.ok);
  CHECK(change.have_source);
  CHECK(change.source_offline);

  CHECK(
      units::parse_detect_result("Welcome to Hiwonder\nT1CTL_DETECT_OK=1\nsource: mac\n", &change));
  CHECK(change.ok);
  CHECK(change.have_source);
  CHECK(!change.source_offline);
}

void test_parse_detect_result_failure() {
  units::DetectChange change;
  CHECK(units::parse_detect_result(
      "T1CTL_DETECT_OK=0\ndetail: node /person_detect_pi is not available\n", &change));
  CHECK(!change.ok);
  CHECK(change.detail.find("node /person_detect_pi is not available") != std::string::npos);

  units::DetectChange untouched;
  untouched.ok = true;
  untouched.source_offline = true;
  CHECK(!units::parse_detect_result("banner only\n", &untouched));
  CHECK(untouched.ok);
  CHECK(untouched.source_offline);

  CHECK(!units::parse_detect_result("T1CTL_DETECT_OK=1\n", &change));
}

void test_detect_source_offline_mac_status() {
  MockRunner runner;
  runner.detect_outs = {"T1CTL_DETECT_OK=1\nsource: offline\n", "T1CTL_DETECT_OK=1\nsource: mac\n",
                        "T1CTL_DETECT_OK=1\nsource: mac\n"};
  units::DetectChange change;
  units::Status known;
  CHECK(units::detect_source(runner, units::DetectCommand::Offline, &change, &known));
  CHECK(change.ok);
  CHECK(change.source_offline);
  CHECK(known.demo == units::Demo::Active);
  CHECK(runner.detect_used() == 1);
  CHECK(runner.probe_used() == 0);

  const std::string offline_script = last_detect_script(runner);
  CHECK(offline_script.find("python3") != std::string::npos);
  CHECK(offline_script.find("T1CTL_DETECT_ACTION=offline") != std::string::npos);
  CHECK(offline_script.find("get_parameters") != std::string::npos);
  CHECK(offline_script.find("set_parameters") != std::string::npos);
  CHECK(offline_script.find(units::kPersonPerceptionNode) != std::string::npos);
  CHECK(offline_script.find(units::kPersonDetectPiNode) != std::string::npos);
  CHECK(offline_script.find(units::kDetectionsSourceParam) != std::string::npos);
  CHECK(offline_script.find(units::kDetectEnabledParam) != std::string::npos);
  CHECK(offline_script.find("PYTHONUNBUFFERED=1") != std::string::npos);
  CHECK(offline_script.find("ROS_LOCALHOST_ONLY=0") != std::string::npos);
  CHECK(offline_script.find(".hiwonderrc >/dev/null") != std::string::npos);
  CHECK(offline_script.find("ros2 param") == std::string::npos);

  CHECK(units::detect_source(runner, units::DetectCommand::Mac, &change, &known));
  CHECK(change.ok);
  CHECK(!change.source_offline);
  CHECK(std::string(last_detect_script(runner)).find("T1CTL_DETECT_ACTION=mac") !=
        std::string::npos);

  CHECK(units::detect_source(runner, units::DetectCommand::Status, &change, &known));
  CHECK(change.ok);
  CHECK(!change.source_offline);
  CHECK(std::string(last_detect_script(runner)).find("T1CTL_DETECT_ACTION=status") !=
        std::string::npos);
}

void test_detect_source_container_down() {
  MockRunner runner;
  runner.container_up = false;
  runner.demo_active = false;
  units::DetectChange change;
  units::Status known;
  CHECK(!units::detect_source(runner, units::DetectCommand::Offline, &change, &known));
  CHECK(!change.ok);
  CHECK(runner.detect_used() == 0);
  CHECK(known.demo == units::Demo::Inactive);
  CHECK(change.detail.find("container mentorpi-t1 is not running") != std::string::npos);
}

void test_detect_source_node_missing() {
  MockRunner runner;
  runner.detect_outs = {"T1CTL_DETECT_OK=0\ndetail: node /person_detect_pi is not available\n"};
  runner.detect_codes = {1};
  units::DetectChange change;
  units::Status known;
  CHECK(!units::detect_source(runner, units::DetectCommand::Status, &change, &known));
  CHECK(!change.ok);
  CHECK(runner.detect_used() == 1);
  CHECK(change.detail.find("node /person_detect_pi is not available") != std::string::npos);
}

void test_detect_source_helper_empty_output() {
  MockRunner runner;
  runner.detect_outs = {""};
  units::DetectChange change;
  units::Status known;
  CHECK(!units::detect_source(runner, units::DetectCommand::Offline, &change, &known));
  CHECK(!change.ok);
  CHECK(change.detail.find("detect helper failed") != std::string::npos);
}

void test_print_detect_offline_mac() {
  units::DetectChange change;
  change.ok = true;
  change.have_source = true;
  change.source_offline = true;
  StreamCapture capture(std::cout);
  ui::print_detect(change);
  CHECK(capture.str() == "T1CTL_DETECT_OK=1\nsource: offline\n");

  change.source_offline = false;
  StreamCapture mac_capture(std::cout);
  ui::print_detect(change);
  CHECK(mac_capture.str() == "T1CTL_DETECT_OK=1\nsource: mac\n");
}

void test_print_detect_error_node_missing() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.stock = units::Stock::Inactive;
  StreamCapture cout_capture(std::cout);
  StreamCapture cerr_capture(std::cerr);
  ui::print_detect_error(status, "node /person_detect_pi is not available");
  const std::string cout_out = strip_ansi(cout_capture.str());
  const std::string cerr_out = strip_ansi(cerr_capture.str());
  CHECK(cout_out.find("T1CTL_DETECT_OK=0\n") != std::string::npos);
  CHECK(cout_out.find(kv_text("demo", "active")) != std::string::npos);
  CHECK(cout_out.find("source:") == std::string::npos);
  CHECK(cerr_out.find("error: detect failed\n") != std::string::npos);
  CHECK(cerr_out.find("  node /person_detect_pi is not available\n") != std::string::npos);
}

void test_print_help_detect_section() {
  StreamCapture capture(std::cout);
  ui::print_help();
  const std::string out = capture.str();
  CHECK(out.find("person detections source") != std::string::npos);
  CHECK(out.find("DETECT") != std::string::npos);
  CHECK(out.find("detect offline") != std::string::npos);
  CHECK(out.find("use onboard YOLO11n") != std::string::npos);
  CHECK(out.find("detect mac") != std::string::npos);
  CHECK(out.find("use Mac detections") != std::string::npos);
  CHECK(out.find("print source mac/offline") != std::string::npos);
}

constexpr const char* kCalibFactoryOut =
    "T1CTL_CALIB_OK=1\n"
    "source: factory\n"
    "residual: 0.42 87\n"
    "pose: camera xyz 0.1 0 0.177 factory\n"
    "pose: camera rpy 0 0 0 factory\n"
    "pose: lidar xyz 0.09 0 0.1675195774179554 factory\n"
    "pose: lidar rpy 0 0 180 factory\n"
    "pose: imu xyz 0.0048416 0.011168 0.1212602 factory\n"
    "pose: imu rpy 0 0 0 factory\n";

void test_parse_ros_probe_calibration() {
  units::Status status;
  const units::RosProbeView factory = units::parse_ros_probe("T1CTL_CALIB=factory\n", &status);
  CHECK(factory.have_calibration);
  CHECK(factory.calibration == units::Calibration::Factory);

  const units::RosProbeView file = units::parse_ros_probe("T1CTL_CALIB=file\n", &status);
  CHECK(file.have_calibration);
  CHECK(file.calibration == units::Calibration::File);

  const units::RosProbeView unused = units::parse_ros_probe("T1CTL_CALIB=unused\n", &status);
  CHECK(unused.have_calibration);
  CHECK(unused.calibration == units::Calibration::Unused);

  const units::RosProbeView missing = units::parse_ros_probe("T1CTL_CHASSIS=1\n", &status);
  CHECK(!missing.have_calibration);
}

void test_query_calibration_from_probe() {
  MockRunner runner;
  runner.probe_outs = {kLiveYaml};
  const units::Status status = units::query(runner);
  CHECK(status.have_calibration);
  CHECK(status.calibration == units::Calibration::Factory);
  CHECK(runner.calib_used() == 0);
}

void test_query_calibration_unused_from_probe() {
  MockRunner runner;
  runner.probe_outs = {"T1CTL_CALIB=unused\nT1CTL_CHASSIS=1\nstate: 2\nremote_controller: true\n"};
  const units::Status status = units::query(runner);
  CHECK(status.have_calibration);
  CHECK(status.calibration == units::Calibration::Unused);
  CHECK(status.have_control);
  CHECK(status.mode == units::Mode::Follow);
  CHECK(status.reason.empty());
}

void test_parse_calib_show_factory() {
  calib::Show show;
  CHECK(calib::parse_show(kCalibFactoryOut, &show));
  CHECK(show.ok);
  CHECK(show.have_source);
  CHECK(show.source == units::Calibration::Factory);
  CHECK(show.reason.empty());
  CHECK(show.have_residual);
  CHECK(std::fabs(show.residual_m - 0.42) < 1e-9);
  CHECK(std::fabs(show.residual_deg - 87.0) < 1e-9);
  CHECK(show.have_camera);
  CHECK(show.have_lidar);
  CHECK(show.have_imu);
  CHECK(std::fabs(show.camera.z - 0.177) < 1e-9);
  CHECK(std::fabs(show.lidar.yaw - 180.0) < 1e-9);
  CHECK(show.camera.xyz_source == "factory");
  CHECK(show.lidar.rpy_source == "factory");
}

void test_parse_calib_show_unused() {
  calib::Show show;
  CHECK(
      calib::parse_show("T1CTL_CALIB_OK=1\n"
                        "source: unused\n"
                        "reason: invalid schema\n"
                        "pose: camera xyz 0.1 0 0.177 factory\n"
                        "pose: camera rpy 0 0 0 factory\n",
                        &show));
  CHECK(show.ok);
  CHECK(show.source == units::Calibration::Unused);
  CHECK(show.reason == "invalid schema");
  CHECK(show.have_camera);
  CHECK(!show.have_lidar);
}

void test_parse_calib_show_failure() {
  calib::Show show;
  CHECK(calib::parse_show("T1CTL_CALIB_OK=0\ndetail: helper exploded\n", &show));
  CHECK(!show.ok);
  CHECK(show.detail.find("helper exploded") != std::string::npos);

  calib::Show untouched;
  untouched.ok = true;
  CHECK(!calib::parse_show("banner only\n", &untouched));
  CHECK(untouched.ok);
}

void test_calib_query_factory() {
  MockRunner runner;
  runner.calib_outs = {kCalibFactoryOut};
  calib::Show show;
  CHECK(calib::query(runner, &show));
  CHECK(show.ok);
  CHECK(show.source == units::Calibration::Factory);
  CHECK(show.have_camera);
  CHECK(show.have_lidar);
  CHECK(show.have_imu);
  CHECK(runner.calib_used() == 1);
  CHECK(runner.probe_used() == 0);
  bool saw_show = false;
  for (const auto& args : runner.calls) {
    if (args.size() >= 2 && args[0] == "docker" && args[1] == "exec" &&
        std::strstr(args.back().c_str(), "ros2 run mentorpi_calibration calib show") != nullptr) {
      saw_show = true;
    }
  }
  CHECK(saw_show);
}

void test_calib_query_container_down() {
  MockRunner runner;
  runner.container_up = false;
  runner.demo_active = false;
  runner.calib_outs = {kCalibFactoryOut};
  calib::Show show;
  CHECK(!calib::query(runner, &show));
  CHECK(!show.ok);
  CHECK(runner.calib_used() == 0);
  CHECK(!show.have_camera);
  CHECK(!show.have_lidar);
  CHECK(!show.have_imu);
  CHECK(show.demo == units::Demo::Inactive);
  CHECK(show.detail.find("container mentorpi-t1 is not running") != std::string::npos);
}

void test_calib_query_helper_failed() {
  MockRunner runner;
  runner.calib_outs = {""};
  calib::Show show;
  CHECK(!calib::query(runner, &show));
  CHECK(!show.ok);
  CHECK(!show.have_camera);
  CHECK(show.detail.find("calib helper failed") != std::string::npos);
}

void test_print_calib_factory() {
  calib::Show show;
  CHECK(calib::parse_show(kCalibFactoryOut, &show));
  StreamCapture capture(std::cout);
  ui::print_calib(show);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("source", "factory")) != std::string::npos);
  CHECK(out.find(kv_text("residual", "0.42 m / 87 deg")) != std::string::npos);
  CHECK(out.find("camera xyz") != std::string::npos);
  CHECK(out.find(" 0.100   0.000   0.177 m    factory") != std::string::npos);
  CHECK(out.find("   0.0     0.0     0.0 deg  factory") != std::string::npos);
  CHECK(out.find("lidar xyz") != std::string::npos);
  CHECK(out.find(" 0.090   0.000   0.168 m    factory") != std::string::npos);
  CHECK(out.find("   0.0     0.0   180.0 deg  factory") != std::string::npos);
  CHECK(out.find("imu xyz") != std::string::npos);
  CHECK(out.find(" 0.005   0.011   0.121 m    factory") != std::string::npos);
  CHECK(out.find("  t1ctl calib corner\n") != std::string::npos);
  CHECK(out.find("reason") == std::string::npos);
}

void test_print_calib_unused() {
  calib::Show show;
  show.ok = true;
  show.have_source = true;
  show.source = units::Calibration::Unused;
  show.reason = "invalid schema";
  show.have_camera = true;
  show.camera.x = 0.1;
  show.camera.z = 0.177;
  show.camera.xyz_source = "factory";
  show.camera.rpy_source = "factory";
  StreamCapture capture(std::cout);
  ui::print_calib(show);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("source", "unused")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "invalid schema")) != std::string::npos);
  CHECK(out.find("  t1ctl calib corner\n") != std::string::npos);
}

void test_print_calib_error_container_down() {
  calib::Show show;
  show.demo = units::Demo::Inactive;
  show.stock = units::Stock::Inactive;
  show.detail = "container mentorpi-t1 is not running";
  show.have_camera = true;
  show.camera.z = 0.177;
  StreamCapture cout_capture(std::cout);
  StreamCapture cerr_capture(std::cerr);
  ui::print_calib_error(show);
  const std::string cout_out = strip_ansi(cout_capture.str());
  const std::string cerr_out = strip_ansi(cerr_capture.str());
  CHECK(cout_out.find(kv_text("demo", "inactive")) != std::string::npos);
  CHECK(cout_out.find(kv_text("stock", "inactive")) != std::string::npos);
  CHECK(cout_out.find("0.177") == std::string::npos);
  CHECK(cout_out.find("factory") == std::string::npos);
  CHECK(cout_out.find("  t1ctl start\n") != std::string::npos);
  CHECK(cerr_out.find("error: calib show failed\n") != std::string::npos);
  CHECK(cerr_out.find("  container mentorpi-t1 is not running\n") != std::string::npos);
}

void test_print_status_calibration_factory() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.have_control = true;
  status.mode = units::Mode::Follow;
  status.have_calibration = true;
  status.calibration = units::Calibration::Factory;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("calibration", "factory")) != std::string::npos);
  CHECK(out.find(kv_text("mode", "follow")) != std::string::npos);
  CHECK(out.find("reason") == std::string::npos);
  CHECK(out.find("t1ctl calib") == std::string::npos);
}

void test_print_status_calibration_unused_next() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.have_control = true;
  status.mode = units::Mode::Follow;
  status.have_calibration = true;
  status.calibration = units::Calibration::Unused;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("calibration", "unused")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "no /control/status")) == std::string::npos);
  CHECK(out.find("\n  t1ctl calib\n") != std::string::npos);
}

void test_print_status_calibration_does_not_take_mode_reason() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.have_control = true;
  status.mode = units::Mode::Forbidden;
  status.reason = "operator";
  status.have_calibration = true;
  status.calibration = units::Calibration::Unused;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("mode", "forbidden")) != std::string::npos);
  CHECK(out.find(kv_text("reason", "operator")) != std::string::npos);
  CHECK(out.find(kv_text("calibration", "unused")) != std::string::npos);
  CHECK(out.find("t1ctl mode allow") != std::string::npos);
  CHECK(out.find("t1ctl calib\n") == std::string::npos);
}

void test_print_help_calib_section() {
  StreamCapture capture(std::cout);
  ui::print_help();
  const std::string out = capture.str();
  CHECK(out.find("sensor extrinsics") != std::string::npos);
  CHECK(out.find("CALIB") != std::string::npos);
  CHECK(out.find("print live poses and residual") != std::string::npos);
  CHECK(out.find("calib camera") != std::string::npos);
  CHECK(out.find("calib side") != std::string::npos);
  CHECK(out.find("--side left|right [--timeout S]") != std::string::npos);
  CHECK(out.find("--height M --pitch DEG --roll DEG") != std::string::npos);
  CHECK(out.find("[--roll") == std::string::npos);
  CHECK(out.find("calib floor") == std::string::npos);
  CHECK(out.find("poses from the platform model") != std::string::npos);
  CHECK(out.find("saved file applied at start") != std::string::npos);
  CHECK(out.find("file present but not applied") != std::string::npos);
}

const char* last_calib_script(const MockRunner& runner) {
  for (auto it = runner.calls.rbegin(); it != runner.calls.rend(); ++it) {
    if (it->size() >= 2 && (*it)[0] == "docker" && (*it)[1] == "exec" &&
        std::strstr(it->back().c_str(), "ros2 run mentorpi_calibration") != nullptr) {
      return it->back().c_str();
    }
  }
  return "";
}

const std::vector<std::string>* last_docker_cp(const MockRunner& runner) {
  for (auto it = runner.calls.rbegin(); it != runner.calls.rend(); ++it) {
    if (it->size() >= 4 && (*it)[0] == "docker" && (*it)[1] == "cp") {
      return &(*it);
    }
  }
  return nullptr;
}

constexpr const char* kCalibDriveHoldOut =
    "stage: drive\n"
    "hint: drive forward\n"
    "hint: you drive the pad; this command does not move the robot\n"
    "travel: 0.4\n"
    "turn: 0\n";

constexpr const char* kCalibDriveProposalOut =
    "T1CTL_CALIB_OK=1\n"
    "stage: drive\n"
    "residual_before: 0.041\n"
    "residual_after: 0.009\n"
    "field: lidar_x 0.090 0.088 computed\n"
    "field: lidar_y 0.000 0.021 computed\n"
    "field: lidar_yaw 180.0 2.4 computed\n"
    "field: imu_yaw_sign ok ok drive\n";

constexpr const char* kCalibHoldOut =
    "stage: corner\n"
    "hint: stand in front of a right-angle corner\n"
    "frames: 12\n"
    "cloud_points: 1840\n"
    "scan_rays: 196\n"
    "imu_samples: 12\n";

constexpr const char* kCalibProposalOut =
    "T1CTL_CALIB_OK=1\n"
    "stage: corner\n"
    "cause: axis swap\n"
    "residual_before: 0.031\n"
    "residual_after: 0.008\n"
    "imu_residual_before: 4.2\n"
    "imu_residual_after: 0.4\n"
    "field: camera_z 0.177 0.172 computed\n"
    "field: camera_pitch 0.0 2.4 computed\n"
    "field: imu_pitch 0.0 -4.1 computed\n";

constexpr const char* kCalibSaveOut =
    "T1CTL_CALIB_OK=1\n"
    "field: camera_z 0.177 0.172 computed\n"
    "field: camera_pitch 0.0 2.4 computed\n"
    "field: lidar_yaw 180.0 2.4 corner\n"
    "field: lidar_z 0.168 0.170 measured\n"
    "field: imu_pitch 0.0 -4.1 computed\n"
    "field: imu_yaw_sign ok ok drive\n";

void test_print_calib_camera_yaw_uses_degrees() {
  calib::Result result;
  result.stage = "corner";
  result.fields.push_back({"camera_yaw", "0.0", "-3.13", "computed"});
  StreamCapture capture(std::cout);
  ui::print_calib_proposal(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find("deg") != std::string::npos);
  CHECK(out.find(" m") == std::string::npos);
}

void test_parse_calib_result_proposal() {
  calib::Result result;
  CHECK(calib::parse_result(kCalibProposalOut, &result));
  CHECK(result.ok);
  CHECK(result.stage == "corner");
  CHECK(result.cause == "axis swap");
  CHECK(result.have_residual_before);
  CHECK(std::fabs(result.residual_before - 0.031) < 1e-9);
  CHECK(result.have_residual_after);
  CHECK(std::fabs(result.residual_after - 0.008) < 1e-9);
  CHECK(result.have_imu_residual_before);
  CHECK(std::fabs(result.imu_residual_before - 4.2) < 1e-9);
  CHECK(result.have_imu_residual_after);
  CHECK(std::fabs(result.imu_residual_after - 0.4) < 1e-9);
  CHECK(result.fields.size() == 3);
  CHECK(result.fields[0].name == "camera_z");
  CHECK(result.fields[0].before == "0.177");
  CHECK(result.fields[0].after == "0.172");
  CHECK(result.fields[0].tag == "computed");
}

void test_parse_calib_result_hold() {
  calib::Result result;
  CHECK(!calib::parse_result(kCalibHoldOut, &result));
  calib::Result merged;
  std::istringstream ss(kCalibHoldOut);
  std::string line;
  while (std::getline(ss, line)) {
    calib::parse_result_line(line, &merged);
  }
  CHECK(merged.stage == "corner");
  CHECK(merged.hints.size() == 1);
  CHECK(merged.hints[0] == "stand in front of a right-angle corner");
  CHECK(merged.have_frames);
  CHECK(merged.frames == 12);
  CHECK(merged.have_cloud_points);
  CHECK(merged.cloud_points == 1840);
  CHECK(merged.have_scan_rays);
  CHECK(merged.scan_rays == 196);
  CHECK(merged.have_imu_samples);
  CHECK(merged.imu_samples == 12);
}

void test_parse_calib_result_save() {
  calib::Result result;
  CHECK(calib::parse_result(kCalibSaveOut, &result));
  CHECK(result.ok);
  CHECK(result.fields.size() == 6);
  CHECK(result.fields[3].name == "lidar_z");
  CHECK(result.fields[3].tag == "measured");
  CHECK(result.fields[5].name == "imu_yaw_sign");
  CHECK(result.fields[5].after == "ok");
  CHECK(result.fields[5].tag == "drive");
}

void test_calib_run_corner_script() {
  MockRunner runner;
  runner.calib_outs = {kCalibProposalOut};
  calib::Result result;
  CHECK(calib::run_command(runner, {"corner"}, &result));
  CHECK(result.ok);
  CHECK(result.stage == "corner");
  CHECK(runner.calib_used() == 1);
  const std::string script = last_calib_script(runner);
  CHECK(script.find("calib corner") != std::string::npos);
  CHECK(script.find("PYTHONUNBUFFERED=1") != std::string::npos);
  CHECK(script.find("ROS_LOCALHOST_ONLY=0") != std::string::npos);
  CHECK(runner.docker_cp_used() == 0);
}

constexpr const char* kCalibLidarProposalOut =
    "T1CTL_CALIB_OK=1\n"
    "stage: lidar\n"
    "field: lidar_z 0.168 0.170 measured\n"
    "field: lidar_pitch 0.0 0.0 measured\n"
    "field: lidar_roll 0.0 0.0 measured\n";

void test_calib_run_lidar_script() {
  MockRunner runner;
  runner.calib_outs = {kCalibLidarProposalOut};
  calib::Result result;
  CHECK(calib::run_command(runner, {"lidar", "--height", "0.17", "--pitch", "0", "--roll", "0"},
                           &result));
  CHECK(result.ok);
  CHECK(result.stage == "lidar");
  CHECK(result.fields.size() == 3);
  CHECK(result.fields[0].name == "lidar_z");
  CHECK(result.fields[0].tag == "measured");
  const std::string script = last_calib_script(runner);
  CHECK(script.find("calib lidar") != std::string::npos);
  CHECK(script.find("--height") != std::string::npos);
  CHECK(script.find("--pitch") != std::string::npos);
  CHECK(script.find("--roll") != std::string::npos);
}

void test_calib_run_camera_script() {
  MockRunner runner;
  runner.calib_outs = {
      "T1CTL_CALIB_OK=1\n"
      "stage: camera\n"
      "field: camera_z 0.177 0.180 measured\n"};
  calib::Result result;
  CHECK(calib::run_command(runner, {"camera", "--height", "0.18"}, &result));
  CHECK(result.ok);
  CHECK(result.stage == "camera");
  CHECK(result.fields.size() == 1);
  CHECK(result.fields[0].name == "camera_z");
  CHECK(result.fields[0].tag == "measured");
  const std::string script = last_calib_script(runner);
  CHECK(script.find("calib camera") != std::string::npos);
  CHECK(script.find("--height") != std::string::npos);
}

void test_calib_run_container_down() {
  MockRunner runner;
  runner.container_up = false;
  runner.demo_active = false;
  runner.calib_outs = {kCalibProposalOut};
  calib::Result result;
  CHECK(!calib::run_command(runner, {"corner"}, &result));
  CHECK(runner.calib_used() == 0);
  CHECK(result.detail.find("container mentorpi-t1 is not running") != std::string::npos);
}

void test_print_calib_hold_frame() {
  calib::Result result;
  std::istringstream ss(kCalibHoldOut);
  std::string line;
  while (std::getline(ss, line)) {
    calib::parse_result_line(line, &result);
  }
  StreamCapture capture(std::cout);
  ui::print_calib_hold(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("stage", "corner")) != std::string::npos);
  CHECK(out.find("  stand in front of a right-angle corner") != std::string::npos);
  CHECK(out.find(kv_text("frames", "12")) != std::string::npos);
  CHECK(out.find(kv_text("cloud points", "1840")) != std::string::npos);
  CHECK(out.find(kv_text("scan rays", "196")) != std::string::npos);
  CHECK(out.find(kv_text("imu samples", "12")) != std::string::npos);
}

void test_parse_calib_result_drive_hold() {
  calib::Result merged;
  std::istringstream ss(kCalibDriveHoldOut);
  std::string line;
  while (std::getline(ss, line)) {
    calib::parse_result_line(line, &merged);
  }
  CHECK(merged.stage == "drive");
  CHECK(merged.hints.size() == 2);
  CHECK(merged.hints[0] == "drive forward");
  CHECK(merged.hints[1] == "you drive the pad; this command does not move the robot");
  CHECK(merged.have_travel);
  CHECK(std::fabs(merged.travel_m - 0.4) < 1e-9);
  CHECK(merged.have_turn);
  CHECK(std::fabs(merged.turn_deg - 0.0) < 1e-9);
}

void test_print_calib_hold_drive_frame() {
  calib::Result result;
  std::istringstream ss(kCalibDriveHoldOut);
  std::string line;
  while (std::getline(ss, line)) {
    calib::parse_result_line(line, &result);
  }
  StreamCapture capture(std::cout);
  ui::print_calib_hold(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("stage", "drive")) != std::string::npos);
  CHECK(out.find("  drive forward") != std::string::npos);
  CHECK(out.find("  you drive the pad; this command does not move the robot") != std::string::npos);
  CHECK(out.find(kv_text("travel", "0.4 m")) != std::string::npos);
  CHECK(out.find(kv_text("turn", "0 deg")) != std::string::npos);
  CHECK(out.find("then turn in place") == std::string::npos);
}

void test_parse_calib_result_drive_hold_phase_switch() {
  calib::Result merged;
  const char* text =
      "stage: drive\n"
      "hint: drive forward\n"
      "hint: you drive the pad; this command does not move the robot\n"
      "travel: 0.4\n"
      "turn: 0\n"
      "hint:\n"
      "hint: now turn in place\n"
      "hint: you drive the pad; this command does not move the robot\n"
      "travel: 0.4\n"
      "turn: 12\n";
  std::istringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    calib::parse_result_line(line, &merged);
  }
  CHECK(merged.clear_hints);
  CHECK(merged.hints.size() == 2);
  CHECK(merged.hints[0] == "now turn in place");
  CHECK(merged.hints[1] == "you drive the pad; this command does not move the robot");
  CHECK(merged.have_turn);
  CHECK(std::fabs(merged.turn_deg - 12.0) < 1e-9);
}

void test_print_calib_proposal_drive_frame() {
  calib::Result result;
  CHECK(calib::parse_result(kCalibDriveProposalOut, &result));
  StreamCapture capture(std::cout);
  ui::print_calib_proposal(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("stage", "drive")) != std::string::npos);
  CHECK(out.find(kv_text("residual", "0.041 -> 0.009 m")) != std::string::npos);
  CHECK(out.find(kv_text("lidar x", "0.090 -> 0.088 m")) != std::string::npos);
  CHECK(out.find(kv_text("lidar y", "0.000 -> 0.021 m")) != std::string::npos);
  CHECK(out.find(kv_text("lidar yaw", "180.0 -> 2.4 deg")) != std::string::npos);
  CHECK(out.find(kv_text("imu yaw sign", "ok")) != std::string::npos);
  CHECK(out.find("imu yaw sign") != std::string::npos);
  CHECK(out.find("  t1ctl calib accept\n") != std::string::npos);
}

void test_calib_run_drive_script() {
  MockRunner runner;
  runner.calib_outs = {kCalibDriveProposalOut};
  calib::Result result;
  CHECK(calib::run_command(runner, {"drive"}, &result));
  CHECK(result.ok);
  CHECK(result.stage == "drive");
  CHECK(result.fields.size() == 4);
  CHECK(result.fields[3].name == "imu_yaw_sign");
  CHECK(result.fields[3].after == "ok");
  CHECK(result.fields[3].tag == "drive");
  const std::string script = last_calib_script(runner);
  CHECK(script.find("calib drive") != std::string::npos);
}

void test_print_calib_proposal_lidar_frame() {
  calib::Result result;
  CHECK(calib::parse_result(kCalibLidarProposalOut, &result));
  StreamCapture capture(std::cout);
  ui::print_calib_proposal(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("stage", "lidar")) != std::string::npos);
  CHECK(out.find(kv_text("lidar z", "0.168 -> 0.170 m")) != std::string::npos);
  CHECK(out.find(kv_text("lidar pitch", "0.0 -> 0.0 deg")) != std::string::npos);
  CHECK(out.find(kv_text("lidar roll", "0.0 -> 0.0 deg")) != std::string::npos);
  CHECK(out.find("residual") == std::string::npos);
  CHECK(out.find("  t1ctl calib accept\n") != std::string::npos);
  CHECK(out.find("  t1ctl calib reject\n") != std::string::npos);
}

void test_print_calib_proposal_frame() {
  calib::Result result;
  CHECK(calib::parse_result(kCalibProposalOut, &result));
  StreamCapture capture(std::cout);
  ui::print_calib_proposal(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("stage", "corner")) != std::string::npos);
  CHECK(out.find(kv_text("cause", "axis swap")) != std::string::npos);
  CHECK(out.find(kv_text("residual", "0.031 -> 0.008 m")) != std::string::npos);
  CHECK(out.find(kv_text("imu residual", "4.2 -> 0.4 deg")) != std::string::npos);
  CHECK(out.find(kv_text("camera z", "0.177 -> 0.172 m")) != std::string::npos);
  CHECK(out.find(kv_text("camera pitch", "0.0 -> 2.4 deg")) != std::string::npos);
  CHECK(out.find(kv_text("imu pitch", "0.0 -> -4.1 deg")) != std::string::npos);
  CHECK(out.find("  t1ctl calib accept\n") != std::string::npos);
  CHECK(out.find("  t1ctl calib reject\n") != std::string::npos);
}

void test_print_calib_save_frame() {
  calib::Result result;
  CHECK(calib::parse_result(kCalibSaveOut, &result));
  StreamCapture capture(std::cout);
  ui::print_calib_save(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("camera z", "0.177 -> 0.172 m")) != std::string::npos);
  CHECK(out.find("computed") != std::string::npos);
  CHECK(out.find(kv_text("lidar z", "0.168 -> 0.170 m")) != std::string::npos);
  CHECK(out.find("measured") != std::string::npos);
  CHECK(out.find("Calibration saved.\n") != std::string::npos);
  CHECK(out.find(kv_text("live", "after restart")) != std::string::npos);
  CHECK(out.find("  t1ctl restart\n") != std::string::npos);
}

void test_print_calib_stage_error_corner() {
  calib::Result result;
  result.demo = units::Demo::Inactive;
  result.stock = units::Stock::Inactive;
  result.stage = "corner";
  result.detail = "container mentorpi-t1 is not running\nno camera depth";
  StreamCapture cout_capture(std::cout);
  StreamCapture cerr_capture(std::cerr);
  ui::print_calib_stage_error(result);
  const std::string cout_out = strip_ansi(cout_capture.str());
  const std::string cerr_out = strip_ansi(cerr_capture.str());
  CHECK(cout_out.find(kv_text("demo", "inactive")) != std::string::npos);
  CHECK(cout_out.find(kv_text("stock", "inactive")) != std::string::npos);
  CHECK(cout_out.find("  t1ctl start\n") != std::string::npos);
  CHECK(cerr_out.find("error: calib corner failed\n") != std::string::npos);
  CHECK(cerr_out.find("  container mentorpi-t1 is not running\n") != std::string::npos);
  CHECK(cerr_out.find("  no camera depth\n") != std::string::npos);
  CHECK(cerr_out.find("calib show failed") == std::string::npos);
}

void test_print_calib_stage_phrases() {
  calib::Result result;
  result.stage = "corner";
  StreamCapture kept_capture(std::cout);
  ui::print_calib_stage_phrase(result, true);
  CHECK(kept_capture.str() == "Corner kept.\n");
  StreamCapture dropped_capture(std::cout);
  ui::print_calib_stage_phrase(result, false);
  CHECK(dropped_capture.str() == "Corner dropped.\n");
}

constexpr const char* kCalibSideContainerFrame =
    "/home/ubuntu/mentorpi_t1_ws/config/platform/t1/side_frame.png";

constexpr const char* kCalibSideOut =
    "hint: place an object clearly to one side\n"
    "frames: 8\n"
    "T1CTL_CALIB_OK=1\n"
    "stage: side\n"
    "observed: operator left\n"
    "observed: scan left 24.6\n"
    "observed: cloud right -25.1\n"
    "layer: cloud_geometry\n"
    "frame: /home/ubuntu/mentorpi_t1_ws/config/platform/t1/side_frame.png\n"
    "field: camera_transverse_mirror 0 1 computed\n";

void test_calib_run_side_script() {
  MockRunner runner;
  runner.calib_outs = {kCalibSideOut};
  calib::Result result;
  CHECK(calib::run_command(runner, {"side", "--side", "left"}, &result));
  CHECK(result.ok);
  CHECK(result.stage == "side");
  const std::string script = last_calib_script(runner);
  CHECK(script.find("calib side") != std::string::npos);
  CHECK(script.find("--side left") != std::string::npos);
  CHECK(script.find("PYTHONUNBUFFERED=1") != std::string::npos);
  CHECK(script.find("ROS_LOCALHOST_ONLY=0") != std::string::npos);
}

void test_parse_calib_result_side() {
  calib::Result result;
  CHECK(calib::parse_result(kCalibSideOut, &result));
  CHECK(result.ok);
  CHECK(result.stage == "side");
  CHECK(result.observed.size() == 3);
  CHECK(result.observed[0] == "operator left");
  CHECK(result.observed[1] == "scan left 24.6");
  CHECK(result.observed[2] == "cloud right -25.1");
  CHECK(result.layer == "cloud_geometry");
  CHECK(result.frame == kCalibSideContainerFrame);
  CHECK(result.fields.size() == 1);
  CHECK(result.fields[0].name == "camera_transverse_mirror");
  CHECK(result.fields[0].before == "0");
  CHECK(result.fields[0].after == "1");
  CHECK(result.fields[0].tag == "computed");
  CHECK(result.have_frames);
  CHECK(result.frames == 8);
  CHECK(result.hints.size() == 1);
}

void test_print_calib_side_frame() {
  calib::Result result;
  CHECK(calib::parse_result(kCalibSideOut, &result));
  StreamCapture capture(std::cout);
  ui::print_calib_side(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("stage", "side")) != std::string::npos);
  CHECK(out.find(kv_text("operator", "left")) != std::string::npos);
  CHECK(out.find(kv_text("scan", "left 24.6")) != std::string::npos);
  CHECK(out.find(kv_text("cloud", "right -25.1")) != std::string::npos);
  CHECK(out.find(kv_text("layer", "cloud_geometry")) != std::string::npos);
  CHECK(out.find(kCalibSideContainerFrame) != std::string::npos);
  CHECK(out.find("camera transverse mirror") != std::string::npos);
  CHECK(out.find("0 -> 1") != std::string::npos);
  CHECK(out.find("  t1ctl calib accept\n") != std::string::npos);
  CHECK(out.find("  t1ctl calib reject\n") != std::string::npos);
}

void test_calib_run_side_copies_frame() {
  MockRunner runner;
  runner.calib_outs = {kCalibSideOut};
  calib::Result result;
  CHECK(calib::run_command(runner, {"side", "--side", "left"}, &result));
  CHECK(result.ok);
  CHECK(result.frame_copied);
  CHECK(result.host_frame == calib::kHostSideFramePath);
  CHECK(result.frame_copy_error.empty());
  CHECK(runner.docker_cp_used() == 1);
  const std::vector<std::string>* cp = last_docker_cp(runner);
  CHECK(cp != nullptr);
  CHECK((*cp)[2] == std::string("mentorpi-t1:") + kCalibSideContainerFrame);
  CHECK((*cp)[3] == calib::kHostSideFramePath);
  StreamCapture capture(std::cout);
  ui::print_calib_side(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("frame", calib::kHostSideFramePath)) != std::string::npos);
  CHECK(out.find(kCalibSideContainerFrame) == std::string::npos);
}

void test_calib_run_side_missing_frame() {
  MockRunner runner;
  runner.calib_outs = {kCalibSideOut};
  runner.docker_cp_codes = {1};
  calib::Result result;
  CHECK(calib::run_command(runner, {"side", "--side", "left"}, &result));
  CHECK(result.ok);
  CHECK(!result.frame_copied);
  CHECK(result.host_frame.empty());
  CHECK(result.frame_copy_error == "not found in container");
  CHECK(runner.docker_cp_used() == 1);
  StreamCapture capture(std::cout);
  ui::print_calib_side(result);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("frame", "not found in container")) != std::string::npos);
  CHECK(out.find(kv_text("layer", "cloud_geometry")) != std::string::npos);
  CHECK(out.find("error:") == std::string::npos);
}

std::string write_temp_rmem_file(const std::string& content) {
  static int counter = 0;
  const std::string path = "/tmp/t1ctl_rmem_test_" + std::to_string(++counter);
  std::ofstream out(path);
  out << content;
  return path;
}

void test_fill_dds_buffers_active() {
  const std::string path = write_temp_rmem_file("16777216\n");
  units::Status status;
  units::fill_dds_buffers(status, path);
  CHECK(status.dds_buffers == units::DdsBuffers::Active);
  CHECK(status.rmem_max == 16777216);
  std::remove(path.c_str());
}

void test_fill_dds_buffers_degraded() {
  const std::string path = write_temp_rmem_file("212992\n");
  units::Status status;
  units::fill_dds_buffers(status, path);
  CHECK(status.dds_buffers == units::DdsBuffers::Degraded);
  CHECK(status.rmem_max == 212992);
  std::remove(path.c_str());
}

void test_fill_dds_buffers_unknown() {
  units::Status status;
  units::fill_dds_buffers(status, "/tmp/t1ctl_rmem_nonexistent");
  CHECK(status.dds_buffers == units::DdsBuffers::Unknown);
  CHECK(status.rmem_max == 0);
}

void test_print_status_dds_buffers_degraded() {
  units::Status status;
  status.demo = units::Demo::Active;
  status.have_control = true;
  status.mode = units::Mode::Follow;
  status.dds_buffers = units::DdsBuffers::Degraded;
  status.rmem_max = 212992;
  StreamCapture capture(std::cout);
  ui::print_status(status);
  const std::string out = strip_ansi(capture.str());
  CHECK(out.find(kv_text("dds buffers", "degraded (rmem_max 212992 < 8388608, run make deploy)")) !=
        std::string::npos);
}

}  // namespace

int main() {
  test_banner_before_yaml();
  test_lidar_probe_parse();
  test_camera_probe_parse();
  test_imu_probe_parse();
  test_odometry_probe_parse();
  test_model_probe_parse();
  test_true_false_parse();
  test_forbidden_and_reason_parse();
  test_timeout_then_retry();
  test_parsed_false_is_not_a_timeout();
  test_model_active_query();
  test_imu_active_query();
  test_odometry_active_query();
  test_camera_active_query();
  test_camera_degraded_query();
  test_imu_degraded_query();
  test_odometry_degraded_query();
  test_model_degraded_query();
  test_demo_down_defaults();
  test_live_container_despite_inactive_unit();
  test_live_probe_all_active();
  test_cold_then_warm_queries();
  test_exhausted_timeout_stays_default();
  test_viewer_constants();
  test_viewer_probe_parse();
  test_viewer_status_query();
  test_viewer_probe_script_no_echo();
  test_viewer_start_without_systemctl();
  test_viewer_stop_without_systemctl();
  test_viewer_demo_down();
  test_viewer_start_failure_shows_diag();
  test_viewer_start_demo_inactive_detail();
  test_print_viewer_status_no_hints();
  test_print_status_degraded_no_hints();
  test_print_status_imu_degraded_hint();
  test_print_status_odometry_degraded_hint();
  test_print_status_active_sensors();
  test_print_started_no_kv();
  test_print_restarted_no_kv();
  test_print_stock_no_kv();
  test_print_action_error_no_kv();
  test_print_version();
  test_print_status_version_kv();
  test_fill_dds_buffers_active();
  test_fill_dds_buffers_degraded();
  test_fill_dds_buffers_unknown();
  test_print_status_dds_buffers_degraded();
  test_print_status_forbidden_operator();
  test_print_status_forbidden_topic_reason();
  test_print_status_missing_control_not_follow();
  test_print_status_demo_down_not_follow();
  test_print_help_forbidden_dictionary();
  test_forbidden_query();
  test_missing_control_query();
  test_parse_mode_result_success();
  test_parse_mode_result_failure();
  test_set_mode_forbid_allow_manual();
  test_set_mode_container_down();
  test_set_mode_service_missing();
  test_set_mode_helper_empty_output();
  test_print_mode_forbid();
  test_print_mode_allow();
  test_print_mode_manual();
  test_print_mode_error_container_down();
  test_print_help_mode_section();
  test_parse_debug_result_success();
  test_parse_debug_result_failure();
  test_run_debug_on_off_status();
  test_run_debug_container_down();
  test_run_debug_node_missing();
  test_run_debug_overlay_fail_still_starts_bridge();
  test_print_debug_on_off();
  test_print_debug_error_node_missing();
  test_print_help_debug_section();
  test_parse_detect_result_success();
  test_parse_detect_result_failure();
  test_detect_source_offline_mac_status();
  test_detect_source_container_down();
  test_detect_source_node_missing();
  test_detect_source_helper_empty_output();
  test_print_detect_offline_mac();
  test_print_detect_error_node_missing();
  test_print_help_detect_section();
  test_parse_ros_probe_calibration();
  test_query_calibration_from_probe();
  test_query_calibration_unused_from_probe();
  test_parse_calib_show_factory();
  test_parse_calib_show_unused();
  test_parse_calib_show_failure();
  test_calib_query_factory();
  test_calib_query_container_down();
  test_calib_query_helper_failed();
  test_print_calib_factory();
  test_print_calib_unused();
  test_print_calib_error_container_down();
  test_print_status_calibration_factory();
  test_print_status_calibration_unused_next();
  test_print_status_calibration_does_not_take_mode_reason();
  test_print_help_calib_section();
  test_print_calib_camera_yaw_uses_degrees();
  test_parse_calib_result_proposal();
  test_parse_calib_result_hold();
  test_parse_calib_result_save();
  test_calib_run_corner_script();
  test_calib_run_lidar_script();
  test_calib_run_camera_script();
  test_calib_run_container_down();
  test_print_calib_hold_frame();
  test_parse_calib_result_drive_hold();
  test_parse_calib_result_drive_hold_phase_switch();
  test_print_calib_hold_drive_frame();
  test_print_calib_proposal_drive_frame();
  test_calib_run_drive_script();
  test_print_calib_proposal_lidar_frame();
  test_print_calib_proposal_frame();
  test_print_calib_save_frame();
  test_print_calib_stage_error_corner();
  test_print_calib_stage_phrases();
  test_calib_run_side_script();
  test_parse_calib_result_side();
  test_print_calib_side_frame();
  test_calib_run_side_copies_frame();
  test_calib_run_side_missing_frame();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "ok\n";
  return 0;
}
