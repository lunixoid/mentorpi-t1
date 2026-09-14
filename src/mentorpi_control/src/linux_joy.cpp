#include <dirent.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mentorpi_control/js_read.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace mentorpi_control {

class LinuxJoyNode : public rclcpp::Node {
 public:
  LinuxJoyNode() : Node("linux_joy") {
    declare_parameter<std::string>("device", "");
    declare_parameter<std::string>("name_substr", "WirelessGamepad");
    declare_parameter<double>("rate_hz", 20.0);
    declare_parameter<int64_t>("retry_ms", 200);

    device_param_ = get_parameter("device").as_string();
    name_substr_ = get_parameter("name_substr").as_string();

    const double rate_hz = get_parameter("rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }
    keepalive_ms_ = static_cast<int>(std::llround(1000.0 / rate_hz));
    if (keepalive_ms_ < 1) {
      keepalive_ms_ = 1;
    }

    const int64_t retry_ms = get_parameter("retry_ms").as_int();
    if (retry_ms <= 0) {
      throw std::invalid_argument("retry_ms must be > 0");
    }
    retry_ = std::chrono::milliseconds(retry_ms);

    const auto last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pub_ = create_publisher<sensor_msgs::msg::Joy>("/joy", last_qos);

    running_ = true;
    io_thread_ = std::thread([this] { io_loop(); });

    RCLCPP_INFO(get_logger(),
                "linux_joy started (keepalive_ms=%d retry_ms=%ld device='%s' "
                "name_substr='%s')",
                keepalive_ms_, retry_ms, device_param_.c_str(), name_substr_.c_str());
  }

  ~LinuxJoyNode() override {
    running_ = false;
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
    close_fd();
  }

 private:
  static bool readable(const std::string& path) { return ::access(path.c_str(), R_OK) == 0; }

  static bool is_js_by_id_name(const std::string& name) {
    const std::string suffix = "-joystick";
    if (name.size() < suffix.size()) {
      return false;
    }
    if (name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
      return false;
    }
    return name.find("-event-joystick") == std::string::npos;
  }

  std::string scan_by_id() const {
    DIR* dir = ::opendir("/dev/input/by-id");
    if (dir == nullptr) {
      return {};
    }
    std::string fallback;
    std::string preferred;
    while (const dirent* ent = ::readdir(dir)) {
      const std::string name = ent->d_name;
      if (!is_js_by_id_name(name)) {
        continue;
      }
      const std::string path = std::string("/dev/input/by-id/") + name;
      if (!readable(path)) {
        continue;
      }
      if (fallback.empty()) {
        fallback = path;
      }
      if (!name_substr_.empty() && name.find(name_substr_) != std::string::npos) {
        preferred = path;
        break;
      }
    }
    ::closedir(dir);
    return preferred.empty() ? fallback : preferred;
  }

  std::string scan_js() const {
    for (int i = 0; i < 16; ++i) {
      const std::string path = "/dev/input/js" + std::to_string(i);
      if (readable(path)) {
        return path;
      }
    }
    return {};
  }

  std::string resolve_device() const {
    if (!device_param_.empty()) {
      return readable(device_param_) ? device_param_ : std::string{};
    }
    const std::string by_id = scan_by_id();
    if (!by_id.empty()) {
      return by_id;
    }
    return scan_js();
  }

  void close_fd() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
      open_path_.clear();
      have_state_ = false;
    }
  }

  bool open_device(const std::string& path) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
      return false;
    }
    uint8_t naxes = 0;
    uint8_t nbuttons = 0;
    char name[128] = {0};
    if (::ioctl(fd, JSIOCGAXES, &naxes) < 0) {
      naxes = 0;
    }
    if (::ioctl(fd, JSIOCGBUTTONS, &nbuttons) < 0) {
      nbuttons = 0;
    }
    (void)::ioctl(fd, JSIOCGNAME(sizeof(name)), name);

    fd_ = fd;
    open_path_ = path;
    joy_.axes.assign(naxes, 0.0f);
    joy_.buttons.assign(nbuttons, 0);
    have_state_ = true;

    RCLCPP_INFO(get_logger(), "opened %s name='%s' axes=%u buttons=%u", path.c_str(), name,
                static_cast<unsigned>(naxes), static_cast<unsigned>(nbuttons));
    return true;
  }

  void apply_event(const js_event& ev) {
    const uint8_t type = ev.type & ~JS_EVENT_INIT;
    if (type == JS_EVENT_AXIS) {
      if (ev.number >= joy_.axes.size()) {
        joy_.axes.resize(ev.number + 1, 0.0f);
      }
      float v = static_cast<float>(ev.value) / 32767.0f;
      if (v > 1.0f) {
        v = 1.0f;
      } else if (v < -1.0f) {
        v = -1.0f;
      }
      joy_.axes[ev.number] = v;
    } else if (type == JS_EVENT_BUTTON) {
      if (ev.number >= joy_.buttons.size()) {
        joy_.buttons.resize(ev.number + 1, 0);
      }
      joy_.buttons[ev.number] = ev.value ? 1 : 0;
    }
  }

  void publish_joy() {
    if (!have_state_) {
      return;
    }
    joy_.header.stamp = now();
    joy_.header.frame_id = open_path_;
    pub_->publish(joy_);
  }

  // Drain HID events. Returns false if the fd is gone.
  // EAGAIN / empty queue is a held stick, not a disconnect.
  bool drain() {
    js_event ev{};
    while (true) {
      const ssize_t n = ::read(fd_, &ev, sizeof(ev));
      const JsIoResult r = classify_js_read(n, errno, sizeof(ev));
      if (r == JsIoResult::GotEvent) {
        apply_event(ev);
        continue;
      }
      if (r == JsIoResult::NoEvent) {
        return true;
      }
      RCLCPP_WARN(get_logger(), "lost %s (%s)", open_path_.c_str(), std::strerror(errno));
      close_fd();
      return false;
    }
  }

  void io_loop() {
    auto last_retry = std::chrono::steady_clock::now() - retry_;
    while (running_ && rclcpp::ok()) {
      if (fd_ < 0) {
        const auto now_st = std::chrono::steady_clock::now();
        if (now_st - last_retry < retry_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          continue;
        }
        last_retry = now_st;
        const std::string path = resolve_device();
        if (path.empty() || !open_device(path)) {
          continue;
        }
        if (drain() && have_state_) {
          publish_joy();
        }
      }

      // drain() may have closed the fd; do not poll(-1).
      if (fd_ < 0) {
        continue;
      }

      pollfd pfd{};
      pfd.fd = fd_;
      pfd.events = POLLIN;
      const int pr = ::poll(&pfd, 1, keepalive_ms_);
      if (!running_) {
        break;
      }
      if (pr < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_WARN(get_logger(), "poll: %s", std::strerror(errno));
        close_fd();
        continue;
      }
      if (pr > 0 && js_poll_lost(pfd.revents)) {
        RCLCPP_WARN(get_logger(), "lost %s (poll hup/err)", open_path_.c_str());
        close_fd();
        continue;
      }

      bool events = false;
      if (pr > 0 && (pfd.revents & POLLIN)) {
        events = drain();
        if (fd_ < 0) {
          continue;
        }
        if (events && have_state_) {
          publish_joy();
        }
      } else if (pr == 0 && have_state_) {
        // Keepalive: no HID events while the fd is open. This is a held
        // stick (joydev is edge-triggered). 2.4G RF-off with the dongle
        // still enumerated looks the same and cannot be distinguished.
        publish_joy();
      }
    }
  }

  std::string device_param_;
  std::string name_substr_;
  std::chrono::milliseconds retry_{200};
  int keepalive_ms_{50};
  int fd_{-1};
  std::string open_path_;
  bool have_state_{false};
  sensor_msgs::msg::Joy joy_{};
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  std::atomic<bool> running_{false};
  std::thread io_thread_;
};

}  // namespace mentorpi_control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mentorpi_control::LinuxJoyNode>());
  rclcpp::shutdown();
  return 0;
}
