#ifndef MENTORPI_CONTROL_JOY_FRESHNESS_HPP_
#define MENTORPI_CONTROL_JOY_FRESHNESS_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>

namespace mentorpi_control {

// Command freshness vs remote-presence freshness.
//
// Both clocks are ROS *receipt* time (steady_clock at the /joy callback).
// Joy.header.stamp is not an input: linux_joy keepalive rewrites stamps
// without a new HID event, so a stamp is not proof of a live stick.
//
// Default command window (not an SLA): 2 * (1000 / rate_hz) ms.
// At rate_hz=20 that is 100 ms — two linux_joy keepalive periods, equal to
// platform_adapter cmd_timeout_ms. Long enough that a held stick (no HID
// events, but keepalive /joy still arriving) is not zeroed; short enough
// that a dead /joy stream cannot hold a non-zero Twist until joy_timeout_ms
// (1000, remote status only).

inline int64_t default_cmd_freshness_ms(double rate_hz) {
  if (!(rate_hz > 0.0)) {
    return 100;
  }
  const double ms = 2000.0 / rate_hz;
  const auto rounded = static_cast<int64_t>(std::llround(ms));
  return rounded > 0 ? rounded : 1;
}

inline bool is_fresh(bool have, std::chrono::steady_clock::time_point last_receipt,
                     std::chrono::steady_clock::time_point now, std::chrono::milliseconds timeout) {
  return have && (now - last_receipt <= timeout);
}

}  // namespace mentorpi_control

#endif  // MENTORPI_CONTROL_JOY_FRESHNESS_HPP_
