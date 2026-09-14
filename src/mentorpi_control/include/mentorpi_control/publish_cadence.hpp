#ifndef MENTORPI_CONTROL_PUBLISH_CADENCE_HPP_
#define MENTORPI_CONTROL_PUBLISH_CADENCE_HPP_

#include <chrono>

namespace mentorpi_control {

// Event callbacks always publish. The wall timer is keepalive only: skip if
// a send already happened within one period. Not an SLA.
inline bool timer_keepalive_due(bool ever_sent, std::chrono::steady_clock::time_point now,
                                std::chrono::steady_clock::time_point last_send,
                                std::chrono::steady_clock::duration period) {
  if (!ever_sent) {
    return true;
  }
  return now - last_send >= period;
}

}  // namespace mentorpi_control

#endif  // MENTORPI_CONTROL_PUBLISH_CADENCE_HPP_
