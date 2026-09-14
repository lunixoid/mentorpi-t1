#ifndef MENTORPI_CONTROL_CONTROL_MUX_GATE_HPP_
#define MENTORPI_CONTROL_CONTROL_MUX_GATE_HPP_

#include <cstdint>

#include "mentorpi_control/control_mode.hpp"
#include "mentorpi_control/joy_twist_mapping.hpp"

namespace mentorpi_control {

// Select the active motion source. stop_request zeros any mode without
// changing the mode itself. FORBIDDEN and unknown states are zeros.
// Inactive-source twists must not leak into the output.
inline Twist2d gate_mux(uint8_t state, bool stop_request, const Twist2d& manual,
                        const Twist2d& follow) {
  if (stop_request) {
    return Twist2d{};
  }
  if (state == kControlManual) {
    return manual;
  }
  if (state == kControlAutoFollow) {
    return follow;
  }
  return Twist2d{};
}

}  // namespace mentorpi_control

#endif  // MENTORPI_CONTROL_CONTROL_MUX_GATE_HPP_
