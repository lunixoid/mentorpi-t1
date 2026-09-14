#ifndef MENTORPI_CONTROL_PAD_COMMAND_HPP_
#define MENTORPI_CONTROL_PAD_COMMAND_HPP_

#include <cstdint>
#include <vector>

#include "mentorpi_control/joy_twist_mapping.hpp"

namespace mentorpi_control {

struct PadCommand {
  bool publish_cmd{false};
  Twist2d twist{};
  bool remote{false};
};

// Follow mute: publish_cmd is false when not manual, so pad_teleop does not
// publish /control/manual_cmd_vel outside MANUAL. Stale command, Follow,
// and Forbidden (not manual) reset hysteresis and return a zero Twist even
// if the last mapped stick was non-zero. Remote uses a longer window.
inline PadCommand decide_pad(bool manual, bool command_fresh, bool remote_fresh,
                             const std::vector<float>& axes, int64_t linear_index,
                             int64_t angular_index, const JoyTwistParams& params,
                             JoyShapeState& state) {
  PadCommand out;
  out.publish_cmd = manual;
  out.remote = remote_fresh;
  if (!manual || !command_fresh) {
    state.reset();
    return out;
  }
  out.twist = joy_axes_to_twist(axes, linear_index, angular_index, params, state);
  return out;
}

}  // namespace mentorpi_control

#endif  // MENTORPI_CONTROL_PAD_COMMAND_HPP_
