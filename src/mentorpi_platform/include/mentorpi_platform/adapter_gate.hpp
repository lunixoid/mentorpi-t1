#ifndef MENTORPI_PLATFORM_ADAPTER_GATE_HPP_
#define MENTORPI_PLATFORM_ADAPTER_GATE_HPP_

namespace mentorpi_platform {

struct Twist2d {
  double linear_x{0.0};
  double angular_z{0.0};
};

struct AdapterOutput {
  Twist2d chassis{};
  bool command_timeout{true};
  bool forbidden{false};
};

// Watchdog and Forbidden both force a zero chassis Twist. No /control/state
// means not Forbidden. Missing or stale /vehicle/cmd_vel is a timeout.
// MANUAL vs AUTO_FOLLOW source selection is control_mux, not this gate.
inline AdapterOutput gate_adapter(bool have_cmd, bool cmd_timed_out, bool have_state,
                                  bool forbidden_flag, Twist2d last) {
  AdapterOutput out;
  out.command_timeout = !have_cmd || cmd_timed_out;
  out.forbidden = have_state && forbidden_flag;
  if (!out.command_timeout && !out.forbidden) {
    out.chassis = last;
  }
  return out;
}

}  // namespace mentorpi_platform

#endif  // MENTORPI_PLATFORM_ADAPTER_GATE_HPP_
