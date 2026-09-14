#ifndef MENTORPI_CONTROL_CONTROL_MODE_HPP_
#define MENTORPI_CONTROL_CONTROL_MODE_HPP_

#include <cstdint>
#include <string>

namespace mentorpi_control {

// Wire values match mentorpi_msgs/msg/ControlState.
constexpr uint8_t kControlForbidden = 0;
constexpr uint8_t kControlManual = 1;
constexpr uint8_t kControlAutoFollow = 2;
inline constexpr const char* kOperatorHoldReason = "operator";

struct ControlModeState {
  uint8_t state{kControlAutoFollow};
  std::string reason;
};

struct SetControlModeResult {
  bool success{false};
  uint8_t active_state{kControlAutoFollow};
  std::string reason;
};

inline bool is_control_mode(uint8_t state) {
  return state == kControlForbidden || state == kControlManual || state == kControlAutoFollow;
}

// Empty outside FORBIDDEN. In FORBIDDEN the stored hold reason is published.
inline std::string control_status_reason(const ControlModeState& mode) {
  if (mode.state != kControlForbidden) {
    return std::string();
  }
  return mode.reason;
}

// Pad Select toggles only MANUAL <-> AUTO_FOLLOW. FORBIDDEN is a hold:
// the button must not release it.
inline void apply_mode_toggle(ControlModeState& mode) {
  if (mode.state == kControlForbidden) {
    return;
  }
  if (mode.state == kControlAutoFollow) {
    mode.state = kControlManual;
  } else {
    mode.state = kControlAutoFollow;
  }
}

// Host write-path. Invalid target_state is rejected without changing mode.
// FORBIDDEN is always an operator hold in this node; AUTO_FOLLOW/MANUAL
// clear the reason.
inline SetControlModeResult apply_set_control_mode(ControlModeState& mode, uint8_t target_state) {
  SetControlModeResult out;
  if (!is_control_mode(target_state)) {
    out.success = false;
    out.active_state = mode.state;
    out.reason = control_status_reason(mode);
    return out;
  }
  mode.state = target_state;
  if (target_state == kControlForbidden) {
    mode.reason = kOperatorHoldReason;
  } else {
    mode.reason.clear();
  }
  out.success = true;
  out.active_state = mode.state;
  out.reason = control_status_reason(mode);
  return out;
}

}  // namespace mentorpi_control

#endif  // MENTORPI_CONTROL_CONTROL_MODE_HPP_
