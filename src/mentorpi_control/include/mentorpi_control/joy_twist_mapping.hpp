#ifndef MENTORPI_CONTROL_JOY_TWIST_MAPPING_HPP_
#define MENTORPI_CONTROL_JOY_TWIST_MAPPING_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mentorpi_control {

// Planar command from stick axes. Other Twist fields stay zero.
struct Twist2d {
  double linear_x{0.0};
  double angular_z{0.0};
};

inline bool twist2d_is_zero(const Twist2d& t) { return t.linear_x == 0.0 && t.angular_z == 0.0; }

inline double clamp_unit(double x) { return std::clamp(x, -1.0, 1.0); }

// Deadzone with continuous renormalization of the remaining range.
// |x| <= deadzone → 0; |x| == 1 → ±1; no step at the deadzone edge.
// Axes are independent: call once per axis.
inline double scale_axis(double raw, double deadzone) {
  const double x = clamp_unit(raw);
  if (!(deadzone > 0.0)) {
    return x;
  }
  if (deadzone >= 1.0) {
    return 0.0;
  }
  const double ax = std::abs(x);
  if (ax <= deadzone) {
    return 0.0;
  }
  const double mag = (ax - deadzone) / (1.0 - deadzone);
  return std::copysign(mag, x);
}

inline double axis_or_zero(const std::vector<float>& axes, int64_t index) {
  if (index < 0 || static_cast<size_t>(index) >= axes.size()) {
    return 0.0;
  }
  return static_cast<double>(axes[static_cast<size_t>(index)]);
}

// Per-axis stick shaping. Numeric values come from launch/ROS params, not
// from this function. Schmitt: inactive until |x| > enter; stays active
// until |x| <= release. Anti-deadzone: first non-zero unit is min_out/max_out,
// then the remaining stick maps linearly onto [min_out, max_out].
struct AxisShapeParams {
  double enter{0.0};
  double release{0.0};
  double min_out{0.0};
  double max_out{1.0};
};

struct JoyTwistParams {
  AxisShapeParams linear;
  AxisShapeParams angular;
};

struct AxisLatch {
  bool active{false};

  void reset() noexcept { active = false; }
};

struct JoyShapeState {
  AxisLatch linear;
  AxisLatch angular;

  void reset() noexcept {
    linear.reset();
    angular.reset();
  }
};

// Returns a signed unit in [-1, 1]. 0 is exact zero (hardware halt path).
// No smoothing, low-pass, or acceleration ramp.
inline double shape_axis(double raw, const AxisShapeParams& p, AxisLatch& latch) {
  const double x = clamp_unit(raw);
  const double ax = std::abs(x);

  if (!(p.max_out > 0.0) || !(p.enter < 1.0)) {
    latch.reset();
    return 0.0;
  }

  if (!latch.active) {
    if (ax <= p.enter) {
      return 0.0;
    }
    latch.active = true;
  } else if (ax <= p.release) {
    latch.reset();
    return 0.0;
  }

  const double u = scale_axis(x, p.enter);
  double min_frac = p.min_out / p.max_out;
  if (!(min_frac > 0.0)) {
    min_frac = 0.0;
  } else if (min_frac > 1.0) {
    min_frac = 1.0;
  }

  const double mag = (!(std::abs(u) > 0.0)) ? min_frac : min_frac + (1.0 - min_frac) * std::abs(u);
  return std::copysign(mag, x);
}

// Signs match existing pad_teleop val_map(v, 1, -1, -max, max) == -max * v
// after shape_axis. Neutral / below release is a fully zero Twist2d.
inline Twist2d joy_axes_to_twist(double linear_axis, double angular_axis,
                                 const JoyTwistParams& params, JoyShapeState& state) {
  Twist2d out;
  out.linear_x = -params.linear.max_out * shape_axis(linear_axis, params.linear, state.linear);
  out.angular_z = -params.angular.max_out * shape_axis(angular_axis, params.angular, state.angular);
  return out;
}

inline Twist2d joy_axes_to_twist(const std::vector<float>& axes, int64_t linear_index,
                                 int64_t angular_index, const JoyTwistParams& params,
                                 JoyShapeState& state) {
  return joy_axes_to_twist(axis_or_zero(axes, linear_index), axis_or_zero(axes, angular_index),
                           params, state);
}

// Legacy continuous deadzone (no output floor, no hysteresis). Used by
// existing sign/release tests; production pad_teleop uses JoyTwistParams.
inline Twist2d joy_axes_to_twist(double linear_axis, double angular_axis, double max_linear,
                                 double max_angular, double deadzone) {
  JoyShapeState state;
  JoyTwistParams params;
  params.linear = {deadzone, deadzone, 0.0, max_linear};
  params.angular = {deadzone, deadzone, 0.0, max_angular};
  return joy_axes_to_twist(linear_axis, angular_axis, params, state);
}

inline Twist2d joy_axes_to_twist(const std::vector<float>& axes, int64_t linear_index,
                                 int64_t angular_index, double max_linear, double max_angular,
                                 double deadzone) {
  JoyShapeState state;
  JoyTwistParams params;
  params.linear = {deadzone, deadzone, 0.0, max_linear};
  params.angular = {deadzone, deadzone, 0.0, max_angular};
  return joy_axes_to_twist(axes, linear_index, angular_index, params, state);
}

}  // namespace mentorpi_control

#endif  // MENTORPI_CONTROL_JOY_TWIST_MAPPING_HPP_
