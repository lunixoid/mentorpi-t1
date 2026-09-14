#include <cmath>
#include <iostream>

#include "motion_control/follow_control.hpp"

using motion_control::apply_rate_limit;
using motion_control::compute_follow_twist;
using motion_control::FollowControlParams;
using motion_control::Twist2d;

namespace {

int g_fails = 0;

// Fixture values match I4; they are not baked into compute_follow_twist.
FollowControlParams kParams() {
  FollowControlParams p{};
  p.standoff = 0.5;
  p.max_linear = 0.37;
  p.max_linear_follow = 0.20;
  p.max_angular = 2.0;
  p.kp_lin = 0.8;
  p.kp_ang = 1.5;
  p.dist_deadband = 0.05;
  p.min_breakaway_linear = 0.10;
  p.accel_linear = 0.30;
  p.decel_linear = 0.60;
  p.ang_deadband = 0.05;
  p.track_half_sum = 0.1407;
  return p;
}

constexpr double kDt = 1.0 / 20.0;  // rate_hz = 20 in I4

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

double ceiling(const FollowControlParams& p) { return std::min(p.max_linear_follow, p.max_linear); }

// Run the law until the command settles, so steady-state behaviour can be asserted.
Twist2d settle(double x, double y, const FollowControlParams& p, int ticks = 400) {
  Twist2d tw{0.0, 0.0};
  for (int i = 0; i < ticks; ++i) {
    tw = compute_follow_twist(x, y, tw.linear_x, kDt, p);
  }
  return tw;
}

// AC1. Approach grows with distance; corridor and nearer are exactly zero on both channels.
void test_profile_zones() {
  const FollowControlParams p = kParams();
  const double corridor_far = p.standoff + p.dist_deadband;

  const Twist2d near_target = settle(corridor_far + 0.10, 0.0, p);
  const Twist2d far_target = settle(corridor_far + 0.30, 0.0, p);
  expect(near_target.linear_x > 0.0, "AC1 beyond corridor → linear_x > 0");
  expect(far_target.linear_x > near_target.linear_x, "AC1 farther person → faster command");

  const Twist2d mid = settle(p.standoff, 0.0, p);
  expect(near(mid.linear_x, 0.0), "AC1 inside corridor → linear_x = 0");
  expect(near(mid.angular_z, 0.0), "AC1 inside corridor → angular_z = 0");

  const Twist2d edge = settle(corridor_far - 1e-6, 0.0, p);
  expect(near(edge.linear_x, 0.0), "AC1 last point inside the corridor → linear_x = 0");

  // D3.1, documented step: just outside the corridor the P-law emits kp_lin * dist_deadband,
  // not zero. Raising kp_lin on T5 makes this step larger — it is a known property, not a bug.
  const Twist2d beyond = settle(corridor_far + 1e-6, 0.0, p);
  expect(near(beyond.linear_x, p.kp_lin * p.dist_deadband, 1e-6),
         "D3.1 step at the corridor edge is kp_lin * dist_deadband");

  const Twist2d saturated = settle(5.0, 0.0, p);
  expect(near(saturated.linear_x, ceiling(p), 1e-9), "AC1 far away saturates at ceiling");
}

// AC2. No reverse at any distance, including range = 0 and a target behind the robot.
void test_no_reverse() {
  const FollowControlParams p = kParams();
  const double ranges[] = {0.0, 0.1, 0.30, p.standoff - p.dist_deadband - 0.01};
  for (const double r : ranges) {
    expect(near(settle(r, 0.0, p).linear_x, 0.0), "AC2 nearer than corridor → linear_x = 0");
    expect(near(settle(0.0, r, p).linear_x, 0.0), "AC2 nearer than corridor via y → linear_x = 0");
  }

  for (int i = -180; i <= 180; ++i) {
    const double bearing = static_cast<double>(i) * M_PI / 180.0;
    for (const double r : {0.0, 0.2, 0.5, 1.0, 3.0}) {
      const Twist2d tw = settle(r * std::cos(bearing), r * std::sin(bearing), p, 60);
      expect(tw.linear_x >= 0.0, "AC2 linear_x ≥ 0 for any input");
    }
  }

  const Twist2d behind = settle(-2.0, 0.0, p);
  expect(behind.linear_x >= 0.0, "AC2 target behind → no reverse");
}

// AC3. Yaw is alive while moving, including at the follow ceiling; zero when standing still.
void test_yaw_while_moving() {
  const FollowControlParams p = kParams();

  const Twist2d far_off_axis = settle(3.0, 1.0, p);
  expect(far_off_axis.linear_x > 0.0, "AC3 far off-axis target → robot moves");
  expect(far_off_axis.angular_z > 0.0, "AC3 yaw alive at the follow ceiling");

  const Twist2d right = settle(3.0, -1.0, p);
  expect(right.angular_z < 0.0, "AC3 target to the right → negative yaw");

  const Twist2d in_corridor = settle(0.0, p.standoff, p);
  expect(near(in_corridor.linear_x, 0.0), "AC3 sideways at standoff → no motion");
  expect(near(in_corridor.angular_z, 0.0), "AC3 no turning in place");

  const Twist2d nearer = settle(0.30, 0.20, p);
  expect(near(nearer.linear_x, 0.0), "AC3 nearer than corridor → no motion");
  expect(near(nearer.angular_z, 0.0), "AC3 nearer than corridor → no turning in place");

  const Twist2d dead_ahead = settle(1.5, 0.0, p);
  expect(near(dead_ahead.angular_z, 0.0), "AC3 inside ang_deadband → no yaw");
}

// AC4. Fast-track invariant and the angular ceiling hold on every tick, transients included.
void test_track_invariant() {
  const FollowControlParams p = kParams();
  const double ceil = ceiling(p);

  for (int i = -180; i <= 180; ++i) {
    const double bearing = static_cast<double>(i) * M_PI / 180.0;
    for (const double r : {0.0, 0.4, 0.55, 0.8, 2.0, 6.0}) {
      Twist2d tw{0.0, 0.0};
      for (int t = 0; t < 80; ++t) {
        tw =
            compute_follow_twist(r * std::cos(bearing), r * std::sin(bearing), tw.linear_x, kDt, p);
        expect(tw.linear_x + std::abs(tw.angular_z) * p.track_half_sum <= ceil + 1e-12,
               "AC4 fast track stays within the follow ceiling");
        expect(std::abs(tw.angular_z) <= p.max_angular + 1e-12, "AC4 |angular_z| ≤ max_angular");
      }
    }
  }

  // Transient: rolling straight at the ceiling, then the person jumps sideways.
  Twist2d tw = settle(6.0, 0.0, p);
  expect(near(tw.linear_x, ceil, 1e-9), "AC4 fixture reaches the ceiling before the transient");
  for (int t = 0; t < 40; ++t) {
    tw = compute_follow_twist(6.0, 4.0, tw.linear_x, kDt, p);
    expect(tw.linear_x + std::abs(tw.angular_z) * p.track_half_sum <= ceil + 1e-12,
           "AC4 invariant holds while the ramp catches up with a lowered cap");
  }
  expect(tw.angular_z > 0.0, "AC4 yaw recovers once the ramp has caught up");
}

// AC5. Breakaway lifts only from standstill; a rolling robot may decay below it.
void test_breakaway_from_standstill_only() {
  const FollowControlParams p = kParams();
  // Small error: kp_lin * err is below the breakaway command.
  const double x = p.standoff + p.dist_deadband + 0.02;
  expect(p.kp_lin * (x - p.standoff) < p.min_breakaway_linear, "AC5 fixture err is a small one");

  const Twist2d first = compute_follow_twist(x, 0.0, 0.0, kDt, p);
  expect(near(first.linear_x, p.min_breakaway_linear, 1e-9),
         "AC5 first command out of standstill is the breakaway, not one accel step");
  expect(first.linear_x > p.accel_linear * kDt, "AC5 the ramp does not swallow the breakaway");

  const Twist2d rolling = settle(x, 0.0, p);
  expect(rolling.linear_x < p.min_breakaway_linear, "AC5 rolling robot decays below breakaway");
  expect(near(rolling.linear_x, p.kp_lin * (x - p.standoff), 1e-9),
         "AC5 rolling robot settles on the profile value");

  FollowControlParams low_ceiling = p;
  low_ceiling.max_linear_follow = p.min_breakaway_linear * 0.5;
  const Twist2d capped = compute_follow_twist(5.0, 0.0, 0.0, 1000.0, low_ceiling);
  expect(near(capped.linear_x, low_ceiling.max_linear_follow, 1e-9),
         "AC5 ceiling below breakaway emits the ceiling, not the breakaway");
}

// AC1/AC2 of T2. Rate limit shapes acceleration and deceleration and lands exactly.
void test_rate_limit() {
  const FollowControlParams p = kParams();

  double v = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double next = apply_rate_limit(v, 1.0, kDt, p.accel_linear, p.decel_linear);
    expect(near(next - v, p.accel_linear * kDt, 1e-12), "T2 AC1 rise limited by accel * dt");
    v = next;
  }

  v = 0.5;
  for (int i = 0; i < 3; ++i) {
    const double next = apply_rate_limit(v, 0.0, kDt, p.accel_linear, p.decel_linear);
    expect(near(v - next, p.decel_linear * kDt, 1e-12), "T2 AC1 fall limited by decel * dt");
    v = next;
  }

  // Landing is exact: the residual never survives as a fraction of the step.
  v = 0.01;
  v = apply_rate_limit(v, 0.0, kDt, p.accel_linear, p.decel_linear);
  expect(v == 0.0, "T2 AC2 deceleration lands on a true zero");
  expect(apply_rate_limit(0.0, 0.004, kDt, p.accel_linear, p.decel_linear) == 0.004,
         "T2 AC2 acceleration lands exactly on target");

  // AC3. Non-positive dt does not move the command.
  expect(apply_rate_limit(0.12, 1.0, 0.0, p.accel_linear, p.decel_linear) == 0.12,
         "T2 AC3 dt = 0 leaves the command alone");
  expect(apply_rate_limit(0.12, 1.0, -kDt, p.accel_linear, p.decel_linear) == 0.12,
         "T2 AC3 dt < 0 leaves the command alone");
}

// The ramp is visible through the law: a standing robot does not jump to the profile value.
void test_ramp_through_law() {
  const FollowControlParams p = kParams();
  const Twist2d first = compute_follow_twist(6.0, 0.0, 0.0, kDt, p);
  expect(near(first.linear_x, p.min_breakaway_linear, 1e-12),
         "T2 AC4 first tick from standstill is the breakaway, not the ceiling");

  const Twist2d second = compute_follow_twist(6.0, 0.0, first.linear_x, kDt, p);
  expect(near(second.linear_x, first.linear_x + p.accel_linear * kDt, 1e-12),
         "T2 AC4 above the breakaway the ramp shapes the rise");

  int ticks = 0;
  Twist2d tw{0.0, 0.0};
  while (tw.linear_x < ceiling(p) - 1e-12 && ticks < 1000) {
    tw = compute_follow_twist(6.0, 0.0, tw.linear_x, kDt, p);
    ++ticks;
  }
  expect(ticks > 1, "T2 AC4 reaching the ceiling takes more than one tick");
  expect(near(tw.linear_x, ceiling(p), 1e-9), "T2 AC4 the ramp does reach the ceiling");
}

// AC6 is a source-level check (no stop_range / dist_deadband_near / TEMPORARY); this asserts the
// behavioural half: y enters the linear channel only through range.
void test_y_only_via_range() {
  const FollowControlParams p = kParams();
  const Twist2d sideways = settle(0.0, p.standoff, p);
  expect(near(sideways.linear_x, 0.0), "sideways at standoff → linear_x = 0");

  const Twist2d diagonal = settle(p.standoff * 0.6, p.standoff * 0.8, p);
  expect(near(diagonal.linear_x, 0.0), "diagonal at corridor range → linear_x = 0");
}

}  // namespace

int main() {
  test_profile_zones();
  test_no_reverse();
  test_yaw_while_moving();
  test_track_invariant();
  test_breakaway_from_standstill_only();
  test_rate_limit();
  test_ramp_through_law();
  test_y_only_via_range();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_follow_control: ok\n";
  return 0;
}
