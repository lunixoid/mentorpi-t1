#include <cmath>
#include <iostream>
#include <vector>

#include "motion_control/virtual_bumper.hpp"

using motion_control::bumper_margin;
using motion_control::Footprint;
using motion_control::footprint_clearance;
using motion_control::integrate_twist;
using motion_control::min_clearance;
using motion_control::Point2d;
using motion_control::Pose2d;
using motion_control::rollout_admissible;
using motion_control::rollout_twist;

namespace {

int g_fails = 0;

// As-built outline from the URDF meshes (SD036); a fixture, not baked into the functions.
constexpr Footprint kFootprint{0.175, 0.166, 0.123};
const Pose2d kOrigin{0.0, 0.0, 0.0};

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

// AC3. Distance to the outline: edge, inside, corner; the pose is honoured.
void test_clearance() {
  expect(near(footprint_clearance(kFootprint, kOrigin, {kFootprint.front + 0.1, 0.0}), 0.1),
         "T1 AC3 point 0.1 m ahead of the front edge");
  expect(footprint_clearance(kFootprint, kOrigin, {0.05, -0.05}) == 0.0, "T1 AC3 inside is 0");
  expect(near(footprint_clearance(kFootprint, kOrigin,
                                  {kFootprint.front + 0.03, kFootprint.half_width + 0.04}),
              0.05),
         "T1 AC3 diagonal from the corner is the distance to the corner");
  expect(near(footprint_clearance(kFootprint, kOrigin, {-(kFootprint.back + 0.2), 0.0}), 0.2),
         "T1 AC3 point behind the back edge");

  const Pose2d turned{1.0, 0.0, M_PI / 2.0};
  expect(near(footprint_clearance(kFootprint, turned, {1.0, kFootprint.front + 0.1}), 0.1),
         "T1 AC3 turned pose: front edge faces +y");
  expect(std::isinf(min_clearance(kFootprint, kOrigin, {})), "T1 AC3 no points is +inf");
}

void test_integrate() {
  const Pose2d straight = integrate_twist(kOrigin, 0.2, 0.0, 1.0);
  expect(near(straight.x, 0.2) && near(straight.y, 0.0), "integrate: straight line");
  const Pose2d spin = integrate_twist(kOrigin, 0.0, 0.8, 1.0);
  expect(near(spin.x, 0.0) && near(spin.y, 0.0) && near(spin.yaw, 0.8), "integrate: in place");
  const Pose2d half = integrate_twist(kOrigin, 0.5, 0.5, 2.0 * M_PI);  // r = 1, half turn
  expect(near(half.x, 0.0, 1e-9) && near(half.y, 2.0, 1e-9), "integrate: arc of radius 1");
}

// AC4. Rollouts: forward, turn in place hitting with a corner, reverse away from a point ahead.
void test_rollout() {
  const std::vector<Point2d> ahead{{kFootprint.front + 0.3, 0.0}};
  const auto forward = rollout_twist(kFootprint, 0.2, 0.0, 1.0, 0.1, ahead);
  expect(near(forward.min_clearance, 0.1, 1e-9), "T1 AC4 forward 0.2 m over 1 s leaves 0.1");
  expect(near(forward.start_clearance, 0.3, 1e-9), "T1 AC4 start clearance is at t = 0");
  expect(near(forward.end.x, 0.2, 1e-9), "T1 AC4 the last sample lands on the horizon");

  // 0.15 m from the centre sideways: clear now (0.027), a corner sweeps over it after ~35 deg.
  const std::vector<Point2d> side{{0.0, 0.15}};
  const auto spin = rollout_twist(kFootprint, 0.0, 0.8, 1.0, 0.1, side);
  expect(spin.start_clearance > 0.0, "T1 AC4 side point is clear before turning");
  expect(spin.min_clearance == 0.0, "T1 AC4 turning in place hits it with a corner");

  const auto back = rollout_twist(kFootprint, -0.1, 0.0, 1.0, 0.1, ahead);
  expect(near(back.min_clearance, back.start_clearance, 1e-12),
         "T1 AC4 reversing does not shrink the clearance to a point ahead");
  expect(back.non_decreasing, "T1 AC4 reversing away is non-decreasing");
}

// AC5. Exit rule: 0.03 m from a point with a 0.05 m margin, backing off is admissible, pushing on
// is not.
void test_exit_rule() {
  const double margin = 0.05;
  const std::vector<Point2d> close{{kFootprint.front + 0.03, 0.0}};
  const auto away = rollout_twist(kFootprint, -0.1, 0.0, 1.0, 0.1, close);
  const auto toward = rollout_twist(kFootprint, 0.1, 0.0, 1.0, 0.1, close);
  expect(rollout_admissible(away, margin), "T1 AC5 moving away from a close point is admissible");
  expect(!rollout_admissible(toward, margin), "T1 AC5 moving toward it is not");

  const std::vector<Point2d> far{{kFootprint.front + 0.5, 0.0}};
  expect(rollout_admissible(rollout_twist(kFootprint, 0.1, 0.0, 1.0, 0.1, far), margin),
         "T1 AC5 a rollout that keeps the margin is admissible");
  expect(!rollout_admissible(rollout_twist(kFootprint, 0.0, 0.8, 1.0, 0.1, {{0.0, 0.15}}), margin),
         "T1 AC5 a turn that touches is not admissible");
  expect(rollout_admissible(rollout_twist(kFootprint, 0.2, 0.3, 1.0, 0.1, {}), margin),
         "T1 AC5 no points is always admissible");
}

// AC6. Margin grows with speed from margin_min to margin_max.
void test_margin() {
  const double lo = 0.05;
  const double hi = 0.20;
  const double ref = 0.25;
  expect(bumper_margin(0.0, lo, hi, ref) == lo, "T1 AC6 standing is margin_min");
  expect(near(bumper_margin(ref, lo, hi, ref), hi), "T1 AC6 at speed_ref is margin_max");
  expect(near(bumper_margin(2.0 * ref, lo, hi, ref), hi), "T1 AC6 above speed_ref is capped");
  expect(near(bumper_margin(-0.1, lo, hi, ref), bumper_margin(0.1, lo, hi, ref)),
         "T1 AC6 reverse speed counts by magnitude");
  double prev = bumper_margin(0.0, lo, hi, ref);
  bool monotonic = true;
  for (int i = 1; i <= 100; ++i) {
    const double m = bumper_margin(0.004 * i, lo, hi, ref);
    monotonic = monotonic && m >= prev;
    prev = m;
  }
  expect(monotonic, "T1 AC6 margin is monotonic in speed");
}

}  // namespace

int main() {
  test_clearance();
  test_integrate();
  test_rollout();
  test_exit_rule();
  test_margin();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_virtual_bumper: ok\n";
  return 0;
}
