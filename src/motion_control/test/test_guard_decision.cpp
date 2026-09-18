#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <vector>

#include "motion_control/avoidance_planner.hpp"
#include "motion_control/guard_decision.hpp"
#include "sd036_fixtures.hpp"

using motion_control::AvoidanceMemory;
using motion_control::AvoidanceParams;
using motion_control::AvoidanceResult;
using motion_control::decide_guard;
using motion_control::drop_self_points;
using motion_control::FollowControlParams;
using motion_control::GuardDecision;
using motion_control::GuardInputs;
using motion_control::GuardMemory;
using motion_control::GuardParams;
using motion_control::kControlAutoFollow;
using motion_control::kControlForbidden;
using motion_control::kControlManual;
using motion_control::plan_avoidance;
using motion_control::Point2d;
using motion_control::Twist2d;
using sd036_test::circle;
using sd036_test::kDt;
using sd036_test::segment;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

bool is(const GuardDecision& d, bool stop, const char* reason) {
  return d.stop_request == stop && std::strcmp(d.reason, reason) == 0;
}

// Everything healthy in AUTO_FOLLOW; tests break one input at a time.
GuardInputs healthy(const std::vector<Point2d>* points, Twist2d command, double now_s = 0.0) {
  GuardInputs in{};
  in.have_state = true;
  in.state = kControlAutoFollow;
  in.target_usable = true;
  in.command_fresh = true;
  in.command = command;
  in.scan_fresh = true;
  in.points = points;
  in.now_s = now_s;
  return in;
}

std::vector<Point2d> wall_ahead(const GuardParams& g, double gap) {
  std::vector<Point2d> pts;
  segment(pts, g.footprint.front + gap, -0.5, g.footprint.front + gap, 0.5);
  return pts;
}

// AC1. Outside AUTO_FOLLOW the guard never restricts, whatever else is wrong.
void test_not_auto_follow() {
  const GuardParams g = sd036_test::guard_params();
  const auto wall = wall_ahead(g, 0.05);
  for (const bool have_state : {false, true}) {
    for (const uint8_t state : {kControlForbidden, kControlManual}) {
      GuardInputs in = healthy(&wall, Twist2d{0.3, 0.0});
      in.have_state = have_state;
      in.state = have_state ? state : kControlAutoFollow;
      in.target_usable = false;
      in.command_fresh = false;
      in.scan_fresh = false;
      GuardMemory m;
      expect(is(decide_guard(in, g, m), false, ""),
             "T3 AC1 no state / MANUAL / FORBIDDEN: stop_request false, reason empty");
    }
  }
}

// AC2 and AC3. Target, command, scan — in this order.
void test_order() {
  const GuardParams g = sd036_test::guard_params();
  const auto wall = wall_ahead(g, 0.05);
  GuardMemory m;

  GuardInputs no_target = healthy(&wall, Twist2d{0.3, 0.0});
  no_target.target_usable = false;
  no_target.command_fresh = false;
  no_target.scan_fresh = false;
  expect(is(decide_guard(no_target, g, m), true, "no_target"), "T3 AC2 unusable target: no_target");

  GuardInputs no_command = healthy(&wall, Twist2d{0.3, 0.0});
  no_command.command_fresh = false;
  no_command.scan_fresh = false;
  expect(is(decide_guard(no_command, g, m), true, "no_command"),
         "T3 AC3 stale command: no_command");

  GuardInputs no_scan = healthy(&wall, Twist2d{0.3, 0.0});
  no_scan.scan_fresh = false;
  expect(is(decide_guard(no_scan, g, m), false, "no_scan"),
         "T3 AC3 stale scan: no restriction, reason no_scan");

  GuardInputs no_points = healthy(nullptr, Twist2d{0.3, 0.0});
  expect(is(decide_guard(no_points, g, m), false, "no_scan"), "T3 AC3 no point set: no_scan");
}

// AC4. Collision within the stop horizon stops; a free path and backing off do not.
void test_obstacle() {
  const GuardParams g = sd036_test::guard_params();
  const auto wall = wall_ahead(g, 0.05);
  const std::vector<Point2d> none;

  GuardMemory m1;
  expect(is(decide_guard(healthy(&wall, Twist2d{0.2, 0.0}), g, m1), true, "obstacle"),
         "T3 AC4 0.2 m/s into a wall 0.05 m ahead: obstacle");

  GuardMemory m2;
  expect(is(decide_guard(healthy(&none, Twist2d{0.2, 0.0}), g, m2), false, ""),
         "T3 AC4 the same command on a free path: no restriction");

  GuardMemory m3;
  const auto close = wall_ahead(g, 0.025);
  expect(is(decide_guard(healthy(&close, Twist2d{-0.1, 0.0}), g, m3), false, ""),
         "T3 AC4 backing off an object already inside stop_margin is allowed");

  GuardMemory m4;
  expect(is(decide_guard(healthy(&close, Twist2d{0.0, 0.0}), g, m4), false, ""),
         "T3 AC4 a zero command is never a hit");

  GuardMemory m5;
  expect(is(decide_guard(healthy(&wall, Twist2d{0.02, 0.0}), g, m5), false, ""),
         "T3 AC4 a slow approach that stays outside stop_margin over the horizon is allowed");
}

// AC5. Obstacle stop holds for hold_s; leaving AUTO_FOLLOW clears it at once.
void test_hold() {
  const GuardParams g = sd036_test::guard_params();
  const auto wall = wall_ahead(g, 0.05);
  const std::vector<Point2d> none;

  GuardMemory m;
  decide_guard(healthy(&wall, Twist2d{0.2, 0.0}, 0.0), g, m);
  expect(is(decide_guard(healthy(&none, Twist2d{0.2, 0.0}, 0.1), g, m), true, "obstacle"),
         "T3 AC5 within hold_s the obstacle stop holds on a free path");
  expect(is(decide_guard(healthy(&none, Twist2d{0.2, 0.0}, g.hold_s + 0.05), g, m), false, ""),
         "T3 AC5 after hold_s it clears");

  GuardMemory manual_m;
  decide_guard(healthy(&wall, Twist2d{0.2, 0.0}, 0.0), g, manual_m);
  GuardInputs manual = healthy(&none, Twist2d{0.2, 0.0}, 0.05);
  manual.state = kControlManual;
  expect(is(decide_guard(manual, g, manual_m), false, ""), "T3 AC5 MANUAL clears the hold at once");
  expect(is(decide_guard(healthy(&none, Twist2d{0.2, 0.0}, 0.1), g, manual_m), false, ""),
         "T3 AC5 back in AUTO_FOLLOW the old hold does not return");
}

// AC6 (D6.2). Commands the planner emits with the starting values never trip the guard on the
// same points: the T2 scenes, a run at follow speed, and random scenes over several ticks.
void test_consistency_with_planner() {
  const FollowControlParams f = sd036_test::follow_params();
  const AvoidanceParams a = sd036_test::avoid_params();
  const GuardParams g = sd036_test::guard_params();
  const auto& fp = a.footprint;

  int checked = 0;
  bool clean = true;
  const auto run = [&](double tx, double ty, const std::vector<Point2d>& pts, AvoidanceMemory& m,
                       int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
      const AvoidanceResult r = plan_avoidance(tx, ty, pts, kDt, f, a, m);
      GuardMemory gm;
      const GuardDecision d = decide_guard(healthy(&pts, r.cmd), g, gm);
      clean = clean && !(d.stop_request && std::strcmp(d.reason, "obstacle") == 0);
      ++checked;
    }
  };

  {
    AvoidanceMemory m;
    run(2.0, 0.0, circle(0.45, 0.0, 0.05), m, 1);
    AvoidanceMemory fast;
    fast.prev_linear = 0.25;
    fast.prev_cmd = {0.25, 0.0};
    run(2.0, 0.0, circle(0.45, 0.0, 0.05), fast, 1);
  }
  {
    std::vector<Point2d> wall;
    segment(wall, fp.front + 0.10, -0.6, fp.front + 0.10, 0.6);
    AvoidanceMemory m;
    run(2.0, -0.5, wall, m, 5);
  }
  {
    std::vector<Point2d> pocket;
    segment(pocket, fp.front + 0.06, -0.5, fp.front + 0.06, 0.5);
    segment(pocket, -0.8, fp.half_width + 0.12, fp.front + 0.06, fp.half_width + 0.12);
    segment(pocket, -0.8, -(fp.half_width + 0.12), fp.front + 0.06, -(fp.half_width + 0.12));
    AvoidanceMemory m;
    run(2.0, 0.0, pocket, m, 5);
  }

  std::mt19937 rng(3613);
  std::uniform_real_distribution<double> coord(-1.5, 1.5);
  std::uniform_real_distribution<double> target_x(0.2, 3.0);
  std::uniform_real_distribution<double> speed(0.0, 0.25);
  std::uniform_int_distribution<int> count(0, 60);
  for (int scene = 0; scene < 1000; ++scene) {
    std::vector<Point2d> pts;
    const int n = count(rng);
    for (int i = 0; i < n; ++i) {
      pts.push_back({coord(rng), coord(rng)});
    }
    drop_self_points(pts, fp, sd036_test::kSelfFilterPad);
    AvoidanceMemory m;
    m.prev_linear = speed(rng);
    m.prev_cmd = {m.prev_linear, 0.0};
    run(target_x(rng), coord(rng), pts, m, 5);
  }

  expect(checked > 5000, "T3 AC6 enough planner commands were checked");
  expect(clean, "T3 AC6 no planner command trips the guard with obstacle");
}

}  // namespace

int main() {
  test_not_auto_follow();
  test_order();
  test_obstacle();
  test_hold();
  test_consistency_with_planner();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_guard_decision: ok\n";
  return 0;
}
