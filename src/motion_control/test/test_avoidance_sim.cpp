#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "sd036_fixtures.hpp"
#include "sim_world.hpp"

using motion_control::AvoidState;
using motion_control::Maneuver;
using motion_control::Pose2d;
using sd036_sim::Circle;
using sd036_sim::Contour;
using sd036_sim::Person;
using sd036_sim::Segment;
using sd036_sim::SimConfig;
using sd036_sim::TickTrace;
using sd036_sim::World;
using sd036_test::kDt;

namespace {

int g_fails = 0;

struct Run {
  std::vector<TickTrace> trace;
  bool reached{false};
  double min_clearance{sd036_sim::kInf};
  int obstacle_stops{0};

  bool saw(Maneuver m) const {
    for (const TickTrace& t : trace) {
      if (t.maneuver == m && t.target_visible && t.scan_used) {
        return true;
      }
    }
    return false;
  }

  // Index of the first tick with maneuver m, -1 if none.
  int first(Maneuver m, int from = 0) const {
    for (int i = from; i < static_cast<int>(trace.size()); ++i) {
      if (trace[i].maneuver == m && trace[i].target_visible && trace[i].scan_used) {
        return i;
      }
    }
    return -1;
  }
};

const char* state_name(const TickTrace& t) {
  if (!t.target_visible) {
    return "INACTIVE";
  }
  if (!t.scan_used) {
    return "NO_SCAN";
  }
  switch (t.state) {
    case AvoidState::kFree:
      return "FREE";
    case AvoidState::kAvoiding:
      return "AVOIDING";
    case AvoidState::kBlocked:
      return "BLOCKED";
  }
  return "?";
}

const char* maneuver_name(Maneuver m) {
  switch (m) {
    case Maneuver::kNone:
      return "-";
    case Maneuver::kArc:
      return "ARC";
    case Maneuver::kRotate:
      return "ROTATE";
    case Maneuver::kReverse:
      return "REVERSE";
  }
  return "?";
}

// Trace for a failed scene: every change of state or maneuver and every 20th tick.
void dump(const char* scene, const Run& run) {
  std::cerr << "--- trace " << scene << " (t x y yaw state maneuver v w stop reason clearance)\n";
  for (std::size_t i = 0; i < run.trace.size(); ++i) {
    const TickTrace& t = run.trace[i];
    const bool change = i == 0 || t.state != run.trace[i - 1].state ||
                        t.maneuver != run.trace[i - 1].maneuver ||
                        t.stop_request != run.trace[i - 1].stop_request ||
                        t.target_visible != run.trace[i - 1].target_visible;
    if (!change && i % 20 != 0) {
      continue;
    }
    char line[200];
    std::snprintf(line, sizeof(line),
                  "%6.2f %6.3f %6.3f %6.3f %-8s %-7s %6.3f %6.3f %d %-10s %.3f\n", t.t, t.pose.x,
                  t.pose.y, t.pose.yaw, state_name(t), maneuver_name(t.maneuver),
                  t.executed.linear_x, t.executed.angular_z, t.stop_request ? 1 : 0, t.reason,
                  t.world_clearance);
    std::cerr << line;
  }
}

void expect(bool cond, const char* what, const char* scene, const Run& run) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    dump(scene, run);
    ++g_fails;
  }
}

constexpr double kReachRange = 0.7;  // standoff 0.5 + corridor 0.05 + P-law undershoot (SD025 D3.3)

Contour make(World world, Pose2d start = {0.0, 0.0, 0.0}, SimConfig config = {}) {
  return Contour(std::move(world), start, sd036_test::follow_params(), sd036_test::avoid_params(),
                 sd036_test::guard_params(), config, kDt);
}

// Runs until the target is within reach (and the robot stands) or the time is up.
Run simulate(Contour& contour, double seconds, bool stop_on_reach = true) {
  Run run;
  const int ticks = static_cast<int>(seconds / kDt);
  for (int i = 0; i < ticks; ++i) {
    const TickTrace t = contour.step();
    run.trace.push_back(t);
    run.min_clearance = std::min(run.min_clearance, t.world_clearance);
    if (t.stop_request && std::strcmp(t.reason, "obstacle") == 0) {
      ++run.obstacle_stops;
    }
    if (contour.target_range() <= kReachRange) {
      run.reached = true;
      if (stop_on_reach && t.executed.linear_x == 0.0 && t.executed.angular_z == 0.0) {
        break;
      }
    }
  }
  return run;
}

Person target_at(double x, double y) { return Person{x, y, true}; }

// D4.6, operator decision on T4 (2026-09-18): an object whose avoidance needs more heading change
// than the camera half FOV relative to the target takes the target out of frame, and the gate
// stops the robot (BA negative 2). Asserted as such: after the loss the robot stands, no contact.
void expect_limitation(const char* scene, const Run& run) {
  int lost = -1;
  for (int i = 0; i < static_cast<int>(run.trace.size()); ++i) {
    if (!run.trace[i].target_visible) {
      lost = i;
      break;
    }
  }
  expect(lost >= 0, "T4 D4.6 limitation: the target leaves the frame", scene, run);
  bool stands = lost >= 0;
  for (int i = std::max(lost, 0); stands && i < static_cast<int>(run.trace.size()); ++i) {
    const TickTrace& t = run.trace[i];
    stands = t.executed.linear_x == 0.0 && t.executed.angular_z == 0.0;
  }
  expect(stands, "T4 D4.6 limitation: from the loss on the robot stands", scene, run);
  expect(!run.reached, "T4 D4.6 limitation: the target is not reached", scene, run);
}

// AC2. Chair off the line of sight, offset doorway, wall along the way: reach the target, never
// touch, guard quiet. A chair right on the line is the D4.6 limitation.
void test_reach_scenes() {
  {
    // Chair edge crosses the line robot-target: the robot cannot pass straight, avoidance needed.
    World w;
    w.circles.push_back(Circle{1.2, 0.25, 0.2});
    w.persons.push_back(target_at(3.0, 0.0));
    Contour c = make(w);
    const Run run = simulate(c, 40.0);
    expect(run.min_clearance > 0.0, "T4 AC1 chair: no contact", "chair", run);
    expect(run.reached, "T4 AC2 chair: target reached", "chair", run);
    expect(run.obstacle_stops == 0, "T4 AC2 chair: guard never stops on obstacle", "chair", run);
    expect(run.saw(Maneuver::kArc), "T4 AC2 chair: the robot flows around on arcs", "chair", run);
  }
  {
    World w;
    w.circles.push_back(Circle{1.2, 0.0, 0.2});
    w.persons.push_back(target_at(3.0, 0.0));
    Contour c = make(w);
    const Run run = simulate(c, 20.0, false);
    expect(run.min_clearance > 0.0, "T4 AC1 chair on the line: no contact", "chair_on_line", run);
    expect(run.obstacle_stops == 0, "T4 AC2 chair on the line: guard never stops on obstacle",
           "chair_on_line", run);
    expect_limitation("chair_on_line", run);
  }
  {
    World w;
    w.segments.push_back(Segment{1.5, -2.0, 1.5, -0.1});
    w.segments.push_back(Segment{1.5, 0.5, 1.5, 2.0});
    w.persons.push_back(target_at(3.0, 0.2));
    Contour c = make(w);
    const Run run = simulate(c, 40.0);
    expect(run.min_clearance > 0.0, "T4 AC1 doorway: no contact", "doorway", run);
    expect(run.reached, "T4 AC2 doorway: target reached through a 0.6 m gap", "doorway", run);
    expect(run.obstacle_stops == 0, "T4 AC2 doorway: guard never stops on obstacle", "doorway",
           run);
  }
  {
    World w;
    w.segments.push_back(Segment{-0.5, 0.35, 4.0, 0.35});
    w.persons.push_back(target_at(3.0, 0.0));
    Contour c = make(w);
    const Run run = simulate(c, 40.0);
    expect(run.min_clearance > 0.0, "T4 AC1 wall along: no contact", "wall_along", run);
    expect(run.reached, "T4 AC2 wall along: target reached", "wall_along", run);
    expect(run.obstacle_stops == 0, "T4 AC2 wall along: guard never stops on obstacle",
           "wall_along", run);
  }
}

// AC3. Object right against the front: turn in place, then arcs; the turn needed to clear it is
// wider than the camera half FOV, so this is the D4.6 limitation too. Dead end: back off.
void test_close_and_dead_end() {
  const auto fp = sd036_test::avoid_params().footprint;
  {
    World w;
    w.segments.push_back(Segment{fp.front + 0.10, -0.25, fp.front + 0.10, 0.25});
    w.persons.push_back(target_at(2.5, -0.6));
    Contour c = make(w);
    const Run run = simulate(c, 20.0, false);
    const int rotate = run.first(Maneuver::kRotate);
    expect(run.min_clearance > 0.0, "T4 AC1 close object: no contact", "close_object", run);
    expect(rotate >= 0, "T4 AC3 close object: ROTATE in the trace", "close_object", run);
    expect(rotate >= 0 && run.first(Maneuver::kArc, rotate) > rotate,
           "T4 AC3 close object: ARC after ROTATE", "close_object", run);
    expect_limitation("close_object", run);
  }
  {
    World w;
    const double side = fp.half_width + 0.12;
    w.segments.push_back(Segment{-1.2, side, fp.front + 0.06, side});
    w.segments.push_back(Segment{-1.2, -side, fp.front + 0.06, -side});
    w.segments.push_back(Segment{fp.front + 0.06, -side, fp.front + 0.06, side});
    w.persons.push_back(target_at(2.5, 0.0));
    Contour c = make(w);
    const Run run = simulate(c, 20.0, false);
    expect(run.min_clearance > 0.0, "T4 AC1 dead end: no contact", "dead_end", run);
    expect(run.saw(Maneuver::kReverse), "T4 AC3 dead end: REVERSE in the trace", "dead_end", run);
  }
}

// AC4. Boxed in: BLOCKED, zero to the chassis, the robot does not move.
void test_boxed() {
  const auto fp = sd036_test::avoid_params().footprint;
  const double gap = 0.03;
  const double xf = fp.front + gap;
  const double xb = -(fp.back + gap);
  const double ys = fp.half_width + gap;
  World w;
  w.segments = {Segment{xf, -ys, xf, ys}, Segment{xb, -ys, xb, ys}, Segment{xb, ys, xf, ys},
                Segment{xb, -ys, xf, -ys}};
  w.persons.push_back(target_at(2.0, 0.0));
  Contour c = make(w);
  const Run run = simulate(c, 5.0, false);
  bool blocked_zero = true;
  for (std::size_t i = 1; i < run.trace.size(); ++i) {  // tick 0 has no scan-based history yet
    const TickTrace& t = run.trace[i];
    blocked_zero = blocked_zero && t.state == AvoidState::kBlocked && t.executed.linear_x == 0.0 &&
                   t.executed.angular_z == 0.0;
  }
  expect(blocked_zero, "T4 AC4 boxed in: BLOCKED and zero on every tick", "boxed", run);
  expect(c.pose().x == 0.0 && c.pose().y == 0.0 && c.pose().yaw == 0.0,
         "T4 AC4 boxed in: the robot has not moved", "boxed", run);
  expect(run.min_clearance > 0.0, "T4 AC1 boxed in: no contact", "boxed", run);
}

// AC5. A stranger on the way is not an obstacle: points excluded, no avoidance because of them.
void test_stranger() {
  World w;
  w.persons.push_back(Person{1.2, 0.0, false});
  w.persons.push_back(target_at(3.0, 0.0));
  Contour c = make(w);
  Run run;
  bool excluded = false;
  bool free_only = true;
  for (int i = 0; i < 400; ++i) {
    const TickTrace t = c.step();
    run.trace.push_back(t);
    run.min_clearance = std::min(run.min_clearance, t.world_clearance);
    if (t.world_clearance < 0.3) {
      break;  // people are the operator's business; stop before contact
    }
    excluded = excluded || t.excluded_points > 0;
    free_only = free_only && t.state == AvoidState::kFree;
  }
  expect(excluded, "T4 AC5 stranger: person points are excluded", "stranger", run);
  expect(free_only, "T4 AC5 stranger: no avoidance because of a person", "stranger", run);
}

// AC6. No target with an object ahead: no motion, guard no_target. Target lost during avoidance:
// zero on the same tick.
void test_no_target() {
  {
    World w;
    w.circles.push_back(Circle{0.6, 0.0, 0.15});
    w.persons.push_back(target_at(-2.0, 0.0));  // behind: outside the camera sector
    Contour c = make(w);
    const Run run = simulate(c, 5.0, false);
    bool still = true;
    for (const TickTrace& t : run.trace) {
      still = still && t.executed.linear_x == 0.0 && t.executed.angular_z == 0.0 &&
              t.stop_request && std::strcmp(t.reason, "no_target") == 0;
    }
    expect(still, "T4 AC6 no target: zero command and no_target on every tick", "no_target", run);
    expect(c.pose().x == 0.0 && c.pose().y == 0.0, "T4 AC6 no target: the robot has not moved",
           "no_target", run);
  }
  {
    World w;
    w.circles.push_back(Circle{1.2, 0.0, 0.2});
    w.persons.push_back(target_at(3.0, 0.0));
    Contour c = make(w);
    Run run;
    int lost_tick = -1;
    for (int i = 0; i < 800; ++i) {
      const TickTrace t = c.step();
      run.trace.push_back(t);
      if (lost_tick < 0 && t.state == AvoidState::kAvoiding && t.target_visible) {
        c.world().persons[0].x = c.pose().x - 2.0 * std::cos(c.pose().yaw);  // jumps behind
        c.world().persons[0].y = c.pose().y - 2.0 * std::sin(c.pose().yaw);
        lost_tick = i + 1;
      } else if (lost_tick >= 0) {
        break;
      }
    }
    expect(lost_tick > 0, "T4 AC6 lost target: avoidance started", "lost_target", run);
    if (lost_tick > 0 && lost_tick < static_cast<int>(run.trace.size())) {
      const TickTrace& t = run.trace[lost_tick];
      expect(!t.target_visible && t.executed.linear_x == 0.0 && t.executed.angular_z == 0.0,
             "T4 AC6 lost target: zero command on the same tick", "lost_target", run);
    }
  }
}

// AC7. No scan: the SD025 law as is, guard no_scan.
void test_no_scan() {
  World w;
  w.circles.push_back(Circle{1.2, 0.0, 0.2});
  w.persons.push_back(target_at(3.0, 0.0));
  SimConfig config;
  config.scan_enabled = false;
  Contour c = make(w, {0.0, 0.0, 0.0}, config);
  const auto f = sd036_test::follow_params();
  double prev = 0.0;
  bool law = true;
  bool reason = true;
  Run run;
  for (int i = 0; i < 40; ++i) {
    const Pose2d before = c.pose();
    const auto base = sd036_sim::to_frame(before, 3.0, 0.0);
    const auto expected = motion_control::compute_follow_twist(base.x, base.y, prev, kDt, f);
    prev = expected.linear_x;
    const TickTrace t = c.step();
    run.trace.push_back(t);
    law = law && t.executed.linear_x == expected.linear_x &&
          t.executed.angular_z == expected.angular_z;
    reason = reason && !t.stop_request && std::strcmp(t.reason, "no_scan") == 0;
  }
  expect(law, "T4 AC7 no scan: command equals the SD025 law", "no_scan", run);
  expect(reason, "T4 AC7 no scan: guard reports no_scan without restriction", "no_scan", run);
}

}  // namespace

int main() {
  test_reach_scenes();
  test_close_and_dead_end();
  test_boxed();
  test_stranger();
  test_no_target();
  test_no_scan();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_avoidance_sim: ok\n";
  return 0;
}
