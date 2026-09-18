#ifndef MOTION_CONTROL_TEST_SIM_WORLD_HPP_
#define MOTION_CONTROL_TEST_SIM_WORLD_HPP_

// SD036 D8. 2D closed-loop simulation without ROS: circles and segments, a ray-cast lidar in
// lidar_frame, an exact differential-drive chassis, a field-of-view perception model and the
// contour motion_control -> obstacle_guard -> gate_mux. Occlusion of people is not modelled.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "motion_control/avoidance_planner.hpp"
#include "motion_control/follow_control.hpp"
#include "motion_control/guard_decision.hpp"
#include "motion_control/obstacle_points.hpp"
#include "motion_control/virtual_bumper.hpp"

namespace sd036_sim {

using motion_control::AvoidanceMemory;
using motion_control::AvoidanceParams;
using motion_control::AvoidanceResult;
using motion_control::AvoidState;
using motion_control::FollowControlParams;
using motion_control::Footprint;
using motion_control::GuardDecision;
using motion_control::GuardInputs;
using motion_control::GuardMemory;
using motion_control::GuardParams;
using motion_control::Maneuver;
using motion_control::Point2d;
using motion_control::Pose2d;
using motion_control::Twist2d;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPersonBodyRadius = 0.15;   // what the lidar hits of a person at 0.18 m
constexpr double kMinPerceptionRange = 0.3;  // Aurora depth does not measure closer (SD025)

struct Circle {
  double x;
  double y;
  double r;
};

struct Segment {
  double x0;
  double y0;
  double x1;
  double y1;
};

struct Person {
  double x;
  double y;
  bool target;
};

struct World {
  std::vector<Circle> circles;
  std::vector<Segment> segments;
  std::vector<Person> persons;
};

// --- geometry -------------------------------------------------------------------------------

inline Point2d to_frame(const Pose2d& frame, double wx, double wy) {
  const double c = std::cos(frame.yaw);
  const double s = std::sin(frame.yaw);
  const double dx = wx - frame.x;
  const double dy = wy - frame.y;
  return Point2d{c * dx + s * dy, -s * dx + c * dy};
}

inline Pose2d compose(const Pose2d& a, const Pose2d& b) {
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return Pose2d{a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw};
}

// Distance along a unit ray to the first hit, +inf when none.
inline double ray_circle(double ox, double oy, double dx, double dy, const Circle& c) {
  const double fx = ox - c.x;
  const double fy = oy - c.y;
  const double b = fx * dx + fy * dy;
  const double cc = fx * fx + fy * fy - c.r * c.r;
  const double disc = b * b - cc;
  if (disc < 0.0) {
    return kInf;
  }
  const double sq = std::sqrt(disc);
  const double t0 = -b - sq;
  const double t1 = -b + sq;
  if (t0 > 1e-9) {
    return t0;
  }
  if (t1 > 1e-9) {
    return t1;
  }
  return kInf;
}

inline double cross(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

inline double ray_segment(double ox, double oy, double dx, double dy, const Segment& s) {
  const double ex = s.x1 - s.x0;
  const double ey = s.y1 - s.y0;
  const double denom = cross(dx, dy, ex, ey);
  if (std::abs(denom) < 1e-12) {
    return kInf;
  }
  const double wx = s.x0 - ox;
  const double wy = s.y0 - oy;
  const double t = cross(wx, wy, ex, ey) / denom;
  const double u = cross(wx, wy, dx, dy) / denom;
  if (t > 1e-9 && u >= 0.0 && u <= 1.0) {
    return t;
  }
  return kInf;
}

inline double point_segment_distance(double px, double py, const Segment& s) {
  const double ex = s.x1 - s.x0;
  const double ey = s.y1 - s.y0;
  const double len2 = ex * ex + ey * ey;
  double u = len2 > 0.0 ? ((px - s.x0) * ex + (py - s.y0) * ey) / len2 : 0.0;
  u = std::clamp(u, 0.0, 1.0);
  return std::hypot(px - (s.x0 + u * ex), py - (s.y0 + u * ey));
}

inline bool segments_intersect(const Segment& a, const Segment& b) {
  const double d1 = cross(b.x1 - b.x0, b.y1 - b.y0, a.x0 - b.x0, a.y0 - b.y0);
  const double d2 = cross(b.x1 - b.x0, b.y1 - b.y0, a.x1 - b.x0, a.y1 - b.y0);
  const double d3 = cross(a.x1 - a.x0, a.y1 - a.y0, b.x0 - a.x0, b.y0 - a.y0);
  const double d4 = cross(a.x1 - a.x0, a.y1 - a.y0, b.x1 - a.x0, b.y1 - a.y0);
  return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0));
}

inline double segment_segment_distance(const Segment& a, const Segment& b) {
  if (segments_intersect(a, b)) {
    return 0.0;
  }
  return std::min({point_segment_distance(a.x0, a.y0, b), point_segment_distance(a.x1, a.y1, b),
                   point_segment_distance(b.x0, b.y0, a), point_segment_distance(b.x1, b.y1, a)});
}

// Outline of the robot to the world geometry (people included); <= 0 means contact.
inline double footprint_world_clearance(const Footprint& fp, const Pose2d& robot, const World& w) {
  double best = kInf;
  for (const Circle& c : w.circles) {
    best = std::min(best, motion_control::footprint_clearance(fp, robot, {c.x, c.y}) - c.r);
  }
  for (const Person& p : w.persons) {
    best = std::min(best,
                    motion_control::footprint_clearance(fp, robot, {p.x, p.y}) - kPersonBodyRadius);
  }
  const Pose2d corners_local[4] = {{fp.front, fp.half_width, 0.0},
                                   {fp.front, -fp.half_width, 0.0},
                                   {-fp.back, -fp.half_width, 0.0},
                                   {-fp.back, fp.half_width, 0.0}};
  Segment edges[4];
  for (int i = 0; i < 4; ++i) {
    const Pose2d a = compose(robot, corners_local[i]);
    const Pose2d b = compose(robot, corners_local[(i + 1) % 4]);
    edges[i] = Segment{a.x, a.y, b.x, b.y};
  }
  for (const Segment& s : w.segments) {
    if (motion_control::footprint_clearance(fp, robot, {s.x0, s.y0}) == 0.0 ||
        motion_control::footprint_clearance(fp, robot, {s.x1, s.y1}) == 0.0) {
      return 0.0;
    }
    for (const Segment& e : edges) {
      best = std::min(best, segment_segment_distance(s, e));
    }
  }
  return best;
}

// --- sensors --------------------------------------------------------------------------------

struct LidarModel {
  Pose2d in_base{0.09, 0.0, M_PI};  // URDF lidar_frame
  int rays{450};
  double range_min{0.05};
  double range_max{12.0};
  float no_return{26.0f};  // LD19 past its range (SD012)
};

inline std::vector<float> raycast(const World& w, const Pose2d& lidar_world, const LidarModel& l) {
  std::vector<float> ranges(static_cast<std::size_t>(l.rays), l.no_return);
  const double increment = 2.0 * M_PI / l.rays;
  for (int i = 0; i < l.rays; ++i) {
    const double a = lidar_world.yaw - M_PI + increment * i;
    const double dx = std::cos(a);
    const double dy = std::sin(a);
    double t = kInf;
    for (const Circle& c : w.circles) {
      t = std::min(t, ray_circle(lidar_world.x, lidar_world.y, dx, dy, c));
    }
    for (const Person& p : w.persons) {
      t = std::min(
          t, ray_circle(lidar_world.x, lidar_world.y, dx, dy, Circle{p.x, p.y, kPersonBodyRadius}));
    }
    for (const Segment& s : w.segments) {
      t = std::min(t, ray_segment(lidar_world.x, lidar_world.y, dx, dy, s));
    }
    if (t <= l.range_max) {
      ranges[static_cast<std::size_t>(i)] = static_cast<float>(t);
    }
  }
  return ranges;
}

// --- contour --------------------------------------------------------------------------------

struct SimConfig {
  bool scan_enabled{true};
  int scan_every_ticks{2};  // 10 Hz scan against a 20 Hz control tick
  double self_filter_pad{0.02};
  double person_exclusion_radius{0.25};
};

struct TickTrace {
  double t;
  Pose2d pose;
  bool target_visible;
  bool scan_used;
  AvoidState state;  // meaningful when target_visible and scan_used
  Maneuver maneuver;
  Twist2d desired;
  bool stop_request;
  const char* reason;
  Twist2d executed;
  std::size_t obstacle_points;
  std::size_t excluded_points;
  double world_clearance;
};

// Mirrors the nodes of SD036: motion_control (gate, NO_SCAN fallback, planner, ramp reset on
// stop_request), obstacle_guard (decide_guard on the same points) and gate_mux of control_mux.
class Contour {
 public:
  Contour(World world, Pose2d start, FollowControlParams follow, AvoidanceParams avoid,
          GuardParams guard, SimConfig config, double dt)
      : world_(std::move(world)),
        pose_(start),
        follow_(follow),
        avoid_(avoid),
        guard_(guard),
        config_(config),
        dt_(dt) {}

  World& world() { return world_; }
  const Pose2d& pose() const { return pose_; }

  const Person* target() const {
    for (const Person& p : world_.persons) {
      if (p.target) {
        return &p;
      }
    }
    return nullptr;
  }

  double target_range() const {
    const Person* t = target();
    return t ? std::hypot(t->x - pose_.x, t->y - pose_.y) : kInf;
  }

  TickTrace step() {
    TickTrace tr{};
    tr.t = tick_ * dt_;

    // Perception: people inside the camera sector and beyond the depth minimum.
    std::vector<Point2d> persons;
    bool target_visible = false;
    Point2d target_base{0.0, 0.0};
    for (const Person& p : world_.persons) {
      const Point2d b = to_frame(pose_, p.x, p.y);
      const bool visible = std::abs(std::atan2(b.y, b.x)) <= avoid_.camera_half_fov &&
                           std::hypot(b.x, b.y) >= kMinPerceptionRange;
      if (!visible) {
        continue;
      }
      persons.push_back(b);
      if (p.target) {
        target_visible = true;
        target_base = b;
      }
    }

    // Lidar at 10 Hz; points keep the pose of the scan until the next one, as in the node.
    if (config_.scan_enabled && tick_ % config_.scan_every_ticks == 0) {
      const Pose2d lidar_world = compose(pose_, lidar_.in_base);
      const auto ranges = raycast(world_, lidar_world, lidar_);
      points_ =
          motion_control::scan_to_base_points(ranges, -M_PI, 2.0 * M_PI / lidar_.rays,
                                              lidar_.range_min, lidar_.range_max, lidar_.in_base);
      motion_control::drop_self_points(points_, avoid_.footprint, config_.self_filter_pad);
      excluded_ =
          motion_control::exclude_person_points(points_, persons, config_.person_exclusion_radius);
      have_scan_ = true;
    }
    const bool scan_used = config_.scan_enabled && have_scan_;

    // motion_control.
    Twist2d desired{0.0, 0.0};
    tr.state = AvoidState::kFree;
    tr.maneuver = Maneuver::kNone;
    if (!target_visible) {
      motion_control::reset_avoidance(memory_);
    } else {
      if (stop_request_) {
        motion_control::reset_avoidance(memory_);  // D5.4
      }
      if (scan_used) {
        const AvoidanceResult r = motion_control::plan_avoidance(
            target_base.x, target_base.y, points_, dt_, follow_, avoid_, memory_);
        desired = r.cmd;
        tr.state = r.state;
        tr.maneuver = r.maneuver;
      } else {
        desired = motion_control::compute_follow_twist(target_base.x, target_base.y,
                                                       memory_.prev_linear, dt_, follow_);
        memory_.prev_linear = desired.linear_x;
      }
    }

    // obstacle_guard.
    GuardInputs in{};
    in.have_state = true;
    in.state = motion_control::kControlAutoFollow;
    in.target_usable = target_visible;
    in.command_fresh = true;
    in.command = desired;
    in.scan_fresh = scan_used;
    in.points = scan_used ? &points_ : nullptr;
    in.now_s = tr.t;
    const GuardDecision d = motion_control::decide_guard(in, guard_, guard_memory_);
    stop_request_ = d.stop_request;

    // gate_mux: stop_request zeros the command.
    const Twist2d executed = d.stop_request ? Twist2d{0.0, 0.0} : desired;
    pose_ = motion_control::integrate_twist(pose_, executed.linear_x, executed.angular_z, dt_);
    ++tick_;

    tr.pose = pose_;
    tr.target_visible = target_visible;
    tr.scan_used = scan_used;
    tr.desired = desired;
    tr.stop_request = d.stop_request;
    tr.reason = d.reason;
    tr.executed = executed;
    tr.obstacle_points = scan_used ? points_.size() : 0;
    tr.excluded_points = scan_used ? excluded_ : 0;
    tr.world_clearance = footprint_world_clearance(avoid_.footprint, pose_, world_);
    return tr;
  }

 private:
  World world_;
  Pose2d pose_;
  FollowControlParams follow_;
  AvoidanceParams avoid_;
  GuardParams guard_;
  SimConfig config_;
  LidarModel lidar_{};
  double dt_;
  int tick_{0};
  AvoidanceMemory memory_{};
  GuardMemory guard_memory_{};
  bool stop_request_{false};
  bool have_scan_{false};
  std::vector<Point2d> points_;
  std::size_t excluded_{0};
};

}  // namespace sd036_sim

#endif  // MOTION_CONTROL_TEST_SIM_WORLD_HPP_
