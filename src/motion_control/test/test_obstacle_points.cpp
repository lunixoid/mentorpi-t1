#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "motion_control/obstacle_points.hpp"

using motion_control::drop_self_points;
using motion_control::exclude_person_points;
using motion_control::Footprint;
using motion_control::Point2d;
using motion_control::Pose2d;
using motion_control::scan_to_base_points;

namespace {

int g_fails = 0;

// As-built outline from the URDF meshes (SD036); a fixture, not baked into the functions.
constexpr Footprint kFootprint{0.175, 0.166, 0.123};
constexpr double kPad = 0.02;
// URDF lidar_frame: 0.09 m forward, turned by pi.
const Pose2d kLidarInBase{0.09, 0.0, M_PI};

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

// AC1. A ray goes through the lidar pose; broken and out-of-range ranges are dropped.
void test_scan_to_base() {
  const std::vector<float> one{1.0f};
  const auto front = scan_to_base_points(one, 0.0, 0.01, 0.05, 12.0, kLidarInBase);
  expect(front.size() == 1, "T1 AC1 one valid ray gives one point");
  if (front.size() == 1) {
    expect(near(front[0].x, -0.91) && near(front[0].y, 0.0),
           "T1 AC1 ray 1.0 m at angle 0 with pose (0.09, 0, pi) lands at (-0.91, 0)");
  }

  const auto quarter = scan_to_base_points(std::vector<float>{0.0f, 2.0f}, 0.0, M_PI / 2.0, 0.05,
                                           12.0, Pose2d{0.0, 0.0, 0.0});
  expect(quarter.size() == 1, "T1 AC1 range below range_min is dropped");
  if (quarter.size() == 1) {
    expect(near(quarter[0].x, 0.0) && near(quarter[0].y, 2.0),
           "T1 AC1 ray index advances by angle_increment");
  }

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<float> broken{nan, inf, -inf, 26.0f, 0.01f};
  expect(scan_to_base_points(broken, 0.0, 0.01, 0.05, 12.0, kLidarInBase).empty(),
         "T1 AC1 NaN, inf, 26.0 past range_max and r < range_min are all dropped");
}

// AC2. Self filter keeps the pad boundary; person exclusion counts what it drops.
void test_self_and_person_filters() {
  std::vector<Point2d> pts{
      {0.0, 0.0},                                      // centre
      {kFootprint.front + kPad - 0.001, 0.0},          // inside the pad
      {kFootprint.front + kPad + 0.001, 0.0},          // just outside
      {0.0, -(kFootprint.half_width + kPad + 0.001)},  // just outside, side
      {-(kFootprint.back + kPad - 0.001), 0.05},       // inside the pad, back
  };
  drop_self_points(pts, kFootprint, kPad);
  expect(pts.size() == 2, "T1 AC2 self filter drops inside + pad and keeps pad + 1 mm");

  std::vector<Point2d> scene{{1.0, 0.0}, {1.2, 0.1}, {1.0, 1.0}, {-1.0, 0.0}};
  const std::vector<Point2d> persons{{1.1, 0.0}};
  const std::size_t dropped = exclude_person_points(scene, persons, 0.2);
  expect(dropped == 2, "T1 AC2 two points within the person radius are counted");
  expect(scene.size() == 2, "T1 AC2 two points far from the person stay");

  std::vector<Point2d> untouched{{1.0, 0.0}, {0.5, 0.5}};
  expect(exclude_person_points(untouched, {}, 0.35) == 0 && untouched.size() == 2,
         "T1 AC2 empty person list drops nothing");
}

}  // namespace

int main() {
  test_scan_to_base();
  test_self_and_person_filters();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_obstacle_points: ok\n";
  return 0;
}
