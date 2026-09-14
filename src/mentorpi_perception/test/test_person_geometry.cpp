// T6 AC3: no rclcpp, no sockets, no Ultralytics/MPS — header-only geometry.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

#include "mentorpi_perception/person_geometry.hpp"

using mentorpi_perception::bbox_clip_to_image;
using mentorpi_perception::bbox_from_detection;
using mentorpi_perception::BboxPx;
using mentorpi_perception::CameraIntrinsics;
using mentorpi_perception::CloudXyzView;
using mentorpi_perception::intrinsics_valid;
using mentorpi_perception::kClusterMinPoints;
using mentorpi_perception::kClusterSlabM;
using mentorpi_perception::nearest_cluster_in_bbox;
using mentorpi_perception::PersonXyRange;
using mentorpi_perception::project_point;
using mentorpi_perception::rotate_bbox_yaw;
using mentorpi_perception::xy_range_in_base;
using mentorpi_perception::Xyz;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

bool near(float a, float b) { return std::fabs(a - b) < 1.0e-5f; }

// Stand as-built (SD022 ### As-built п. 8): /aurora/rgb/camera_info.
constexpr double kFx = 417.4124755859375;
constexpr double kFy = 418.379150390625;
constexpr double kCx = 337.4750061035156;
constexpr double kCy = 183.7637481689453;
constexpr int kRgbW = 640;
constexpr int kRgbH = 400;

// Aurora point_step with a colour gap after xyz, as on the stand.
constexpr int kPointStep = 32;
// Capacity of the flat buffer. Deliberately NOT 640*400 and not any w*h of the
// RGB frame: the fix must not lean on a grid interpretation (AC1, AC3).
constexpr std::size_t kCapacity = 4096;

CameraIntrinsics stand_intrinsics() {
  CameraIntrinsics cam;
  cam.fx = kFx;
  cam.fy = kFy;
  cam.cx = kCx;
  cam.cy = kCy;
  cam.width = kRgbW;
  cam.height = kRgbH;
  return cam;
}

// REP-103 camera optical (x right, y down, z forward) -> base_footprint
// (x forward, y left) for a camera mounted straight ahead. This is what tf2
// does on the robot; the unit test spells it out so "left -> y>0" is testable
// without ROS.
PersonXyRange optical_to_base(const Xyz& p) {
  return xy_range_in_base(static_cast<double>(p.z), -static_cast<double>(p.x));
}

// A compacted list of valid points with a zero tail — the real /aurora/points2
// shape (SD022 D6.1), not an organized grid.
struct FlatCloud {
  std::vector<std::uint8_t> buf;
  std::size_t used{0};
  CloudXyzView view;
};

FlatCloud make_cloud() {
  FlatCloud cloud;
  cloud.buf.assign(kCapacity * static_cast<std::size_t>(kPointStep), 0);
  cloud.view.data = cloud.buf.data();
  cloud.view.data_bytes = cloud.buf.size();
  cloud.view.point_count = kCapacity;
  cloud.view.point_step = kPointStep;
  cloud.view.offset_x = 0;
  cloud.view.offset_y = 4;
  cloud.view.offset_z = 8;
  return cloud;
}

void push_point(FlatCloud* cloud, float x, float y, float z) {
  if (cloud->used >= kCapacity) {
    std::cerr << "FAIL: cloud capacity\n";
    ++g_fails;
    return;
  }
  const std::size_t base = cloud->used * static_cast<std::size_t>(kPointStep);
  std::memcpy(cloud->buf.data() + base + 0, &x, sizeof(float));
  std::memcpy(cloud->buf.data() + base + 4, &y, sizeof(float));
  std::memcpy(cloud->buf.data() + base + 8, &z, sizeof(float));
  ++cloud->used;
}

// Plant a patch of points that projects into the given pixel window at depth z.
void push_patch(FlatCloud* cloud, const CameraIntrinsics& cam, double u_from, double u_to,
                double v_from, double v_to, float z_from, float z_to, int count) {
  for (int i = 0; i < count; ++i) {
    const double t = count == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(count - 1);
    const double u = u_from + (u_to - u_from) * t;
    const double v = v_from + (v_to - v_from) * t;
    const float z = z_from + (z_to - z_from) * static_cast<float>(t);
    const float x = static_cast<float>((u - cam.cx) * static_cast<double>(z) / cam.fx);
    const float y = static_cast<float>((v - cam.cy) * static_cast<double>(z) / cam.fy);
    push_point(cloud, x, y, z);
  }
}

BboxPx box_at(double center_u, double width) {
  BboxPx box;
  expect(bbox_from_detection(center_u, 160.0, width, 192.0, &box), "bbox built");
  return box;
}

// Projection is the measured pixel<->point correspondence (D6.2).
void test_projection_round_trip() {
  const CameraIntrinsics cam = stand_intrinsics();
  expect(intrinsics_valid(cam), "stand intrinsics usable");
  expect(!intrinsics_valid(CameraIntrinsics{}), "no camera_info => not usable");

  double u = 0.0;
  double v = 0.0;
  const Xyz centre{0.0f, 0.0f, 2.0f};
  expect(project_point(cam, centre, &u, &v), "project centre");
  expect(std::fabs(u - kCx) < 1e-6 && std::fabs(v - kCy) < 1e-6, "optical axis lands on cx,cy");

  const Xyz left{-0.5f, 0.0f, 2.0f};
  expect(project_point(cam, left, &u, &v), "project left");
  expect(u < kCx, "optical x<0 => left of centre in the frame");

  const Xyz behind{0.1f, 0.0f, -1.0f};
  expect(!project_point(cam, behind, &u, &v), "point behind the camera is not projected");
  expect(!project_point(CameraIntrinsics{}, left, &u, &v), "no intrinsics => no projection");
}

// AC1, AC2. Stand case (### As-built пп. 8–9) on a compacted buffer: person on
// the left inside the bbox, wall behind them, and the near rogue point at
// u=372 v=366 that the old slot arithmetic used to return.
void test_left_person_wins_over_wall_and_rogue() {
  const CameraIntrinsics cam = stand_intrinsics();
  FlatCloud cloud = make_cloud();

  // Rogue: nearest point in the whole cloud, but outside the bbox.
  push_patch(&cloud, cam, 372.0, 372.0, 366.0, 366.0, 0.45f, 0.45f, 1);
  // Wall behind the person, inside the bbox.
  push_patch(&cloud, cam, 120.0, 280.0, 80.0, 250.0, 3.55f, 3.65f, 200);
  // Person, inside the bbox, closer than the wall.
  push_patch(&cloud, cam, 150.0, 250.0, 100.0, 250.0, 2.00f, 2.25f, 200);

  const BboxPx box = box_at(194.0, 207.0);
  Xyz picked{};
  expect(nearest_cluster_in_bbox(cloud.view, cam, box, kClusterSlabM, kClusterMinPoints, &picked),
         "AC1 person cluster found");
  expect(picked.z > 1.9f && picked.z < 2.4f, "AC1 near cluster is the person, not the wall");
  expect(picked.x < 0.0f, "AC1 person is left of the optical axis");

  const PersonXyRange geom = optical_to_base(picked);
  expect(geom.y > 0.0f, "AC2 person on the left => base y > 0");
  expect(geom.x > 0.0f, "AC2 person ahead => base x > 0");
  expect(near(geom.range, std::hypot(geom.x, geom.y)), "range = hypot(x,y)");

  // The rogue point is nearer than the person but never enters the bbox.
  double rogue_u = 0.0;
  double rogue_v = 0.0;
  Xyz rogue{};
  expect(mentorpi_perception::read_cloud_xyz(cloud.view, 0, &rogue), "rogue readable");
  expect(project_point(cam, rogue, &rogue_u, &rogue_v), "rogue projects");
  expect(rogue_u > box.u_max, "rogue sits outside the bbox");
  expect(rogue.z < picked.z, "rogue is nearer than the person, and still not picked");
}

// AC2. Same scene mirrored: right of the frame must give base y < 0, and the
// bbox on the optical axis must give |y| ~ 0. The sign of y is never negated —
// it follows from which point is picked.
void test_right_and_ahead_signs() {
  const CameraIntrinsics cam = stand_intrinsics();

  FlatCloud right = make_cloud();
  push_patch(&right, cam, 425.0, 525.0, 100.0, 250.0, 2.00f, 2.25f, 200);
  Xyz right_pt{};
  expect(nearest_cluster_in_bbox(right.view, cam, box_at(481.0, 207.0), kClusterSlabM,
                                 kClusterMinPoints, &right_pt),
         "right cluster found");
  const PersonXyRange right_geom = optical_to_base(right_pt);
  expect(right_geom.y < 0.0f, "AC2 person on the right => base y < 0");
  expect(right_geom.x > 0.0f, "AC2 right keeps base x > 0");

  FlatCloud ahead = make_cloud();
  push_patch(&ahead, cam, 300.0, 375.0, 100.0, 250.0, 2.00f, 2.25f, 200);
  Xyz ahead_pt{};
  expect(nearest_cluster_in_bbox(ahead.view, cam, box_at(337.0, 207.0), kClusterSlabM,
                                 kClusterMinPoints, &ahead_pt),
         "ahead cluster found");
  const PersonXyRange ahead_geom = optical_to_base(ahead_pt);
  expect(ahead_geom.x > 0.0f, "AC2 ahead keeps base x > 0 (SD021 axis intact)");
  expect(std::fabs(ahead_geom.y) < 0.15f, "AC2 ahead keeps |base y| small");
}

// D6.3. A speckle in front of the person has no support and is walked past;
// the person is still the answer.
void test_speckle_in_front_is_skipped() {
  const CameraIntrinsics cam = stand_intrinsics();
  FlatCloud cloud = make_cloud();
  push_patch(&cloud, cam, 190.0, 200.0, 150.0, 160.0, 0.90f, 0.92f, 5);
  push_patch(&cloud, cam, 150.0, 250.0, 100.0, 250.0, 2.00f, 2.25f, 200);

  Xyz picked{};
  expect(nearest_cluster_in_bbox(cloud.view, cam, box_at(194.0, 207.0), kClusterSlabM,
                                 kClusterMinPoints, &picked),
         "speckle scene resolves");
  expect(picked.z > 1.9f, "D6.3 five-point speckle at 0.9 m is not the target");
  expect(optical_to_base(picked).y > 0.0f, "D6.3 person still gives base y > 0");
}

// D6.3. Nothing in the bbox, or support below the floor, means no target —
// a wrong target is worse than none.
void test_no_support_no_target() {
  const CameraIntrinsics cam = stand_intrinsics();
  FlatCloud cloud = make_cloud();
  push_patch(&cloud, cam, 500.0, 600.0, 100.0, 250.0, 2.00f, 2.25f, 200);

  Xyz picked{};
  expect(!nearest_cluster_in_bbox(cloud.view, cam, box_at(194.0, 207.0), kClusterSlabM,
                                  kClusterMinPoints, &picked),
         "nothing projects into the bbox => no target");

  FlatCloud thin = make_cloud();
  push_patch(&thin, cam, 150.0, 250.0, 100.0, 250.0, 2.00f, 2.25f, kClusterMinPoints - 1);
  expect(!nearest_cluster_in_bbox(thin.view, cam, box_at(194.0, 207.0), kClusterSlabM,
                                  kClusterMinPoints, &picked),
         "cluster below min support => no target");

  // AC5 in the header: without intrinsics there is no fallback path at all.
  FlatCloud full = make_cloud();
  push_patch(&full, cam, 150.0, 250.0, 100.0, 250.0, 2.00f, 2.25f, 200);
  expect(!nearest_cluster_in_bbox(full.view, CameraIntrinsics{}, box_at(194.0, 207.0),
                                  kClusterSlabM, kClusterMinPoints, &picked),
         "AC5 no camera_info => no target, no fallback");
}

// Holes are zeros at the origin, and NaN is not a point either.
void test_holes_and_nan_rejected() {
  const CameraIntrinsics cam = stand_intrinsics();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  FlatCloud cloud = make_cloud();
  for (int i = 0; i < 200; ++i) {
    push_point(&cloud, 0.0f, 0.0f, 0.0f);
    push_point(&cloud, nan, nan, nan);
  }
  Xyz picked{};
  expect(!nearest_cluster_in_bbox(cloud.view, cam, box_at(194.0, 207.0), kClusterSlabM,
                                  kClusterMinPoints, &picked),
         "zeros and NaN are holes, not a cluster");
}

void test_bbox_bounds() {
  BboxPx box;
  expect(bbox_from_detection(100.0, 50.0, 20.0, 10.0, &box), "bbox ok");
  expect(near(static_cast<float>(box.u_min), 90.0f), "bbox u_min");
  expect(near(static_cast<float>(box.u_max), 110.0f), "bbox u_max");
  expect(near(static_cast<float>(box.v_min), 45.0f), "bbox v_min");
  expect(near(static_cast<float>(box.v_max), 55.0f), "bbox v_max");
  expect(!bbox_from_detection(100.0, 50.0, 0.0, 10.0, &box), "zero-width bbox rejected");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  expect(!bbox_from_detection(nan, 50.0, 20.0, 10.0, &box), "NaN bbox rejected");
}

void test_range_ignores_z() {
  const PersonXyRange geom = xy_range_in_base(1.5, -0.8);
  expect(near(geom.range, std::hypot(1.5f, 0.8f)), "range = hypot(x,y)");
  expect(!near(geom.range, std::hypot(1.5f, 0.8f, 0.25f)), "range ignores z");
  expect(near(geom.y, -0.8f), "xy_range_in_base does not negate y");
}

// SD030 T2: ego yaw rotates bbox horizontally; clip keeps overlap with the frame.
void test_rotate_bbox_yaw_zero_unchanged() {
  const CameraIntrinsics cam = stand_intrinsics();
  const BboxPx box = box_at(kCx, 100.0);
  const BboxPx rotated = rotate_bbox_yaw(box, cam, 0.0);
  expect(rotated.u_min == box.u_min && rotated.u_max == box.u_max && rotated.v_min == box.v_min &&
             rotated.v_max == box.v_max,
         "dyaw=0 leaves bbox unchanged");
  const BboxPx bad_cam = rotate_bbox_yaw(box, CameraIntrinsics{}, 0.1);
  expect(bad_cam.u_min == box.u_min && bad_cam.u_max == box.u_max,
         "invalid intrinsics => original");
}

void test_rotate_bbox_yaw_shift() {
  const CameraIntrinsics cam = stand_intrinsics();
  const BboxPx box = box_at(kCx, 100.0);
  const double center_before = (box.u_min + box.u_max) * 0.5;
  const double expected_shift = cam.fx * std::tan(0.1);

  const BboxPx right = rotate_bbox_yaw(box, cam, 0.1);
  const double center_right = (right.u_min + right.u_max) * 0.5;
  expect(std::fabs((center_right - center_before) - expected_shift) < 1.0,
         "dyaw=+0.1 shifts center right by fx*tan(0.1)");
  expect(right.v_min == box.v_min && right.v_max == box.v_max, "dyaw does not move v");

  const BboxPx left = rotate_bbox_yaw(box, cam, -0.1);
  const double center_left = (left.u_min + left.u_max) * 0.5;
  expect(std::fabs((center_left - center_before) + expected_shift) < 1.0,
         "dyaw=-0.1 shifts center left by fx*tan(0.1)");
}

void test_bbox_clip_to_image() {
  const CameraIntrinsics cam = stand_intrinsics();
  BboxPx clipped{};

  BboxPx off_left_box;
  expect(bbox_from_detection(-50.0, 160.0, 40.0, 80.0, &off_left_box), "off-left bbox built");
  expect(!bbox_clip_to_image(off_left_box, cam, &clipped), "bbox fully off-left => false");

  BboxPx off_right_box;
  expect(bbox_from_detection(700.0, 160.0, 40.0, 80.0, &off_right_box), "off-right bbox built");
  expect(!bbox_clip_to_image(off_right_box, cam, &clipped), "bbox fully off-right => false");

  BboxPx partial_box;
  expect(bbox_from_detection(10.0, 160.0, 80.0, 120.0, &partial_box), "partial bbox built");
  expect(bbox_clip_to_image(partial_box, cam, &clipped), "partial overlap => true");
  expect(clipped.u_min == 0.0, "partial clip u_min to frame edge");
  expect(clipped.u_max == partial_box.u_max, "partial clip u_max unchanged");
  expect(clipped.v_min == partial_box.v_min && clipped.v_max == partial_box.v_max,
         "partial clip v unchanged when inside frame");

  BboxPx inside_box;
  expect(bbox_from_detection(320.0, 200.0, 40.0, 40.0, &inside_box), "inside bbox built");
  expect(bbox_clip_to_image(inside_box, cam, &clipped), "inside bbox clips to itself");
  expect(clipped.u_min == inside_box.u_min && clipped.u_max == inside_box.u_max &&
             clipped.v_min == inside_box.v_min && clipped.v_max == inside_box.v_max,
         "inside bbox unchanged after clip");
}

}  // namespace

int main() {
  test_projection_round_trip();
  test_left_person_wins_over_wall_and_rogue();
  test_right_and_ahead_signs();
  test_speckle_in_front_is_skipped();
  test_no_support_no_target();
  test_holes_and_nan_rejected();
  test_bbox_bounds();
  test_range_ignores_z();
  test_rotate_bbox_yaw_zero_unchanged();
  test_rotate_bbox_yaw_shift();
  test_bbox_clip_to_image();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_person_geometry: ok\n";
  return 0;
}
