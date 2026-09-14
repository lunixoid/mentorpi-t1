#include <cmath>
#include <iostream>
#include <set>
#include <vector>

#include "mentorpi_person_detect/byte_track.hpp"

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

mentorpi_person_detect::DetectionBox make_box(float x1, float y1, float x2, float y2, float score) {
  mentorpi_person_detect::DetectionBox box;
  box.x1 = x1;
  box.y1 = y1;
  box.x2 = x2;
  box.y2 = y2;
  box.score = score;
  box.class_id = 0;
  return box;
}

mentorpi_person_detect::ByteTrackConfig default_cfg() {
  return mentorpi_person_detect::ByteTrackConfig{};
}

int confirm_id(mentorpi_person_detect::ByteTracker& tracker,
               const mentorpi_person_detect::DetectionBox& det, double* stamp) {
  tracker.update({}, *stamp);
  *stamp += 0.1;
  expect(tracker.update({det}, *stamp).empty(), "first hit stays unconfirmed");
  *stamp += 0.1;
  const auto confirmed = tracker.update({det}, *stamp);
  expect(confirmed.size() == 1, "second hit publishes the track");
  if (confirmed.empty()) {
    return -1;
  }
  return confirmed[0].track_id;
}

float box_iou_xyxy(float ax1, float ay1, float ax2, float ay2, float bx1, float by1, float bx2,
                   float by2) {
  const float ix1 = std::max(ax1, bx1);
  const float iy1 = std::max(ay1, by1);
  const float ix2 = std::min(ax2, bx2);
  const float iy2 = std::min(ay2, by2);
  const float iw = std::max(0.0f, ix2 - ix1);
  const float ih = std::max(0.0f, iy2 - iy1);
  const float inter = iw * ih;
  const float area_a = std::max(0.0f, ax2 - ax1) * std::max(0.0f, ay2 - ay1);
  const float area_b = std::max(0.0f, bx2 - bx1) * std::max(0.0f, by2 - by1);
  const float union_area = area_a + area_b - inter;
  if (union_area <= 0.0f) {
    return 0.0f;
  }
  return inter / union_area;
}

void test_ac5_legacy() {
  mentorpi_person_detect::ByteTracker tracker(default_cfg());
  const auto det = make_box(100.0f, 100.0f, 200.0f, 300.0f, 0.9f);

  expect(tracker.update({}, 0.0).empty(), "empty warmup frame publishes no ids");

  const auto unconfirmed = tracker.update({det}, 0.5);
  expect(unconfirmed.empty(), "new track on frame_id>1 stays unconfirmed until next hit");

  const auto confirmed = tracker.update({det}, 1.0);
  expect(confirmed.size() == 1, "confirmed track emitted on second consecutive frame");
  if (!confirmed.empty()) {
    expect(confirmed[0].track_id > 0, "confirmed track has positive id");
    expect(std::fabs(confirmed[0].x1 - det.x1) < 1.0f, "bbox x1 preserved within 1 px");
    expect(std::fabs(confirmed[0].y2 - det.y2) < 1.0f, "bbox y2 preserved within 1 px");
  }

  expect(tracker.update({}, 1.5).empty(), "no detections -> no confirmed tracks");
}

void test_ac1_follow_pause() {
  // Fresh inference frames from follow_pause_20260910_154522
  // /perception/detections_2d (header.stamp, republish dropped). Extract:
  // scratch/sd028/a5.py
  using mentorpi_person_detect::DetectionBox;
  struct Frame {
    double stamp_s;
    std::vector<DetectionBox> dets;
  };
  const Frame frames[] = {
      {0.000000, {make_box(255.7500f, 283.3750f, 345.7500f, 397.6250f, 0.646484f)}},
      {0.680000, {make_box(254.4375f, 282.5000f, 345.5625f, 397.5000f, 0.670898f)}},
      {1.480000, {make_box(256.3750f, 283.0000f, 345.6250f, 397.5000f, 0.604004f)}},
      {1.980000, {make_box(255.3125f, 282.5000f, 345.1875f, 397.0000f, 0.621094f)}},
      {2.480000, {make_box(247.2500f, 282.6250f, 344.7500f, 395.3750f, 0.657715f)}},
      {3.080000, {}},
      {4.280000, {make_box(180.9375f, 288.5000f, 274.0625f, 398.0000f, 0.358643f)}},
      {4.880000, {}},
      {5.479000, {make_box(257.8750f, 281.5000f, 346.1250f, 398.0000f, 0.602539f)}},
      {6.281000, {make_box(255.3125f, 279.3750f, 345.1875f, 397.6250f, 0.613770f)}},
      {6.979000, {}},
      {7.980000, {make_box(246.6875f, 281.8750f, 299.8125f, 398.1250f, 0.553711f)}},
      {8.579000, {make_box(248.3750f, 280.1250f, 346.6250f, 395.8750f, 0.742676f)}},
      {9.180000, {make_box(248.2500f, 280.5000f, 346.2500f, 396.0000f, 0.721680f)}},
      {9.680000, {make_box(247.8750f, 280.5000f, 346.1250f, 396.0000f, 0.735840f)}},
      {10.378000, {make_box(248.4375f, 280.5000f, 346.5625f, 396.0000f, 0.730469f)}},
      {10.880000, {make_box(248.3750f, 281.0000f, 345.6250f, 396.0000f, 0.701172f)}},
      {11.480000, {make_box(247.6875f, 281.3750f, 345.3125f, 396.6250f, 0.692871f)}},
      {11.980000, {make_box(247.5000f, 280.5000f, 345.5000f, 396.5000f, 0.709961f)}},
      {12.680000, {make_box(246.5625f, 282.5000f, 343.9375f, 397.0000f, 0.548828f)}},
      {13.280000, {make_box(248.0000f, 281.5000f, 345.5000f, 396.0000f, 0.705078f)}},
      {13.879000, {make_box(255.3125f, 281.5000f, 345.6875f, 398.0000f, 0.704102f)}},
      {14.580000, {make_box(252.4375f, 280.6250f, 346.5625f, 397.3750f, 0.678223f)}},
      {15.178000, {make_box(248.6250f, 281.1250f, 344.3750f, 395.8750f, 0.535156f)}},
      {15.780000, {make_box(232.8750f, 282.8750f, 292.1250f, 399.0000f, 0.560547f)}},
      {16.380000, {make_box(247.7500f, 282.3750f, 343.7500f, 397.6250f, 0.559570f)}},
      {16.880000, {make_box(249.0625f, 280.6250f, 346.4375f, 396.3750f, 0.738770f)}},
      {17.480000, {make_box(258.5000f, 281.8750f, 346.0000f, 397.1250f, 0.595215f)}},
      {18.080000, {make_box(255.0000f, 280.8750f, 346.0000f, 398.1250f, 0.695801f)}},
      {18.880000, {make_box(249.8125f, 281.5000f, 344.1875f, 397.0000f, 0.563965f)}},
      {19.485000, {}},
      {19.979000, {make_box(247.8750f, 281.0000f, 344.1250f, 396.5000f, 0.681152f)}},
      {20.480000, {make_box(250.5000f, 280.6250f, 346.0000f, 396.3750f, 0.674805f)}},
      {21.080000, {make_box(271.5000f, 281.5000f, 346.0000f, 398.0000f, 0.551270f)}},
      {21.880000, {make_box(261.6250f, 281.6250f, 346.3750f, 397.3750f, 0.589355f)}},
      {22.479000, {make_box(253.1250f, 280.5000f, 345.8750f, 397.5000f, 0.648438f)}},
      {22.980000, {make_box(247.5625f, 281.3750f, 344.9375f, 397.6250f, 0.670410f)}},
      {23.580000, {}},
      {24.780000, {make_box(224.5000f, 287.3750f, 276.5000f, 398.6250f, 0.421631f)}},
      {25.274000, {make_box(236.6250f, 283.6250f, 286.3750f, 398.3750f, 0.581055f)}},
      {25.780000, {make_box(248.1250f, 281.5000f, 345.8750f, 396.5000f, 0.680176f)}},
      {26.380000, {make_box(256.5000f, 282.6250f, 345.5000f, 397.3750f, 0.691406f)}},
      {26.980000, {make_box(253.3125f, 283.0000f, 346.6875f, 397.0000f, 0.732910f)}},
      {27.580000, {make_box(249.8750f, 281.8750f, 346.1250f, 396.1250f, 0.682617f)}},
      {28.180000, {make_box(250.3125f, 281.3750f, 345.1875f, 396.6250f, 0.627441f)}},
      {28.679000, {make_box(249.9375f, 282.0000f, 345.5625f, 395.5000f, 0.626953f)}},
      {29.382000, {make_box(244.6875f, 282.5000f, 291.8125f, 398.0000f, 0.380371f)}},
      {29.980000, {make_box(229.6875f, 286.0000f, 277.3125f, 398.5000f, 0.484863f)}},
      {30.580000, {make_box(230.5625f, 285.5000f, 278.4375f, 398.5000f, 0.542969f)}},
      {31.180000, {make_box(237.8125f, 284.1250f, 285.1875f, 398.8750f, 0.444824f)}},
      {31.679000, {make_box(246.8750f, 282.8750f, 296.1250f, 398.1250f, 0.462891f)}},
      {32.179000, {make_box(249.8750f, 282.0000f, 344.1250f, 396.5000f, 0.553711f)}},
      {32.780000, {make_box(248.9375f, 282.0000f, 345.5625f, 396.0000f, 0.604980f)}},
      {33.378000, {make_box(275.0000f, 283.0000f, 346.5000f, 398.0000f, 0.462158f)}},
      {33.880000,
       {make_box(274.1250f, 282.0000f, 345.8750f, 398.0000f, 0.520996f),
        make_box(0.0000f, 0.0000f, 639.0000f, 399.0000f, 0.309814f)}},
      {34.480000, {make_box(248.4375f, 280.0000f, 345.5625f, 396.0000f, 0.681641f)}},
      {34.979000, {make_box(247.9375f, 280.0000f, 345.0625f, 396.5000f, 0.692383f)}},
      {35.580000, {make_box(248.4375f, 280.1250f, 346.0625f, 395.8750f, 0.724121f)}},
      {36.180000, {make_box(248.5000f, 280.1250f, 345.5000f, 395.8750f, 0.700195f)}},
      {36.878000, {make_box(248.2500f, 280.1250f, 345.7500f, 395.8750f, 0.715820f)}},
  };

  mentorpi_person_detect::ByteTracker tracker(default_cfg());
  std::set<int> published_ids;
  int published_boxes = 0;
  for (const auto& frame : frames) {
    const auto out = tracker.update(frame.dets, frame.stamp_s);
    for (const auto& box : out) {
      published_ids.insert(box.track_id);
      ++published_boxes;
    }
  }
  expect(published_boxes > 0, "follow_pause publishes at least one box");
  expect(published_ids.size() == 1, "follow_pause: all published boxes share one track_id");
  if (published_ids.size() != 1) {
    std::cerr << "follow_pause unique ids=" << published_ids.size() << " boxes=" << published_boxes
              << '\n';
  }
}

void test_ac2_velocity() {
  const float w = 100.0f;
  const float h = 200.0f;
  const float y1 = 100.0f;
  const float y2 = y1 + h;
  const float iou_90 = box_iou_xyxy(0.0f, y1, w, y2, 90.0f, y1, 90.0f + w, y2);
  expect(std::fabs(iou_90 - 0.05f) < 0.01f, "90 px shift of 100x200 has IoU ~0.05");

  mentorpi_person_detect::ByteTracker tracker(default_cfg());
  auto box_at = [&](float x1) { return make_box(x1, y1, x1 + w, y2, 0.9f); };

  double stamp = 0.0;
  const int id = confirm_id(tracker, box_at(0.0f), &stamp);
  expect(id > 0, "velocity fixture confirmed");

  float x = 0.0f;
  std::set<int> ids;
  ids.insert(id);
  for (int step = 10; step <= 90; step += 10) {
    x += static_cast<float>(step);
    stamp += 0.5;
    const auto out = tracker.update({box_at(x)}, stamp);
    expect(out.size() == 1, "velocity ramp keeps a published track");
    if (!out.empty()) {
      ids.insert(out[0].track_id);
    }
  }
  for (int i = 0; i < 5; ++i) {
    x += 90.0f;
    stamp += 0.5;
    const auto out = tracker.update({box_at(x)}, stamp);
    expect(out.size() == 1, "velocity coast at 90 px keeps a published track");
    if (!out.empty()) {
      ids.insert(out[0].track_id);
    }
  }
  expect(ids.size() == 1, "AC2: one id across 10→90 px/update then 90 px coast");
}

void expect_return_id(bool same, double gap_s, bool many_updates) {
  mentorpi_person_detect::ByteTracker tracker(default_cfg());
  const auto det = make_box(100.0f, 100.0f, 200.0f, 300.0f, 0.9f);
  double stamp = 0.0;
  const int id = confirm_id(tracker, det, &stamp);
  expect(id > 0, "timeout fixture confirmed");
  const double last_seen = stamp;

  stamp += 0.1;
  expect(tracker.update({}, stamp).empty(), "first miss after confirm publishes nothing");

  if (many_updates) {
    while (stamp + 0.1 < last_seen + gap_s) {
      stamp += 0.1;
      tracker.update({}, stamp);
    }
  }
  stamp = last_seen + gap_s;
  const auto first = tracker.update({det}, stamp);
  stamp += 0.1;
  const auto second = tracker.update({det}, stamp);

  if (same) {
    expect(!first.empty() && first[0].track_id == id, "return within buffer keeps id on first hit");
    if (!first.empty() && first[0].track_id != id) {
      std::cerr << "same-gap " << gap_s << " many=" << many_updates << " got " << first[0].track_id
                << " want " << id << '\n';
    }
  } else {
    expect(first.empty(), "expired lost track is not re-activated on first new hit");
    expect(!second.empty() && second[0].track_id != id, "return after buffer starts a new id");
    if (!second.empty() && second[0].track_id == id) {
      std::cerr << "new-gap " << gap_s << " many=" << many_updates << " kept old id\n";
    }
  }
}

void test_ac3_time_buffer() {
  expect_return_id(true, 2.9, false);
  expect_return_id(true, 2.9, true);
  expect_return_id(false, 3.1, false);
  expect_return_id(false, 3.1, true);
}

void test_ac4_stale_lost() {
  mentorpi_person_detect::ByteTracker tracker(default_cfg());
  auto box_at = [](float x1) { return make_box(x1, 100.0f, x1 + 100.0f, 300.0f, 0.9f); };
  double stamp = 0.0;
  const int id = confirm_id(tracker, box_at(100.0f), &stamp);
  expect(id > 0, "stale-lost fixture confirmed");

  // Steps stay inside track_buffer_s: 0.5 s × 8 shifts = 4 s would expire a stale
  // lost copy before the second miss, so AC4 would not catch D1.2.
  constexpr double kStepS = 0.2;
  stamp += kStepS;
  expect(tracker.update({}, stamp).empty(), "first loss publishes nothing");
  stamp += kStepS;
  const auto back_a = tracker.update({box_at(100.0f)}, stamp);
  expect(!back_a.empty() && back_a[0].track_id == id, "return at A keeps id");

  float x = 100.0f;
  for (int i = 0; i < 8; ++i) {
    x += 40.0f;
    stamp += kStepS;
    const auto moved = tracker.update({box_at(x)}, stamp);
    expect(!moved.empty() && moved[0].track_id == id, "shift A→B keeps id");
  }

  stamp += kStepS;
  expect(tracker.update({}, stamp).empty(), "second loss publishes nothing");
  stamp += kStepS;
  const auto back_b = tracker.update({box_at(x)}, stamp);
  expect(!back_b.empty() && back_b[0].track_id == id,
         "return at B keeps id (stale lost copy at A is gone)");
}

}  // namespace

int main() {
  test_ac5_legacy();
  test_ac1_follow_pause();
  test_ac2_velocity();
  test_ac3_time_buffer();
  test_ac4_stale_lost();

  if (g_fails == 0) {
    std::cout << "test_byte_track OK\n";
    return 0;
  }
  std::cerr << g_fails << " failure(s)\n";
  return 1;
}
