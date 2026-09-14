#pragma once

#include <cstdint>
#include <vector>

namespace mentorpi_person_detect {

struct DetectionBox {
  float x1{0.0f};
  float y1{0.0f};
  float x2{0.0f};
  float y2{0.0f};
  float score{0.0f};
  int class_id{0};
};

struct TrackedBox {
  float x1{0.0f};
  float y1{0.0f};
  float x2{0.0f};
  float y2{0.0f};
  float score{0.0f};
  int class_id{0};
  int track_id{0};
};

struct ByteTrackConfig {
  float track_high_thresh{0.25f};
  float track_low_thresh{0.1f};
  float new_track_thresh{0.25f};
  float match_thresh{0.8f};
  bool fuse_score{false};
  double track_buffer_s{3.0};
};

// Ultralytics-compatible ByteTrack: only is_activated tracks are returned.
class ByteTracker {
 public:
  explicit ByteTracker(const ByteTrackConfig& config = ByteTrackConfig{});

  ~ByteTracker();

  ByteTracker(const ByteTracker&) = delete;
  ByteTracker& operator=(const ByteTracker&) = delete;

  std::vector<TrackedBox> update(const std::vector<DetectionBox>& detections, double stamp_s);

  void reset();

 private:
  ByteTrackConfig config_;
  int frame_id_{0};
  int next_id_{1};
  class Impl;
  Impl* impl_;
};

}  // namespace mentorpi_person_detect
