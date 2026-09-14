#include "mentorpi_person_detect/byte_track.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_set>
#include <vector>

namespace mentorpi_person_detect {
namespace {

enum class TrackState { kNew, kTracked, kLost, kRemoved };

struct Rect {
  float x1{0.0f};
  float y1{0.0f};
  float x2{0.0f};
  float y2{0.0f};
};

float box_iou(const Rect& a, const Rect& b) {
  const float ix1 = std::max(a.x1, b.x1);
  const float iy1 = std::max(a.y1, b.y1);
  const float ix2 = std::min(a.x2, b.x2);
  const float iy2 = std::min(a.y2, b.y2);
  const float iw = std::max(0.0f, ix2 - ix1);
  const float ih = std::max(0.0f, iy2 - iy1);
  const float inter = iw * ih;
  const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
  const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
  const float union_area = area_a + area_b - inter;
  if (union_area <= 0.0f) {
    return 0.0f;
  }
  return inter / union_area;
}

Rect det_to_rect(const DetectionBox& det) { return Rect{det.x1, det.y1, det.x2, det.y2}; }

bool cholesky4(const std::array<double, 16>& s, std::array<double, 16>& l) {
  l.fill(0.0);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = s[static_cast<std::size_t>(i * 4 + j)];
      for (int k = 0; k < j; ++k) {
        sum -= l[static_cast<std::size_t>(i * 4 + k)] * l[static_cast<std::size_t>(j * 4 + k)];
      }
      if (i == j) {
        if (sum <= 0.0) {
          return false;
        }
        l[static_cast<std::size_t>(i * 4 + j)] = std::sqrt(sum);
      } else {
        l[static_cast<std::size_t>(i * 4 + j)] = sum / l[static_cast<std::size_t>(j * 4 + j)];
      }
    }
  }
  return true;
}

void chol_solve4(const std::array<double, 16>& l, const double* b, double* x) {
  double y[4];
  for (int i = 0; i < 4; ++i) {
    double sum = b[i];
    for (int k = 0; k < i; ++k) {
      sum -= l[static_cast<std::size_t>(i * 4 + k)] * y[k];
    }
    y[i] = sum / l[static_cast<std::size_t>(i * 4 + i)];
  }
  for (int i = 3; i >= 0; --i) {
    double sum = y[i];
    for (int k = i + 1; k < 4; ++k) {
      sum -= l[static_cast<std::size_t>(k * 4 + i)] * x[k];
    }
    x[i] = sum / l[static_cast<std::size_t>(i * 4 + i)];
  }
}

// Ultralytics KalmanFilterXYAH: state [cx, cy, a, h, vcx, vcy, va, vh], dt = 1.
struct KalmanFilterXYAH {
  static constexpr int kStateDim = 8;
  static constexpr double kStdWeightPosition = 1.0 / 20.0;
  static constexpr double kStdWeightVelocity = 1.0 / 160.0;

  void initiate(const std::array<double, 4>& measurement, std::array<double, kStateDim>& mean,
                std::array<double, kStateDim * kStateDim>& cov) const {
    mean = {measurement[0], measurement[1], measurement[2], measurement[3], 0.0, 0.0, 0.0, 0.0};
    cov.fill(0.0);
    const double h = measurement[3];
    const double std_dev[kStateDim] = {2.0 * kStdWeightPosition * h,
                                       2.0 * kStdWeightPosition * h,
                                       1e-2,
                                       2.0 * kStdWeightPosition * h,
                                       10.0 * kStdWeightVelocity * h,
                                       10.0 * kStdWeightVelocity * h,
                                       1e-5,
                                       10.0 * kStdWeightVelocity * h};
    for (int i = 0; i < kStateDim; ++i) {
      cov[static_cast<std::size_t>(i * kStateDim + i)] = std_dev[i] * std_dev[i];
    }
  }

  void predict(std::array<double, kStateDim>& mean,
               std::array<double, kStateDim * kStateDim>& cov) const {
    const double h = mean[3];
    const double std_pos = kStdWeightPosition * h;
    const double std_vel = kStdWeightVelocity * h;
    const double q_std[kStateDim] = {std_pos, std_pos, 1e-2, std_pos,
                                     std_vel, std_vel, 1e-5, std_vel};

    mean[0] += mean[4];
    mean[1] += mean[5];
    mean[2] += mean[6];
    mean[3] += mean[7];

    std::array<double, kStateDim * kStateDim> fp{};
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < kStateDim; ++j) {
        double v = cov[static_cast<std::size_t>(i * kStateDim + j)];
        if (i < 4) {
          v += cov[static_cast<std::size_t>((i + 4) * kStateDim + j)];
        }
        fp[static_cast<std::size_t>(i * kStateDim + j)] = v;
      }
    }
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < kStateDim; ++j) {
        double v = fp[static_cast<std::size_t>(i * kStateDim + j)];
        if (j < 4) {
          v += fp[static_cast<std::size_t>(i * kStateDim + (j + 4))];
        }
        cov[static_cast<std::size_t>(i * kStateDim + j)] = v;
      }
    }
    for (int i = 0; i < kStateDim; ++i) {
      cov[static_cast<std::size_t>(i * kStateDim + i)] += q_std[i] * q_std[i];
    }
  }

  void update(const std::array<double, 4>& measurement, std::array<double, kStateDim>& mean,
              std::array<double, kStateDim * kStateDim>& cov) const {
    const double h = mean[3];
    const double std_pos = kStdWeightPosition * h;
    const double r_std[4] = {std_pos, std_pos, 1e-1, std_pos};

    std::array<double, 16> s{};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        s[static_cast<std::size_t>(i * 4 + j)] = cov[static_cast<std::size_t>(i * kStateDim + j)];
      }
      s[static_cast<std::size_t>(i * 4 + i)] += r_std[i] * r_std[i];
    }

    std::array<double, 16> l{};
    double ridge = 1e-9;
    bool ok = false;
    for (int attempt = 0; attempt < 8; ++attempt) {
      std::array<double, 16> s_reg = s;
      for (int i = 0; i < 4; ++i) {
        s_reg[static_cast<std::size_t>(i * 4 + i)] += ridge;
      }
      if (cholesky4(s_reg, l)) {
        s = s_reg;
        ok = true;
        break;
      }
      ridge *= 10.0;
    }
    if (!ok) {
      return;
    }

    // K = (solve(S, P[:, :4].T)).T  → 8×4
    double k_gain[kStateDim * 4];
    for (int i = 0; i < kStateDim; ++i) {
      double b[4] = {cov[static_cast<std::size_t>(i * kStateDim + 0)],
                     cov[static_cast<std::size_t>(i * kStateDim + 1)],
                     cov[static_cast<std::size_t>(i * kStateDim + 2)],
                     cov[static_cast<std::size_t>(i * kStateDim + 3)]};
      double x[4];
      chol_solve4(l, b, x);
      for (int j = 0; j < 4; ++j) {
        k_gain[i * 4 + j] = x[j];
      }
    }

    double innovation[4];
    for (int j = 0; j < 4; ++j) {
      innovation[j] = measurement[j] - mean[static_cast<std::size_t>(j)];
    }
    for (int i = 0; i < kStateDim; ++i) {
      double add = 0.0;
      for (int j = 0; j < 4; ++j) {
        add += k_gain[i * 4 + j] * innovation[j];
      }
      mean[static_cast<std::size_t>(i)] += add;
    }

    // P ← P − K S Kᵀ
    double ks[kStateDim * 4];
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < 4; ++j) {
        double sum = 0.0;
        for (int n = 0; n < 4; ++n) {
          sum += k_gain[i * 4 + n] * s[static_cast<std::size_t>(n * 4 + j)];
        }
        ks[i * 4 + j] = sum;
      }
    }
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < kStateDim; ++j) {
        double sub = 0.0;
        for (int n = 0; n < 4; ++n) {
          sub += ks[i * 4 + n] * k_gain[j * 4 + n];
        }
        cov[static_cast<std::size_t>(i * kStateDim + j)] -= sub;
      }
    }
  }
};

KalmanFilterXYAH g_shared_kalman;

std::array<double, 4> tlwh_to_xyah(float x, float y, float w, float h) {
  const double cx = static_cast<double>(x) + static_cast<double>(w) * 0.5;
  const double cy = static_cast<double>(y) + static_cast<double>(h) * 0.5;
  const double safe_h = std::max(static_cast<double>(h), 1e-3);
  return {cx, cy, static_cast<double>(w) / safe_h, safe_h};
}

Rect state_to_rect(const std::array<double, KalmanFilterXYAH::kStateDim>& mean) {
  const double w = mean[2] * mean[3];
  const double h = mean[3];
  return Rect{static_cast<float>(mean[0] - w * 0.5), static_cast<float>(mean[1] - h * 0.5),
              static_cast<float>(mean[0] + w * 0.5), static_cast<float>(mean[1] + h * 0.5)};
}

struct STrack {
  Rect rect{};
  float score{0.0f};
  int class_id{0};
  int track_id{0};
  int frame_id{0};
  int start_frame{0};
  int tracklet_len{0};
  double last_update_s{0.0};
  bool is_activated{false};
  TrackState state{TrackState::kNew};
  std::array<double, KalmanFilterXYAH::kStateDim> mean{};
  std::array<double, KalmanFilterXYAH::kStateDim * KalmanFilterXYAH::kStateDim> covariance{};

  static STrack from_detection(const DetectionBox& det) {
    STrack track;
    track.rect = det_to_rect(det);
    track.score = det.score;
    track.class_id = det.class_id;
    const float w = std::max(0.0f, track.rect.x2 - track.rect.x1);
    const float h = std::max(0.0f, track.rect.y2 - track.rect.y1);
    const auto xyah = tlwh_to_xyah(track.rect.x1, track.rect.y1, w, h);
    g_shared_kalman.initiate(xyah, track.mean, track.covariance);
    return track;
  }

  void activate(int next_id, int current_frame, double stamp_s) {
    track_id = next_id;
    state = TrackState::kTracked;
    is_activated = (current_frame == 1);
    frame_id = current_frame;
    start_frame = current_frame;
    tracklet_len = 0;
    last_update_s = stamp_s;
    rect = state_to_rect(mean);
  }

  void re_activate(const STrack& new_track, int current_frame, bool new_id, int& next_id,
                   double stamp_s) {
    update_kalman(new_track);
    tracklet_len = 0;
    state = TrackState::kTracked;
    score = new_track.score;
    class_id = new_track.class_id;
    frame_id = current_frame;
    last_update_s = stamp_s;
    is_activated = true;
    if (new_id) {
      track_id = next_id++;
    }
  }

  void update(const STrack& new_track, int current_frame, double stamp_s) {
    update_kalman(new_track);
    state = TrackState::kTracked;
    score = new_track.score;
    class_id = new_track.class_id;
    frame_id = current_frame;
    last_update_s = stamp_s;
    tracklet_len += 1;
    is_activated = true;
  }

  void predict() {
    if (state != TrackState::kTracked) {
      mean[7] = 0.0;
    }
    g_shared_kalman.predict(mean, covariance);
    rect = state_to_rect(mean);
  }

  void mark_lost() { state = TrackState::kLost; }

  void mark_removed() { state = TrackState::kRemoved; }

  TrackedBox to_output() const {
    TrackedBox out;
    out.x1 = rect.x1;
    out.y1 = rect.y1;
    out.x2 = rect.x2;
    out.y2 = rect.y2;
    out.score = score;
    out.class_id = class_id;
    out.track_id = track_id;
    return out;
  }

 private:
  void update_kalman(const STrack& new_track) {
    const float w = std::max(0.0f, new_track.rect.x2 - new_track.rect.x1);
    const float h = std::max(0.0f, new_track.rect.y2 - new_track.rect.y1);
    const auto xyah = tlwh_to_xyah(new_track.rect.x1, new_track.rect.y1, w, h);
    g_shared_kalman.update(xyah, mean, covariance);
    rect = state_to_rect(mean);
  }
};

struct MatchPair {
  int row{0};
  int col{0};
  float cost{0.0f};
};

std::vector<std::vector<float>> iou_distance(const std::vector<STrack>& tracks,
                                             const std::vector<STrack>& detections,
                                             bool fuse_score) {
  std::vector<std::vector<float>> cost(tracks.size(), std::vector<float>(detections.size(), 1.0f));
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    for (std::size_t j = 0; j < detections.size(); ++j) {
      const float iou = box_iou(tracks[i].rect, detections[j].rect);
      float c = 1.0f - iou;
      if (fuse_score) {
        c = 1.0f - iou * detections[j].score;
      }
      cost[i][j] = c;
    }
  }
  return cost;
}

void linear_assignment(const std::vector<std::vector<float>>& cost, int num_b, float thresh,
                       std::vector<std::pair<int, int>>& matches, std::vector<int>& unmatched_a,
                       std::vector<int>& unmatched_b) {
  const int n_a = static_cast<int>(cost.size());
  const int n_b = num_b;
  matches.clear();
  unmatched_a.clear();
  unmatched_b.clear();
  if (n_a == 0) {
    for (int j = 0; j < n_b; ++j) {
      unmatched_b.push_back(j);
    }
    return;
  }
  if (n_b == 0) {
    for (int i = 0; i < n_a; ++i) {
      unmatched_a.push_back(i);
    }
    return;
  }

  std::vector<MatchPair> pairs;
  for (int i = 0; i < n_a; ++i) {
    for (int j = 0; j < n_b; ++j) {
      pairs.push_back(MatchPair{i, j, cost[i][j]});
    }
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const MatchPair& a, const MatchPair& b) { return a.cost < b.cost; });

  std::vector<bool> used_a(n_a, false);
  std::vector<bool> used_b(n_b, false);
  for (const auto& p : pairs) {
    if (p.cost > thresh) {
      break;
    }
    if (used_a[p.row] || used_b[p.col]) {
      continue;
    }
    used_a[p.row] = true;
    used_b[p.col] = true;
    matches.emplace_back(p.row, p.col);
  }
  for (int i = 0; i < n_a; ++i) {
    if (!used_a[i]) {
      unmatched_a.push_back(i);
    }
  }
  for (int j = 0; j < n_b; ++j) {
    if (!used_b[j]) {
      unmatched_b.push_back(j);
    }
  }
}

std::vector<STrack> merge_track_lists(const std::vector<STrack>& a, const std::vector<STrack>& b) {
  std::vector<STrack> out = a;
  for (const auto& t : b) {
    bool found = false;
    for (const auto& existing : out) {
      if (existing.track_id == t.track_id) {
        found = true;
        break;
      }
    }
    if (!found) {
      out.push_back(t);
    }
  }
  return out;
}

std::unordered_set<int> track_ids_of(const std::vector<STrack>& tracks) {
  std::unordered_set<int> ids;
  ids.reserve(tracks.size());
  for (const auto& t : tracks) {
    ids.insert(t.track_id);
  }
  return ids;
}

}  // namespace

class ByteTracker::Impl {
 public:
  explicit Impl(const ByteTrackConfig& config) : config_(config) {}

  std::vector<TrackedBox> update(const std::vector<DetectionBox>& detections, double stamp_s,
                                 int& frame_id, int& next_id) {
    frame_id += 1;

    std::vector<STrack> kept_lost;
    kept_lost.reserve(lost_stracks_.size());
    for (const auto& t : lost_stracks_) {
      double age = stamp_s - t.last_update_s;
      if (age < 0.0) {
        age = 0.0;
      }
      if (age > config_.track_buffer_s) {
        continue;
      }
      kept_lost.push_back(t);
    }
    lost_stracks_ = std::move(kept_lost);

    std::vector<STrack> det_stracks;
    det_stracks.reserve(detections.size());
    for (const auto& det : detections) {
      det_stracks.push_back(STrack::from_detection(det));
    }

    std::vector<STrack> detections_high;
    std::vector<STrack> detections_low;
    for (const auto& det : det_stracks) {
      if (det.score >= config_.track_high_thresh) {
        detections_high.push_back(det);
      } else if (det.score >= config_.track_low_thresh) {
        detections_low.push_back(det);
      }
    }

    std::vector<STrack> unconfirmed;
    std::vector<STrack> tracked;
    for (const auto& t : tracked_stracks_) {
      if (!t.is_activated) {
        unconfirmed.push_back(t);
      } else {
        tracked.push_back(t);
      }
    }

    std::vector<STrack> strack_pool = merge_track_lists(tracked, lost_stracks_);
    for (auto& t : strack_pool) {
      t.predict();
    }

    std::vector<std::pair<int, int>> matches;
    std::vector<int> u_track;
    std::vector<int> u_detection;
    const auto dists = iou_distance(strack_pool, detections_high, config_.fuse_score);
    linear_assignment(dists, static_cast<int>(detections_high.size()), config_.match_thresh,
                      matches, u_track, u_detection);

    std::vector<STrack> activated;
    std::vector<STrack> refind;
    for (const auto& m : matches) {
      auto& track = strack_pool[static_cast<std::size_t>(m.first)];
      const auto& det = detections_high[static_cast<std::size_t>(m.second)];
      if (track.state == TrackState::kTracked) {
        track.update(det, frame_id, stamp_s);
        activated.push_back(track);
      } else {
        track.re_activate(det, frame_id, false, next_id, stamp_s);
        refind.push_back(track);
      }
    }

    std::vector<STrack> r_tracked;
    for (int idx : u_track) {
      r_tracked.push_back(strack_pool[static_cast<std::size_t>(idx)]);
    }
    const auto dists_low = iou_distance(r_tracked, detections_low, config_.fuse_score);
    std::vector<std::pair<int, int>> matches_low;
    std::vector<int> u_track_low;
    std::vector<int> u_detection_low;
    linear_assignment(dists_low, static_cast<int>(detections_low.size()), 0.5f, matches_low,
                      u_track_low, u_detection_low);
    for (const auto& m : matches_low) {
      auto& track = r_tracked[static_cast<std::size_t>(m.first)];
      const auto& det = detections_low[static_cast<std::size_t>(m.second)];
      if (track.state == TrackState::kTracked) {
        track.update(det, frame_id, stamp_s);
        activated.push_back(track);
      } else {
        track.re_activate(det, frame_id, false, next_id, stamp_s);
        refind.push_back(track);
      }
    }

    for (int idx : u_track_low) {
      auto& track = r_tracked[static_cast<std::size_t>(idx)];
      if (track.state != TrackState::kLost) {
        track.mark_lost();
      }
    }

    std::vector<STrack> detections_high_remain;
    for (int idx : u_detection) {
      detections_high_remain.push_back(detections_high[static_cast<std::size_t>(idx)]);
    }

    const auto dists_unconfirmed =
        iou_distance(unconfirmed, detections_high_remain, config_.fuse_score);
    std::vector<std::pair<int, int>> matches_unconfirmed;
    std::vector<int> u_unconfirmed;
    std::vector<int> u_detection_remain;
    linear_assignment(dists_unconfirmed, static_cast<int>(detections_high_remain.size()), 0.7f,
                      matches_unconfirmed, u_unconfirmed, u_detection_remain);

    for (const auto& m : matches_unconfirmed) {
      auto track = unconfirmed[static_cast<std::size_t>(m.first)];
      track.update(detections_high_remain[static_cast<std::size_t>(m.second)], frame_id, stamp_s);
      activated.push_back(track);
    }

    for (int idx : u_unconfirmed) {
      unconfirmed[static_cast<std::size_t>(idx)].mark_removed();
    }

    std::vector<STrack> detections_for_new;
    for (int idx : u_detection_remain) {
      detections_for_new.push_back(detections_high_remain[static_cast<std::size_t>(idx)]);
    }
    for (const auto& det : detections_for_new) {
      if (det.score < config_.new_track_thresh) {
        continue;
      }
      STrack new_track = det;
      new_track.activate(next_id++, frame_id, stamp_s);
      activated.push_back(new_track);
    }

    std::vector<STrack> next_tracked;
    for (const auto& t : activated) {
      if (t.state == TrackState::kTracked) {
        next_tracked.push_back(t);
      }
    }
    for (const auto& t : refind) {
      if (t.state == TrackState::kTracked) {
        next_tracked.push_back(t);
      }
    }
    tracked_stracks_ = std::move(next_tracked);

    const auto tracked_ids = track_ids_of(tracked_stracks_);
    std::vector<STrack> next_lost;
    for (int idx : u_track_low) {
      const auto& track = r_tracked[static_cast<std::size_t>(idx)];
      if (track.state == TrackState::kLost && tracked_ids.count(track.track_id) == 0) {
        next_lost.push_back(track);
      }
    }
    lost_stracks_ = std::move(next_lost);

    std::vector<TrackedBox> output;
    for (const auto& t : tracked_stracks_) {
      if (!t.is_activated) {
        continue;
      }
      output.push_back(t.to_output());
    }
    return output;
  }

  void reset() {
    tracked_stracks_.clear();
    lost_stracks_.clear();
  }

 private:
  ByteTrackConfig config_;
  std::vector<STrack> tracked_stracks_;
  std::vector<STrack> lost_stracks_;
};

ByteTracker::ByteTracker(const ByteTrackConfig& config)
    : config_(config), impl_(new Impl(config)) {}

ByteTracker::~ByteTracker() { delete impl_; }

std::vector<TrackedBox> ByteTracker::update(const std::vector<DetectionBox>& detections,
                                            double stamp_s) {
  return impl_->update(detections, stamp_s, frame_id_, next_id_);
}

void ByteTracker::reset() {
  frame_id_ = 0;
  next_id_ = 1;
  impl_->reset();
}

}  // namespace mentorpi_person_detect
