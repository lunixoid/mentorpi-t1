#pragma once

#include <cmath>

namespace mentorpi_person_detect {

// YOLO11n detect P3/P4/P5 at 640: 80x80 + 40x40 + 20x20 = 8400 proposals.
inline int yolo11n_proposal_stride(int proposal_index, int input_size = 640) {
  if (proposal_index < 0 || input_size < 32) {
    return 8;
  }
  const int n_p3 = (input_size / 8) * (input_size / 8);
  const int n_p4 = (input_size / 16) * (input_size / 16);
  if (proposal_index < n_p3) {
    return 8;
  }
  if (proposal_index < n_p3 + n_p4) {
    return 16;
  }
  return 32;
}

// Stand bag 2026-09-10: person score ~0.5 but published box ~43x33 px at ~(412,267)
// on 640x400 (physically ~13 cm at 1.28 m). Centers were already in 640 px, WH stayed
// in feature-grid units. Recover WH when XY looks like pixels and WH like a grid cell.
inline void recover_wh_if_grid_units(float cx, float cy, float* bw, float* bh, int proposal_index,
                                     int input_size = 640) {
  if (bw == nullptr || bh == nullptr) {
    return;
  }
  if (!std::isfinite(*bw) || !std::isfinite(*bh) || !std::isfinite(cx) || !std::isfinite(cy)) {
    return;
  }
  const float grid = static_cast<float>(input_size / 8);
  if (!(*bw < grid && *bh < grid && (cx > grid || cy > grid))) {
    return;
  }
  const float stride = static_cast<float>(yolo11n_proposal_stride(proposal_index, input_size));
  *bw *= stride;
  *bh *= stride;
}

}  // namespace mentorpi_person_detect
