#include <cmath>
#include <iostream>

#include "mentorpi_person_detect/yolo11n_decode.hpp"

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

}  // namespace

int main() {
  using mentorpi_person_detect::recover_wh_if_grid_units;
  using mentorpi_person_detect::yolo11n_proposal_stride;

  expect(yolo11n_proposal_stride(0) == 8, "P3 start stride 8");
  expect(yolo11n_proposal_stride(6399) == 8, "P3 end stride 8");
  expect(yolo11n_proposal_stride(6400) == 16, "P4 start stride 16");
  expect(yolo11n_proposal_stride(7999) == 16, "P4 end stride 16");
  expect(yolo11n_proposal_stride(8000) == 32, "P5 start stride 32");

  // Stand bag: ~43x33 at (412,267) on P3 must become ~344x264 in 640 space.
  float bw = 43.0f;
  float bh = 33.0f;
  recover_wh_if_grid_units(412.0f, 387.0f, &bw, &bh, 5000);
  expect(std::fabs(bw - 344.0f) < 0.1f, "P3 grid WH * 8 -> width");
  expect(std::fabs(bh - 264.0f) < 0.1f, "P3 grid WH * 8 -> height");

  float pixel_w = 220.0f;
  float pixel_h = 310.0f;
  recover_wh_if_grid_units(320.0f, 240.0f, &pixel_w, &pixel_h, 100);
  expect(std::fabs(pixel_w - 220.0f) < 0.1f, "already-pixel WH is left alone");
  expect(std::fabs(pixel_h - 310.0f) < 0.1f, "already-pixel WH height left alone");

  if (g_fails == 0) {
    std::cout << "test_yolo11n_decode OK\n";
    return 0;
  }
  std::cerr << g_fails << " failure(s)\n";
  return 1;
}
