#include <cmath>
#include <complex>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "mentorpi_voice/fft.hpp"

using mentorpi_voice::fft_forward;
using mentorpi_voice::fft_inverse;
using mentorpi_voice::hann_window;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

constexpr double kPi = 3.14159265358979323846;

void test_round_trip() {
  const size_t n = 512;
  std::vector<std::complex<float>> data(n);
  uint32_t state = 12345;
  for (size_t i = 0; i < n; ++i) {
    state = state * 1664525u + 1013904223u;
    data[i] = std::complex<float>(static_cast<float>(static_cast<int32_t>(state >> 16) - 32768) /
                                      32768.0f,
                                  0.0f);
  }
  const std::vector<std::complex<float>> original = data;
  fft_forward(data);
  fft_inverse(data);
  double worst = 0.0;
  for (size_t i = 0; i < n; ++i) {
    worst = std::max(worst, static_cast<double>(std::abs(data[i] - original[i])));
  }
  expect(worst <= 1e-4, "forward + inverse returns the signal");
  if (worst > 1e-4) {
    std::cerr << "  worst error " << worst << '\n';
  }
}

void test_sine_peak() {
  const size_t n = 512;
  const size_t bin = 20;
  std::vector<std::complex<float>> data(n);
  for (size_t i = 0; i < n; ++i) {
    const double phase = 2.0 * kPi * static_cast<double>(bin) * static_cast<double>(i) /
                         static_cast<double>(n);
    data[i] = std::complex<float>(static_cast<float>(std::cos(phase)), 0.0f);
  }
  fft_forward(data);
  size_t peak = 0;
  double peak_mag = 0.0;
  for (size_t b = 0; b < n / 2 + 1; ++b) {
    const double mag = std::abs(data[b]);
    if (mag > peak_mag) {
      peak_mag = mag;
      peak = b;
    }
  }
  expect(peak == bin, "cosine peaks in its own bin");
  expect(peak_mag > 0.4 * static_cast<double>(n), "peak holds the energy");
}

void test_bad_size() {
  std::vector<std::complex<float>> data(300);
  bool threw = false;
  try {
    fft_forward(data);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  expect(threw, "size 300 throws");

  std::vector<std::complex<float>> single(1);
  threw = false;
  try {
    fft_inverse(single);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  expect(threw, "size 1 throws");
}

void test_hann_overlap() {
  const size_t n = 512;
  const auto window = hann_window(n);
  expect(window.size() == n, "window size");
  expect(std::fabs(window[0]) < 1e-6, "periodic window starts at zero");
  double worst = 0.0;
  for (size_t i = 0; i < n / 2; ++i) {
    const double sum = window[i] + window[i + n / 2];
    worst = std::max(worst, std::fabs(sum - 1.0));
  }
  expect(worst < 1e-5, "50 % overlap adds up to one");
}

}  // namespace

int main() {
  test_round_trip();
  test_sine_peak();
  test_bad_size();
  test_hann_overlap();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_fft: ok\n";
  return 0;
}
