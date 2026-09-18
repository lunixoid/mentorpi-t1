#ifndef MENTORPI_VOICE_FFT_HPP_
#define MENTORPI_VOICE_FFT_HPP_

#include <cmath>
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mentorpi_voice {

// SD035 I3: radix-2 FFT in place, no external libraries. Twiddles are kept in double so a 4096
// point transform still round-trips within 1e-4.
namespace detail {

inline bool is_power_of_two(size_t n) { return n >= 2 && (n & (n - 1)) == 0; }

inline void fft_radix2(std::vector<std::complex<float>>& data, bool inverse) {
  const size_t n = data.size();
  if (!is_power_of_two(n)) {
    throw std::invalid_argument("fft size must be a power of two >= 2");
  }
  constexpr double kPi = 3.14159265358979323846;

  for (size_t i = 1, j = 0; i < n; ++i) {
    size_t bit = n >> 1;
    for (; (j & bit) != 0; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      std::swap(data[i], data[j]);
    }
  }

  for (size_t len = 2; len <= n; len <<= 1) {
    const double angle = 2.0 * kPi / static_cast<double>(len) * (inverse ? 1.0 : -1.0);
    const std::complex<double> step(std::cos(angle), std::sin(angle));
    for (size_t i = 0; i < n; i += len) {
      std::complex<double> w(1.0, 0.0);
      for (size_t k = 0; k < len / 2; ++k) {
        const std::complex<float> u = data[i + k];
        const std::complex<float> v =
            data[i + k + len / 2] *
            std::complex<float>(static_cast<float>(w.real()), static_cast<float>(w.imag()));
        data[i + k] = u + v;
        data[i + k + len / 2] = u - v;
        w *= step;
      }
    }
  }

  if (inverse) {
    const float scale = 1.0f / static_cast<float>(n);
    for (auto& value : data) {
      value *= scale;
    }
  }
}

}  // namespace detail

inline void fft_forward(std::vector<std::complex<float>>& data) { detail::fft_radix2(data, false); }

inline void fft_inverse(std::vector<std::complex<float>>& data) { detail::fft_radix2(data, true); }

// Periodic Hann window: with 50 % overlap the squared windows add up to a constant.
inline std::vector<float> hann_window(size_t n) {
  constexpr double kPi = 3.14159265358979323846;
  std::vector<float> window(n);
  for (size_t i = 0; i < n; ++i) {
    window[i] = static_cast<float>(
        0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(i) / static_cast<double>(n)));
  }
  return window;
}

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_FFT_HPP_
