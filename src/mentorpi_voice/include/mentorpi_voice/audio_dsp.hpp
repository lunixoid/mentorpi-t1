#ifndef MENTORPI_VOICE_AUDIO_DSP_HPP_
#define MENTORPI_VOICE_AUDIO_DSP_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace mentorpi_voice {

// SD033 I7: capture-side helpers shared by voice_command and the offline bench.

inline void mix_to_mono(const int16_t* interleaved, size_t frames, unsigned channels,
                        std::vector<int16_t>& out) {
  out.resize(frames);
  if (channels <= 1) {
    std::copy(interleaved, interleaved + frames, out.begin());
    return;
  }
  for (size_t i = 0; i < frames; ++i) {
    int32_t sum = 0;
    for (unsigned ch = 0; ch < channels; ++ch) {
      sum += interleaved[i * channels + ch];
    }
    out[i] = static_cast<int16_t>(sum / static_cast<int32_t>(channels));
  }
}

// Low-pass FIR (Hamming-windowed sinc, 21 taps per factor, cutoff 0.9 of the output Nyquist)
// followed by keeping every factor-th sample. State survives between process() calls, so
// chunk boundaries do not change the output.
class Decimator {
 public:
  explicit Decimator(unsigned factor) : factor_(factor) {
    if (factor_ == 0) {
      throw std::invalid_argument("decimation factor must be > 0");
    }
    if (factor_ > 1) {
      design_taps();
      history_.assign(2 * taps_.size(), 0.0);
    }
  }

  void process(const int16_t* in, size_t n, std::vector<int16_t>& out) {
    out.clear();
    if (factor_ == 1) {
      out.assign(in, in + n);
      return;
    }
    out.reserve(n / factor_ + 1);
    const size_t len = taps_.size();
    for (size_t i = 0; i < n; ++i) {
      history_[pos_] = in[i];
      history_[pos_ + len] = in[i];
      pos_ = (pos_ + 1) % len;
      if (++phase_ < factor_) {
        continue;
      }
      phase_ = 0;
      // history_[pos_, pos_ + len) holds the last len samples, oldest first.
      const double* window = history_.data() + pos_;
      double acc = 0.0;
      for (size_t k = 0; k < len; ++k) {
        acc += taps_[k] * window[k];
      }
      out.push_back(saturate(acc));
    }
  }

  void reset() {
    std::fill(history_.begin(), history_.end(), 0.0);
    pos_ = 0;
    phase_ = 0;
  }

  unsigned factor() const { return factor_; }

 private:
  static constexpr double kPi = 3.14159265358979323846;

  static int16_t saturate(double value) {
    const double rounded = std::round(value);
    if (rounded > 32767.0) {
      return 32767;
    }
    if (rounded < -32768.0) {
      return -32768;
    }
    return static_cast<int16_t>(rounded);
  }

  void design_taps() {
    const size_t len = (21 * static_cast<size_t>(factor_)) | 1u;
    const double cutoff = 0.45 / static_cast<double>(factor_);
    const double mid = static_cast<double>(len - 1) / 2.0;
    taps_.resize(len);
    double sum = 0.0;
    for (size_t k = 0; k < len; ++k) {
      const double x = static_cast<double>(k) - mid;
      const double sinc = x == 0.0 ? 2.0 * cutoff : std::sin(2.0 * kPi * cutoff * x) / (kPi * x);
      const double window =
          0.54 - 0.46 * std::cos(2.0 * kPi * static_cast<double>(k) / static_cast<double>(len - 1));
      taps_[k] = sinc * window;
      sum += taps_[k];
    }
    for (double& tap : taps_) {
      tap /= sum;
    }
  }

  unsigned factor_{1};
  std::vector<double> taps_;
  std::vector<double> history_;
  size_t pos_{0};
  unsigned phase_{0};
};

struct LevelStats {
  double peak_dbfs{-120.0};
  double rms_dbfs{-120.0};
  double clipped_share{0.0};
  size_t samples{0};
};

// Peak, RMS and share of clipped samples (32767 or -32768) since the last take().
class LevelMeter {
 public:
  void add(const int16_t* data, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      const int32_t v = data[i];
      const int32_t a = v < 0 ? -v : v;
      peak_ = std::max(peak_, a);
      sum_sq_ += static_cast<double>(v) * static_cast<double>(v);
      if (v == 32767 || v == -32768) {
        ++clipped_;
      }
    }
    samples_ += n;
  }

  LevelStats take() {
    LevelStats stats;
    stats.samples = samples_;
    if (samples_ > 0) {
      stats.peak_dbfs = to_dbfs(static_cast<double>(peak_));
      stats.rms_dbfs = to_dbfs(std::sqrt(sum_sq_ / static_cast<double>(samples_)));
      stats.clipped_share = static_cast<double>(clipped_) / static_cast<double>(samples_);
    }
    peak_ = 0;
    sum_sq_ = 0.0;
    clipped_ = 0;
    samples_ = 0;
    return stats;
  }

 private:
  static double to_dbfs(double amplitude) {
    if (amplitude <= 0.0) {
      return -120.0;
    }
    return std::max(-120.0, 20.0 * std::log10(amplitude / 32768.0));
  }

  int32_t peak_{0};
  double sum_sq_{0.0};
  size_t clipped_{0};
  size_t samples_{0};
};

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_AUDIO_DSP_HPP_
