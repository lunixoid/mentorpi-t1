#ifndef MENTORPI_VOICE_DUAL_CHANNEL_HPP_
#define MENTORPI_VOICE_DUAL_CHANNEL_HPP_

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "mentorpi_voice/fft.hpp"

namespace mentorpi_voice {

// SD035 I4: the stage that turns two capture channels into one mono frame before the single
// channel denoiser of SD033. Frame is `frame` samples per channel (10 ms).
class DualChannel {
 public:
  virtual ~DualChannel() = default;
  virtual void process(const int16_t* interleaved, int16_t* mono_out) = 0;
  virtual void reset() = 0;
  virtual const char* name() const = 0;
};

struct DualOptions {
  size_t nlms_taps{256};
  double nlms_mu{0.1};
  size_t coherence_fft{512};
  double coherence_floor_db{-18.0};
};

namespace detail {

inline int16_t saturate_float(float value) {
  const long rounded = std::lround(value);
  if (rounded > 32767) {
    return 32767;
  }
  if (rounded < -32768) {
    return -32768;
  }
  return static_cast<int16_t>(rounded);
}

// SD035 D1.2: the SD033 behaviour, bit for bit the same as mix_to_mono.
class NoneDual : public DualChannel {
 public:
  NoneDual(size_t frame, unsigned channels) : frame_(frame), channels_(channels) {}

  void process(const int16_t* interleaved, int16_t* mono_out) override {
    for (size_t i = 0; i < frame_; ++i) {
      if (channels_ <= 1) {
        mono_out[i] = interleaved[i];
        continue;
      }
      int32_t sum = 0;
      for (unsigned ch = 0; ch < channels_; ++ch) {
        sum += interleaved[i * channels_ + ch];
      }
      mono_out[i] = static_cast<int16_t>(sum / static_cast<int32_t>(channels_));
    }
  }

  void reset() override {}
  const char* name() const override { return "none"; }

 private:
  size_t frame_{480};
  unsigned channels_{1};
};

// SD035 D2.1: the second channel is treated as a noise reference and its correlated part is
// subtracted from the first one. The microphones sit close to each other, so speech also reaches
// the reference and can be cancelled along with the noise; how much that costs is what the bench
// measures.
class NlmsDual : public DualChannel {
 public:
  NlmsDual(size_t frame, size_t taps, double mu)
      : frame_(frame), taps_(taps), mu_(mu), weights_(taps, 0.0f), history_(2 * taps, 0.0f) {}

  void process(const int16_t* interleaved, int16_t* mono_out) override {
    for (size_t i = 0; i < frame_; ++i) {
      const float primary = interleaved[2 * i];
      const float reference = interleaved[2 * i + 1];

      const float oldest = history_[pos_];
      energy_ += static_cast<double>(reference) * reference -
                 static_cast<double>(oldest) * oldest;
      if (energy_ < 0.0) {
        energy_ = 0.0;
      }
      history_[pos_] = reference;
      history_[pos_ + taps_] = reference;
      pos_ = (pos_ + 1) % taps_;

      const float* window = history_.data() + pos_;
      float estimate = 0.0f;
      for (size_t k = 0; k < taps_; ++k) {
        estimate += weights_[k] * window[k];
      }
      const float error = primary - estimate;

      const float step = static_cast<float>(mu_ / (energy_ + 1e3));
      for (size_t k = 0; k < taps_; ++k) {
        float value = weights_[k] + step * error * window[k];
        weights_[k] = std::clamp(value, -4.0f, 4.0f);
      }
      mono_out[i] = saturate_float(error);
    }
  }

  void reset() override {
    std::fill(weights_.begin(), weights_.end(), 0.0f);
    std::fill(history_.begin(), history_.end(), 0.0f);
    pos_ = 0;
    energy_ = 0.0;
  }

  const char* name() const override { return "nlms"; }

 private:
  size_t frame_{480};
  size_t taps_{256};
  double mu_{0.1};
  std::vector<float> weights_;
  std::vector<float> history_;
  size_t pos_{0};
  double energy_{0.0};
};

// SD035 D2.2: per band coherence between the channels. Bands where the channels agree (speech
// from one direction) pass, bands where they do not (diffuse track noise) are pushed down to the
// floor. Hann window, 50 % overlap, one frame of delay.
class CoherenceDual : public DualChannel {
 public:
  CoherenceDual(size_t frame, size_t fft_size, double floor_db)
      : frame_(frame),
        fft_(fft_size),
        hop_(fft_size / 2),
        floor_(static_cast<float>(std::pow(10.0, floor_db / 20.0))),
        window_(hann_window(fft_size)),
        sxx_(fft_size / 2 + 1, 0.0f),
        syy_(fft_size / 2 + 1, 0.0f),
        sxy_(fft_size / 2 + 1, std::complex<float>(0.0f, 0.0f)) {
    norm_.resize(hop_);
    for (size_t i = 0; i < hop_; ++i) {
      norm_[i] = window_[i] * window_[i] + window_[i + hop_] * window_[i + hop_];
      if (norm_[i] < 1e-6f) {
        norm_[i] = 1e-6f;
      }
    }
  }

  void process(const int16_t* interleaved, int16_t* mono_out) override {
    for (size_t i = 0; i < frame_; ++i) {
      left_.push_back(static_cast<float>(interleaved[2 * i]));
      right_.push_back(static_cast<float>(interleaved[2 * i + 1]));
    }
    while (left_.size() >= fft_) {
      process_hop();
      left_.erase(left_.begin(), left_.begin() + static_cast<std::ptrdiff_t>(hop_));
      right_.erase(right_.begin(), right_.begin() + static_cast<std::ptrdiff_t>(hop_));
    }
    for (size_t i = 0; i < frame_; ++i) {
      if (out_.empty()) {
        mono_out[i] = 0;  // first frame only: the stage needs one FFT window to fill
        continue;
      }
      mono_out[i] = saturate_float(out_.front());
      out_.erase(out_.begin());
    }
  }

  void reset() override {
    left_.clear();
    right_.clear();
    out_.clear();
    std::fill(sxx_.begin(), sxx_.end(), 0.0f);
    std::fill(syy_.begin(), syy_.end(), 0.0f);
    std::fill(sxy_.begin(), sxy_.end(), std::complex<float>(0.0f, 0.0f));
    tail_.assign(hop_, 0.0f);
  }

  const char* name() const override { return "coherence"; }

 private:
  void process_hop() {
    std::vector<std::complex<float>> x(fft_);
    std::vector<std::complex<float>> y(fft_);
    for (size_t i = 0; i < fft_; ++i) {
      x[i] = std::complex<float>(left_[i] * window_[i], 0.0f);
      y[i] = std::complex<float>(right_[i] * window_[i], 0.0f);
    }
    fft_forward(x);
    fft_forward(y);

    const size_t bins = fft_ / 2 + 1;
    std::vector<std::complex<float>> mixed(fft_);
    for (size_t b = 0; b < bins; ++b) {
      const std::complex<float> xb = x[b];
      const std::complex<float> yb = y[b];
      sxx_[b] = kSmooth * sxx_[b] + (1.0f - kSmooth) * std::norm(xb);
      syy_[b] = kSmooth * syy_[b] + (1.0f - kSmooth) * std::norm(yb);
      sxy_[b] = kSmooth * sxy_[b] + (1.0f - kSmooth) * (xb * std::conj(yb));
      const float denom = sxx_[b] * syy_[b] + 1e-6f;
      const float coherence = std::norm(sxy_[b]) / denom;
      const float gain = std::clamp(coherence, floor_, 1.0f);
      const std::complex<float> average = 0.5f * (xb + yb);
      mixed[b] = average * gain;
      if (b > 0 && b < fft_ / 2) {
        mixed[fft_ - b] = std::conj(mixed[b]);
      }
    }
    fft_inverse(mixed);

    if (tail_.size() != hop_) {
      tail_.assign(hop_, 0.0f);
    }
    for (size_t i = 0; i < hop_; ++i) {
      const float value = (tail_[i] + mixed[i].real() * window_[i]) / norm_[i];
      out_.push_back(value);
      tail_[i] = mixed[i + hop_].real() * window_[i + hop_];
    }
  }

  static constexpr float kSmooth = 0.8f;

  size_t frame_{480};
  size_t fft_{512};
  size_t hop_{256};
  float floor_{0.126f};
  std::vector<float> window_;
  std::vector<float> norm_;
  std::vector<float> sxx_;
  std::vector<float> syy_;
  std::vector<std::complex<float>> sxy_;
  std::vector<float> left_;
  std::vector<float> right_;
  std::vector<float> tail_;
  std::vector<float> out_;
};

}  // namespace detail

// Always returns a stage. On any failure it is "none" and error explains why (SD035 D1.3).
inline std::unique_ptr<DualChannel> make_dual_channel(std::string_view kind, unsigned rate,
                                                      size_t frame, unsigned channels,
                                                      const DualOptions& opt, std::string& error) {
  error.clear();
  (void)rate;
  if (kind == "none") {
    return std::make_unique<detail::NoneDual>(frame, channels);
  }
  if (channels != 2) {
    error = "dual needs 2 capture channels, got " + std::to_string(channels);
    return std::make_unique<detail::NoneDual>(frame, channels);
  }
  if (kind == "nlms") {
    if (opt.nlms_taps < 16 || opt.nlms_taps > 2048) {
      error = "nlms_taps must be 16-2048, got " + std::to_string(opt.nlms_taps);
    } else if (opt.nlms_mu <= 0.0 || opt.nlms_mu > 1.0) {
      error = "nlms_mu must be in (0, 1]";
    } else {
      return std::make_unique<detail::NlmsDual>(frame, opt.nlms_taps, opt.nlms_mu);
    }
  } else if (kind == "coherence") {
    if (!detail::is_power_of_two(opt.coherence_fft) || opt.coherence_fft < 128 ||
        opt.coherence_fft > 4096) {
      error = "coherence_fft must be a power of two in 128-4096, got " +
              std::to_string(opt.coherence_fft);
    } else if (opt.coherence_floor_db > 0.0) {
      error = "coherence_floor_db must be <= 0";
    } else {
      return std::make_unique<detail::CoherenceDual>(frame, opt.coherence_fft,
                                                     opt.coherence_floor_db);
    }
  } else {
    error = "unknown dual kind '" + std::string(kind) + "' (none|nlms|coherence)";
  }
  return std::make_unique<detail::NoneDual>(frame, channels);
}

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_DUAL_CHANNEL_HPP_
