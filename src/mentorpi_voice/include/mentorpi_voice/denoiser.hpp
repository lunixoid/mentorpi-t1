#ifndef MENTORPI_VOICE_DENOISER_HPP_
#define MENTORPI_VOICE_DENOISER_HPP_

#include <speex/speex_preprocess.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

extern "C" {
#include <rnnoise.h>
}

namespace mentorpi_voice {

// SD033 I8: noise suppression on 10 ms frames (rate / 100 samples), in place.
class Denoiser {
 public:
  virtual ~Denoiser() = default;
  virtual void process(int16_t* frame) = 0;
  virtual void reset() = 0;
  virtual const char* name() const = 0;
};

struct DenoiseOptions {
  double speex_noise_suppress_db{-30.0};
};

namespace detail {

inline int16_t saturate_sample(float value) {
  const long rounded = std::lround(value);
  if (rounded > 32767) {
    return 32767;
  }
  if (rounded < -32768) {
    return -32768;
  }
  return static_cast<int16_t>(rounded);
}

class NoneDenoiser : public Denoiser {
 public:
  void process(int16_t*) override {}
  void reset() override {}
  const char* name() const override { return "none"; }
};

// RNNoise 0.2 with the built-in model: 48 kHz, 480-sample frames, float samples in int16 scale.
class RnnoiseDenoiser : public Denoiser {
 public:
  RnnoiseDenoiser() : state_(rnnoise_create(nullptr)) {
    const int frame = rnnoise_get_frame_size();
    if (frame > 0) {
      in_.resize(static_cast<size_t>(frame));
      out_.resize(static_cast<size_t>(frame));
    }
  }

  RnnoiseDenoiser(const RnnoiseDenoiser&) = delete;
  RnnoiseDenoiser& operator=(const RnnoiseDenoiser&) = delete;

  ~RnnoiseDenoiser() override {
    if (state_ != nullptr) {
      rnnoise_destroy(state_);
    }
  }

  bool ok() const { return state_ != nullptr && in_.size() == 480; }

  void process(int16_t* frame) override {
    for (size_t i = 0; i < in_.size(); ++i) {
      in_[i] = frame[i];
    }
    rnnoise_process_frame(state_, out_.data(), in_.data());
    for (size_t i = 0; i < out_.size(); ++i) {
      frame[i] = saturate_sample(out_[i]);
    }
  }

  void reset() override { rnnoise_init(state_, nullptr); }

  const char* name() const override { return "rnnoise"; }

 private:
  DenoiseState* state_{nullptr};
  std::vector<float> in_;
  std::vector<float> out_;
};

// SpeexDSP preprocessor: denoise only, AGC, VAD and dereverb off.
class SpeexDenoiser : public Denoiser {
 public:
  SpeexDenoiser(unsigned rate, double noise_suppress_db)
      : rate_(rate), noise_suppress_db_(noise_suppress_db) {
    init();
  }

  SpeexDenoiser(const SpeexDenoiser&) = delete;
  SpeexDenoiser& operator=(const SpeexDenoiser&) = delete;

  ~SpeexDenoiser() override { destroy(); }

  bool ok() const { return state_ != nullptr; }

  void process(int16_t* frame) override {
    speex_preprocess_run(state_, reinterpret_cast<spx_int16_t*>(frame));
  }

  void reset() override {
    destroy();
    init();
  }

  const char* name() const override { return "speexdsp"; }

 private:
  void init() {
    state_ = speex_preprocess_state_init(static_cast<int>(rate_ / 100), static_cast<int>(rate_));
    if (state_ == nullptr) {
      return;
    }
    int on = 1;
    int off = 0;
    int suppress = static_cast<int>(std::lround(noise_suppress_db_));
    speex_preprocess_ctl(state_, SPEEX_PREPROCESS_SET_DENOISE, &on);
    speex_preprocess_ctl(state_, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &suppress);
    speex_preprocess_ctl(state_, SPEEX_PREPROCESS_SET_AGC, &off);
    speex_preprocess_ctl(state_, SPEEX_PREPROCESS_SET_DEREVERB, &off);
    // VAD is off by default; SET_VAD prints "The VAD has been replaced by a hack" on every call.
  }

  void destroy() {
    if (state_ != nullptr) {
      speex_preprocess_state_destroy(state_);
      state_ = nullptr;
    }
  }

  unsigned rate_{48000};
  double noise_suppress_db_{-30.0};
  SpeexPreprocessState* state_{nullptr};
};

}  // namespace detail

// Always returns a denoiser. On any failure it is "none" and error explains why (SD033 D2.3).
inline std::unique_ptr<Denoiser> make_denoiser(std::string_view kind, unsigned rate,
                                               const DenoiseOptions& opt, std::string& error) {
  error.clear();
  if (kind == "none") {
    return std::make_unique<detail::NoneDenoiser>();
  }
  if (kind == "rnnoise") {
    if (rate != 48000) {
      error = "rnnoise needs rate 48000, got " + std::to_string(rate);
    } else {
      auto denoiser = std::make_unique<detail::RnnoiseDenoiser>();
      if (denoiser->ok()) {
        return denoiser;
      }
      error = "rnnoise_create failed or frame size is not 480";
    }
  } else if (kind == "speexdsp") {
    if (rate == 0 || rate % 100 != 0) {
      error = "speexdsp needs a rate divisible by 100, got " + std::to_string(rate);
    } else if (opt.speex_noise_suppress_db > 0.0) {
      error = "speex_noise_suppress_db must be <= 0";
    } else {
      auto denoiser = std::make_unique<detail::SpeexDenoiser>(rate, opt.speex_noise_suppress_db);
      if (denoiser->ok()) {
        return denoiser;
      }
      error = "speex_preprocess_state_init failed";
    }
  } else {
    error = "unknown denoise kind '" + std::string(kind) + "' (none|rnnoise|speexdsp)";
  }
  return std::make_unique<detail::NoneDenoiser>();
}

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_DENOISER_HPP_
