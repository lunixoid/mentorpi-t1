#ifndef MENTORPI_VOICE_COMMAND_PIPELINE_HPP_
#define MENTORPI_VOICE_COMMAND_PIPELINE_HPP_

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mentorpi_voice/audio_dsp.hpp"
#include "mentorpi_voice/denoiser.hpp"
#include "mentorpi_voice/dual_channel.hpp"
#include "mentorpi_voice/phrase_match.hpp"
#include "mentorpi_voice/utterance.hpp"
#include "vosk_api.h"

namespace mentorpi_voice {

// SD033 I9: the recognizer behind the pipeline; tests substitute a scripted engine.
class AsrEngine {
 public:
  virtual ~AsrEngine() = default;
  virtual int accept(const int16_t* data, int n) = 0;
  virtual std::string partial() = 0;
  virtual std::string result() = 0;
  virtual std::string final_result() = 0;
  virtual void reset() = 0;
};

class VoskEngine : public AsrEngine {
 public:
  VoskEngine(VoskModel* model, float sample_rate, const std::vector<std::string>& phrases) {
    if (model != nullptr) {
      const std::string grammar = grammar_json(phrases);
      rec_ = vosk_recognizer_new_grm(model, sample_rate, grammar.c_str());
    }
    if (rec_ != nullptr) {
      vosk_recognizer_set_words(rec_, 1);
    }
  }

  VoskEngine(const VoskEngine&) = delete;
  VoskEngine& operator=(const VoskEngine&) = delete;

  ~VoskEngine() override {
    if (rec_ != nullptr) {
      vosk_recognizer_free(rec_);
    }
  }

  bool ok() const { return rec_ != nullptr; }

  int accept(const int16_t* data, int n) override {
    return vosk_recognizer_accept_waveform_s(rec_, data, n);
  }
  std::string partial() override { return vosk_recognizer_partial_result(rec_); }
  std::string result() override { return vosk_recognizer_result(rec_); }
  std::string final_result() override { return vosk_recognizer_final_result(rec_); }
  void reset() override { vosk_recognizer_reset(rec_); }

 private:
  VoskRecognizer* rec_{nullptr};
};

struct PipelineConfig {
  unsigned capture_rate{48000};
  unsigned asr_rate{16000};
  unsigned channels{1};
  std::string denoise{"none"};
  DenoiseOptions denoise_options;
  // SD035 I5: stage over both capture channels, before the mono mix.
  std::string dual{"none"};
  DualOptions dual_options;
  bool partial_trigger{true};
  std::chrono::milliseconds partial_stable{200};
  std::chrono::milliseconds max_utterance{5000};
  std::vector<std::string> phrases;
  std::vector<std::string> commands;
  double min_confidence{0.6};
};

struct PipelineEvent {
  bool command{false};
  std::optional<size_t> phrase;
  std::string text;
  double conf_min{0.0};  // -1 for via=partial: Vosk gives no confidence there
  const char* via{""};   // partial | final | forced
  const char* reason{""};
  std::chrono::milliseconds audio_time{0};
};

struct PipelineTiming {
  double dual_ms{0.0};
  double denoise_ms{0.0};
  double asr_ms{0.0};
  double audio_s{0.0};
};

// SD033 D1.2, D3: mono -> denoise (10 ms frames) -> decimate -> recognizer -> command events.
// Time inside is audio time (samples processed), so the offline bench and tests behave like the
// robot regardless of wall clock and chunk size.
class CommandPipeline {
 public:
  CommandPipeline(const PipelineConfig& config, std::unique_ptr<AsrEngine> engine)
      : config_(config),
        engine_(std::move(engine)),
        decimator_(checked_factor(config)),
        partial_(config.partial_stable) {
    if (!engine_) {
      throw std::invalid_argument("CommandPipeline needs an AsrEngine");
    }
    frame_ = std::max<size_t>(1, config_.capture_rate / 100);
    denoiser_ = make_denoiser(config_.denoise, config_.capture_rate, config_.denoise_options,
                              denoise_error_);
    dual_ = make_dual_channel(config_.dual, config_.capture_rate, frame_, config_.channels,
                              config_.dual_options, dual_error_);
  }

  const char* denoise_name() const { return denoiser_->name(); }
  const std::string& denoise_error() const { return denoise_error_; }
  const char* dual_name() const { return dual_->name(); }
  const std::string& dual_error() const { return dual_error_; }
  const std::vector<int16_t>& last_asr_block() const { return asr_block_; }

  void feed(const int16_t* interleaved, size_t frames, std::vector<PipelineEvent>& events) {
    events.clear();
    asr_block_.clear();
    if (frames == 0) {
      return;
    }
    // SD035 D1.1: both channels go through the dual stage in whole 10 ms frames, and only
    // then the pipeline works with mono. A tail shorter than a frame waits for the next block.
    const size_t channels = config_.channels == 0 ? 1 : config_.channels;
    in_pending_.insert(in_pending_.end(), interleaved, interleaved + frames * channels);
    const size_t usable = in_pending_.size() / channels / frame_ * frame_;
    if (usable == 0) {
      return;
    }
    mono_.assign(usable, 0);
    const auto dual_start = std::chrono::steady_clock::now();
    for (size_t off = 0; off < usable; off += frame_) {
      dual_->process(in_pending_.data() + off * channels, mono_.data() + off);
    }
    timing_.dual_ms += elapsed_ms(dual_start);
    in_pending_.erase(in_pending_.begin(),
                      in_pending_.begin() + static_cast<std::ptrdiff_t>(usable * channels));
    pending_.insert(pending_.end(), mono_.begin(), mono_.end());
    const size_t full = frame_ == 0 ? pending_.size() : pending_.size() / frame_ * frame_;
    if (full == 0) {
      return;
    }

    const auto denoise_start = std::chrono::steady_clock::now();
    if (frame_ > 0) {
      for (size_t off = 0; off < full; off += frame_) {
        denoiser_->process(pending_.data() + off);
      }
    }
    timing_.denoise_ms += elapsed_ms(denoise_start);
    decimator_.process(pending_.data(), full, asr_block_);
    pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(full));
    processed_ += full;
    timing_.audio_s += static_cast<double>(full) / static_cast<double>(config_.capture_rate);
    if (asr_block_.empty()) {
      return;
    }

    const auto now = audio_time();
    const auto asr_start = std::chrono::steady_clock::now();
    const int rc = engine_->accept(asr_block_.data(), static_cast<int>(asr_block_.size()));
    if (rc == 1) {
      if (!handle_final(engine_->result(), "final", now, events)) {
        // Vosk closed the utterance itself and starts a new one.
        utterance_start_ = now;
        partial_.reset();
      }
    } else if (rc < 0) {
      restart_utterance(now);
    } else {
      bool fired = false;
      if (config_.partial_trigger) {
        const std::string partial_json = engine_->partial();
        const auto idx = partial_.on_partial(partial_json, config_.phrases, now);
        if (idx.has_value() && *idx < config_.commands.size()) {
          PipelineEvent event;
          event.command = true;
          event.phrase = idx;
          event.text = parse_vosk_partial(partial_json);
          event.conf_min = -1.0;
          event.via = "partial";
          event.audio_time = now;
          events.push_back(event);
          restart_utterance(now);
          fired = true;
        }
      }
      if (!fired && config_.max_utterance.count() > 0 &&
          now - utterance_start_ >= config_.max_utterance) {
        if (!handle_final(engine_->final_result(), "forced", now, events)) {
          restart_utterance(now);
        }
      }
    }
    timing_.asr_ms += elapsed_ms(asr_start);
  }

  // Full reset (SD033 D1.3): recognizer, partial wait, denoiser and decimator state.
  void reset() {
    const auto now = audio_time();
    engine_->reset();
    partial_.reset();
    decimator_.reset();
    denoiser_->reset();
    dual_->reset();
    in_pending_.clear();
    pending_.clear();
    asr_block_.clear();
    utterance_start_ = now;
  }

  PipelineTiming take_timing() {
    const PipelineTiming out = timing_;
    timing_ = PipelineTiming{};
    return out;
  }

 private:
  static unsigned checked_factor(const PipelineConfig& config) {
    if (config.asr_rate == 0 || config.capture_rate == 0 ||
        config.capture_rate % config.asr_rate != 0) {
      throw std::invalid_argument("capture_rate must be a positive multiple of asr_rate");
    }
    return config.capture_rate / config.asr_rate;
  }

  static double elapsed_ms(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
        .count();
  }

  std::chrono::milliseconds audio_time() const {
    return std::chrono::milliseconds(
        static_cast<int64_t>(processed_ * 1000 / config_.capture_rate));
  }

  void restart_utterance(std::chrono::milliseconds now) {
    engine_->reset();
    partial_.reset();
    utterance_start_ = now;
  }

  // Returns true when the result gave a command (the recognizer is then restarted).
  bool handle_final(const std::string& json_text, const char* via, std::chrono::milliseconds now,
                    std::vector<PipelineEvent>& events) {
    const CommandDecision decision = decide_command(parse_vosk_result(json_text), config_.phrases,
                                                    config_.commands, config_.min_confidence);
    if (normalize_phrase(decision.text).empty()) {
      return false;
    }
    PipelineEvent event;
    event.text = decision.text;
    event.conf_min = decision.conf_min;
    event.via = via;
    event.audio_time = now;
    if (decision.command.has_value()) {
      event.command = true;
      event.phrase = decision.phrase;
      events.push_back(event);
      restart_utterance(now);
      return true;
    }
    event.reason = decision.ignore_reason;
    events.push_back(event);
    return false;
  }

  PipelineConfig config_;
  std::unique_ptr<AsrEngine> engine_;
  Decimator decimator_;
  PartialTrigger partial_;
  std::unique_ptr<Denoiser> denoiser_;
  std::string denoise_error_;
  std::unique_ptr<DualChannel> dual_;
  std::string dual_error_;
  size_t frame_{480};

  std::vector<int16_t> in_pending_;
  std::vector<int16_t> mono_;
  std::vector<int16_t> pending_;
  std::vector<int16_t> asr_block_;
  uint64_t processed_{0};
  std::chrono::milliseconds utterance_start_{0};
  PipelineTiming timing_;
};

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_COMMAND_PIPELINE_HPP_
