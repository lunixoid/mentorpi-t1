#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mentorpi_voice/audio_dsp.hpp"
#include "mentorpi_voice/command_pipeline.hpp"

using mentorpi_voice::AsrEngine;
using mentorpi_voice::CommandPipeline;
using mentorpi_voice::Decimator;
using mentorpi_voice::mix_to_mono;
using mentorpi_voice::PipelineConfig;
using mentorpi_voice::PipelineEvent;
using std::chrono::milliseconds;

namespace {

int g_fails = 0;

void expect(bool cond, const std::string& what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

constexpr const char* kForbidPartial = R"({"partial" : "режим запрет"})";
constexpr const char* kForbidFinal =
    R"({"result":[{"conf":0.9,"word":"режим"},{"conf":0.9,"word":"запрет"}],"text":"режим запрет"})";

// Scripted recognizer on its own audio clock (samples accepted, 16 kHz). reset() hides every
// scripted partial or final whose utterance began before the reset, like a real recognizer that
// drops the audio it has heard so far.
class FakeEngine : public AsrEngine {
 public:
  struct Partial {
    int64_t from_ms;
    int64_t to_ms;
    std::string json;
  };
  struct Final {
    int64_t at_ms;
    int64_t start_ms;
    std::string json;
    bool delivered{false};
  };

  std::vector<Partial> partials;
  std::vector<Final> finals;
  int partial_calls{0};
  int final_calls{0};
  int reset_calls{0};
  std::optional<int64_t> first_final_ms;

  int accept(const int16_t*, int n) override {
    samples_ += n;
    const int64_t t = now_ms();
    for (auto& f : finals) {
      if (!f.delivered && f.at_ms <= t && f.start_ms >= cleared_ms_) {
        f.delivered = true;
        result_ = f.json;
        return 1;
      }
    }
    return 0;
  }

  std::string partial() override {
    ++partial_calls;
    const int64_t t = now_ms();
    for (const auto& p : partials) {
      if (p.from_ms <= t && t < p.to_ms && p.from_ms >= cleared_ms_) {
        return p.json;
      }
    }
    return R"({"partial" : ""})";
  }

  std::string result() override { return result_; }

  std::string final_result() override {
    ++final_calls;
    if (!first_final_ms.has_value()) {
      first_final_ms = now_ms();
    }
    return R"({"text" : ""})";
  }

  void reset() override {
    ++reset_calls;
    cleared_ms_ = now_ms();
  }

 private:
  int64_t now_ms() const { return samples_ * 1000 / 16000; }

  int64_t samples_{0};
  int64_t cleared_ms_{0};
  std::string result_;
};

PipelineConfig base_config() {
  PipelineConfig config;
  config.capture_rate = 48000;
  config.asr_rate = 16000;
  config.channels = 1;
  config.denoise = "none";
  config.partial_trigger = true;
  config.partial_stable = milliseconds(200);
  config.max_utterance = milliseconds(5000);
  config.phrases = {"режим запрет", "режим следование", "режим ручной"};
  config.commands = {"mode_forbid", "mode_follow", "mode_manual"};
  config.min_confidence = 0.6;
  return config;
}

std::vector<PipelineEvent> run_silence(CommandPipeline& pipeline, size_t total_frames,
                                       size_t chunk) {
  std::vector<PipelineEvent> all;
  std::vector<PipelineEvent> events;
  const std::vector<int16_t> zeros(chunk, 0);
  for (size_t off = 0; off < total_frames; off += chunk) {
    pipeline.feed(zeros.data(), chunk, events);
    all.insert(all.end(), events.begin(), events.end());
  }
  return all;
}

void test_partial_fires_once() {
  auto engine = std::make_unique<FakeEngine>();
  FakeEngine* fake = engine.get();
  fake->partials.push_back({100, 500, kForbidPartial});
  fake->finals.push_back({600, 100, kForbidFinal, false});
  CommandPipeline pipeline(base_config(), std::move(engine));
  expect(std::string(pipeline.denoise_name()) == "none" && pipeline.denoise_error().empty(),
         "denoise none");

  const auto events = run_silence(pipeline, 48000 * 2, 4800);
  int commands = 0;
  for (const auto& e : events) {
    if (e.command) {
      ++commands;
      expect(std::string(e.via) == "partial", "command via partial");
      expect(e.phrase.has_value() && *e.phrase == 0, "command phrase 0");
      expect(e.audio_time == milliseconds(300), "partial held 200 ms: fires at 300 ms");
    }
  }
  expect(commands == 1, "exactly one command from partial + same final");
  expect(fake->reset_calls >= 1, "recognizer reset after the command");
}

void test_forced_final_time(size_t chunk) {
  auto engine = std::make_unique<FakeEngine>();
  FakeEngine* fake = engine.get();
  CommandPipeline pipeline(base_config(), std::move(engine));
  run_silence(pipeline, 48000 * 7, chunk);
  const int64_t block_ms = static_cast<int64_t>(chunk) * 1000 / 48000;
  expect(fake->first_final_ms.has_value() && *fake->first_final_ms >= 5000 &&
             *fake->first_final_ms <= 5000 + block_ms,
         "forced final at 5000 ms within one block (chunk " + std::to_string(chunk) + ")");
  expect(fake->final_calls == 1, "one forced final in 7 s (chunk " + std::to_string(chunk) + ")");
}

void test_forced_final_disabled() {
  auto engine = std::make_unique<FakeEngine>();
  FakeEngine* fake = engine.get();
  PipelineConfig config = base_config();
  config.max_utterance = milliseconds(0);
  CommandPipeline pipeline(config, std::move(engine));
  run_silence(pipeline, 48000 * 7, 4800);
  expect(fake->final_calls == 0, "max_utterance 0 -> no forced final");
}

void test_partial_trigger_off() {
  auto engine = std::make_unique<FakeEngine>();
  FakeEngine* fake = engine.get();
  fake->partials.push_back({100, 500, kForbidPartial});
  fake->finals.push_back({600, 100, kForbidFinal, false});
  PipelineConfig config = base_config();
  config.partial_trigger = false;
  CommandPipeline pipeline(config, std::move(engine));
  const auto events = run_silence(pipeline, 48000 * 2, 4800);
  int commands = 0;
  for (const auto& e : events) {
    if (e.command) {
      ++commands;
      expect(std::string(e.via) == "final", "partial off: command via final");
      expect(e.audio_time == milliseconds(600), "partial off: command at 600 ms");
    }
  }
  expect(commands == 1, "partial off: one command");
  expect(fake->partial_calls == 0, "partial off: partial result never read");
}

void test_asr_block_matches_decimator() {
  PipelineConfig config = base_config();
  config.channels = 2;
  CommandPipeline pipeline(config, std::make_unique<FakeEngine>());

  std::vector<int16_t> stereo(4800 * 2);
  uint32_t state = 7;
  for (auto& s : stereo) {
    state = state * 1664525u + 1013904223u;
    s = static_cast<int16_t>(state >> 16);
  }
  std::vector<PipelineEvent> events;
  pipeline.feed(stereo.data(), 4800, events);

  std::vector<int16_t> mono;
  mix_to_mono(stereo.data(), 4800, 2, mono);
  Decimator decimator(3);
  std::vector<int16_t> expected;
  decimator.process(mono.data(), mono.size(), expected);
  expect(pipeline.last_asr_block().size() == 1600, "4800 stereo frames -> 1600 asr samples");
  expect(pipeline.last_asr_block() == expected, "asr block == Decimator(3)(mix_to_mono)");
}

void test_denoise_fallback() {
  PipelineConfig config = base_config();
  config.denoise = "bogus";
  CommandPipeline pipeline(config, std::make_unique<FakeEngine>());
  expect(std::string(pipeline.denoise_name()) == "none" && !pipeline.denoise_error().empty(),
         "unknown denoise -> none + error");
}

}  // namespace

int main() {
  test_partial_fires_once();
  test_forced_final_time(4800);
  test_forced_final_time(480);
  test_forced_final_disabled();
  test_partial_trigger_off();
  test_asr_block_matches_decimator();
  test_denoise_fallback();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_command_pipeline: ok\n";
  return 0;
}
