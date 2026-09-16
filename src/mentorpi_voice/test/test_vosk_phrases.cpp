// SD033 D8.1: Vosk + CommandPipeline on recorded fixtures. The WAV files are not in git: the test
// reads them from $MENTORPI_VOICE_TEST_DATA_DIR at run time and is skipped (exit 77) without them.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "mentorpi_voice/command_pipeline.hpp"
#include "mentorpi_voice/wav_pcm.hpp"
#include "vosk_api.h"

using mentorpi_voice::CommandPipeline;
using mentorpi_voice::PipelineConfig;
using mentorpi_voice::PipelineEvent;
using mentorpi_voice::read_wav_pcm16;
using mentorpi_voice::VoskEngine;

namespace {

constexpr int kSkipped = 77;
int g_fails = 0;

void expect(bool cond, const std::string& what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

const std::vector<std::string> kPhrases = {"режим запрет", "режим следование", "режим ручной"};
const std::vector<std::string> kCommands = {"mode_forbid", "mode_follow", "mode_manual"};

std::string join(const std::vector<std::string>& values) {
  std::string out;
  for (const auto& v : values) {
    out += (out.empty() ? "" : " ") + v;
  }
  return out.empty() ? "<none>" : out;
}

std::vector<std::string> recognize(VoskModel* model, const std::string& path) {
  std::vector<std::string> detected;
  const auto clip = read_wav_pcm16(path);
  if (!clip.has_value() || clip->sample_rate % 16000 != 0) {
    expect(false, "cannot read a PCM16 WAV with a rate multiple of 16000: " + path);
    return detected;
  }

  // voice_command start values (SD033 I3), no denoise.
  PipelineConfig config;
  config.capture_rate = clip->sample_rate;
  config.asr_rate = 16000;
  config.channels = clip->channels;
  config.denoise = "none";
  config.partial_trigger = true;
  config.partial_stable = std::chrono::milliseconds(200);
  config.max_utterance = std::chrono::milliseconds(5000);
  config.phrases = kPhrases;
  config.commands = kCommands;
  config.min_confidence = 0.6;

  auto engine = std::make_unique<VoskEngine>(model, 16000.0f, kPhrases);
  if (!engine->ok()) {
    expect(false, "vosk_recognizer_new_grm");
    return detected;
  }
  CommandPipeline pipeline(config, std::move(engine));

  const size_t channels = clip->channels;
  const size_t frames = clip->samples.size() / channels;
  const size_t chunk = clip->sample_rate / 10;
  std::vector<PipelineEvent> events;
  auto collect = [&]() {
    for (const auto& e : events) {
      if (e.command && e.phrase.has_value()) {
        detected.push_back(kCommands[*e.phrase]);
      }
    }
  };
  for (size_t off = 0; off < frames; off += chunk) {
    pipeline.feed(clip->samples.data() + off * channels, std::min(chunk, frames - off), events);
    collect();
  }
  const std::vector<int16_t> silence(chunk * channels, 0);
  for (int i = 0; i < 10; ++i) {
    pipeline.feed(silence.data(), chunk, events);
    collect();
  }
  return detected;
}

}  // namespace

int main() {
  const char* data_dir = std::getenv("MENTORPI_VOICE_TEST_DATA_DIR");
  if (data_dir == nullptr || *data_dir == '\0') {
    std::cout << "test_vosk_phrases: skipped: no fixtures (MENTORPI_VOICE_TEST_DATA_DIR not set)\n";
    return kSkipped;
  }
  const std::vector<std::string> names = {"mode_forbid.wav", "mode_follow.wav", "mode_manual.wav",
                                          "other.wav"};
  for (const auto& name : names) {
    const auto path = std::filesystem::path(data_dir) / name;
    if (!std::filesystem::exists(path)) {
      std::cout << "test_vosk_phrases: skipped: no fixture " << path.string() << '\n';
      return kSkipped;
    }
  }

  const char* model_env = std::getenv("VOSK_MODEL_DIR");
  const std::string model_dir =
      model_env != nullptr && *model_env != '\0' ? model_env : MENTORPI_VOICE_MODEL_DIR;
  vosk_set_log_level(-1);
  VoskModel* model = vosk_model_new(model_dir.c_str());
  if (model == nullptr) {
    std::cerr << "FAIL: vosk_model_new " << model_dir << '\n';
    return 1;
  }

  const std::string dir(data_dir);
  const auto forbid = recognize(model, dir + "/mode_forbid.wav");
  expect(forbid == std::vector<std::string>({"mode_forbid"}), "mode_forbid.wav -> " + join(forbid));
  const auto follow = recognize(model, dir + "/mode_follow.wav");
  expect(follow == std::vector<std::string>({"mode_follow"}), "mode_follow.wav -> " + join(follow));
  const auto manual = recognize(model, dir + "/mode_manual.wav");
  expect(manual == std::vector<std::string>({"mode_manual"}), "mode_manual.wav -> " + join(manual));
  const auto other = recognize(model, dir + "/other.wav");
  expect(other.empty(), "other.wav -> " + join(other));

  vosk_model_free(model);
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_vosk_phrases: ok\n";
  return 0;
}
