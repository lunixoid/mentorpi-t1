#include <algorithm>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "mentorpi_voice/utterance.hpp"
#include "mentorpi_voice/wav_pcm.hpp"
#include "vosk_api.h"

using mentorpi_voice::CommandDecision;
using mentorpi_voice::decide_command;
using mentorpi_voice::grammar_json;
using mentorpi_voice::parse_vosk_result;
using mentorpi_voice::read_wav_pcm16;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

const std::vector<std::string> kPhrases = {"режим запрет", "режим следование", "режим ручной"};
const std::vector<std::string> kCommands = {"mode_forbid", "mode_follow", "mode_manual"};
constexpr double kMinConf = 0.6;

void consider(std::optional<std::string>& heard, const CommandDecision& decision) {
  if (decision.command.has_value()) {
    heard = decision.command;
  }
}

std::optional<std::string> recognize_wav(VoskModel* model, const std::string& path) {
  const auto clip = read_wav_pcm16(path);
  if (!clip.has_value()) {
    std::cerr << "FAIL: cannot read " << path << '\n';
    ++g_fails;
    return std::nullopt;
  }
  if (clip->sample_rate != 16000 || clip->channels != 1) {
    std::cerr << "FAIL: " << path << " is not 16 kHz mono\n";
    ++g_fails;
    return std::nullopt;
  }

  const std::string grammar = grammar_json(kPhrases);
  VoskRecognizer* rec = vosk_recognizer_new_grm(model, 16000.0f, grammar.c_str());
  if (rec == nullptr) {
    std::cerr << "FAIL: vosk_recognizer_new_grm\n";
    ++g_fails;
    return std::nullopt;
  }
  vosk_recognizer_set_words(rec, 1);

  std::optional<std::string> heard;
  constexpr int kChunk = 4000;
  const auto& samples = clip->samples;
  for (size_t off = 0; off < samples.size();) {
    const int n = static_cast<int>(std::min(static_cast<size_t>(kChunk), samples.size() - off));
    const int rc = vosk_recognizer_accept_waveform_s(rec, samples.data() + off, n);
    if (rc == 1) {
      consider(heard, decide_command(parse_vosk_result(vosk_recognizer_result(rec)), kPhrases,
                                     kCommands, kMinConf));
    }
    off += static_cast<size_t>(n);
  }

  const std::vector<int16_t> silence(8000, 0);
  if (vosk_recognizer_accept_waveform_s(rec, silence.data(), static_cast<int>(silence.size())) ==
      1) {
    consider(heard, decide_command(parse_vosk_result(vosk_recognizer_result(rec)), kPhrases,
                                   kCommands, kMinConf));
  }
  consider(heard, decide_command(parse_vosk_result(vosk_recognizer_final_result(rec)), kPhrases,
                                 kCommands, kMinConf));
  vosk_recognizer_free(rec);
  return heard;
}

std::string data_path(const char* name) {
  return std::string(MENTORPI_VOICE_TEST_DATA_DIR) + "/" + name;
}

}  // namespace

int main() {
  vosk_set_log_level(0);
  VoskModel* model = vosk_model_new(MENTORPI_VOICE_MODEL_DIR);
  if (model == nullptr) {
    std::cerr << "FAIL: vosk_model_new " << MENTORPI_VOICE_MODEL_DIR << '\n';
    return 1;
  }

  const auto forbid = recognize_wav(model, data_path("mode_forbid.wav"));
  expect(forbid.has_value() && *forbid == "mode_forbid", "mode_forbid.wav -> mode_forbid");
  if (!forbid.has_value() || *forbid != "mode_forbid") {
    std::cerr << "  got " << (forbid ? *forbid : std::string("<none>")) << '\n';
  }

  const auto follow = recognize_wav(model, data_path("mode_follow.wav"));
  expect(follow.has_value() && *follow == "mode_follow", "mode_follow.wav -> mode_follow");
  if (!follow.has_value() || *follow != "mode_follow") {
    std::cerr << "  got " << (follow ? *follow : std::string("<none>")) << '\n';
  }

  const auto manual = recognize_wav(model, data_path("mode_manual.wav"));
  expect(manual.has_value() && *manual == "mode_manual", "mode_manual.wav -> mode_manual");
  if (!manual.has_value() || *manual != "mode_manual") {
    std::cerr << "  got " << (manual ? *manual : std::string("<none>")) << '\n';
  }

  const auto other = recognize_wav(model, data_path("other.wav"));
  expect(!other.has_value(), "other.wav -> none");
  if (other.has_value()) {
    std::cerr << "  got " << *other << '\n';
  }

  vosk_model_free(model);
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_vosk_phrases: ok\n";
  return 0;
}
