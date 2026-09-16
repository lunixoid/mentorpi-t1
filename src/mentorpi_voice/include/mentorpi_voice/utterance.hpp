#ifndef MENTORPI_VOICE_UTTERANCE_HPP_
#define MENTORPI_VOICE_UTTERANCE_HPP_

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <exception>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "mentorpi_voice/phrase_match.hpp"

namespace mentorpi_voice {

struct VoskUtterance {
  std::string text;
  std::vector<std::string> words;
  std::vector<double> word_conf;
};

struct CommandDecision {
  std::optional<std::string> command;
  std::optional<size_t> phrase;
  std::string text;
  double conf_min{0.0};
  const char* ignore_reason{""};
};

inline std::string grammar_json(const std::vector<std::string>& phrases) {
  nlohmann::json g = phrases;
  g.push_back("[unk]");
  return g.dump();
}

inline VoskUtterance parse_vosk_result(std::string_view json_text) {
  VoskUtterance out;
  try {
    const auto parsed = nlohmann::json::parse(json_text);
    out.text = parsed.value("text", std::string());
    if (parsed.contains("result") && parsed["result"].is_array()) {
      for (const auto& word : parsed["result"]) {
        out.words.push_back(word.value("word", std::string()));
        out.word_conf.push_back(word.value("conf", 0.0));
      }
    }
  } catch (const std::exception&) {
  }
  return out;
}

// SD033 I6: text of vosk_recognizer_partial_result; broken JSON gives an empty string.
inline std::string parse_vosk_partial(std::string_view json_text) {
  try {
    const auto parsed = nlohmann::json::parse(json_text);
    return parsed.value("partial", std::string());
  } catch (const std::exception&) {
    return std::string();
  }
}

inline double min_word_conf(const std::vector<double>& word_conf) {
  if (word_conf.empty()) {
    return 0.0;
  }
  return *std::min_element(word_conf.begin(), word_conf.end());
}

// SD033 D3.2: the phrase may be surrounded by other words; only the words of the found phrase
// must reach min_conf.
inline CommandDecision decide_command(const VoskUtterance& utterance,
                                      const std::vector<std::string>& phrases,
                                      const std::vector<std::string>& commands, double min_conf) {
  CommandDecision out;
  out.text = utterance.text;
  out.conf_min = min_word_conf(utterance.word_conf);

  std::vector<std::string> words;
  if (!utterance.words.empty()) {
    words.reserve(utterance.words.size());
    for (const auto& word : utterance.words) {
      words.push_back(normalize_phrase(word));
    }
  } else {
    words = split_words(utterance.text);
  }
  if (words.empty()) {
    out.ignore_reason = "empty";
    return out;
  }

  const auto hit = find_phrase(words, phrases);
  if (!hit.has_value() || hit->phrase >= commands.size()) {
    out.ignore_reason = "no match";
    return out;
  }

  std::vector<double> phrase_conf;
  if (utterance.word_conf.size() == words.size()) {
    const auto first = utterance.word_conf.begin() + static_cast<std::ptrdiff_t>(hit->first_word);
    phrase_conf.assign(first, first + static_cast<std::ptrdiff_t>(hit->word_count));
  }
  out.conf_min = min_word_conf(phrase_conf);
  if (!words_confident(phrase_conf, min_conf)) {
    out.ignore_reason = "low confidence";
    return out;
  }
  out.command = commands[hit->phrase];
  out.phrase = hit->phrase;
  return out;
}

// SD033 D3.1: a phrase in the partial result counts once it has held for `stable` audio time.
// Another phrase or a partial result without a phrase restarts the wait.
class PartialTrigger {
 public:
  explicit PartialTrigger(std::chrono::milliseconds stable) : stable_(stable) {}

  std::optional<size_t> on_partial(std::string_view partial_json_text,
                                   const std::vector<std::string>& phrases,
                                   std::chrono::milliseconds audio_time) {
    const auto hit = find_phrase(split_words(parse_vosk_partial(partial_json_text)), phrases);
    if (!hit.has_value()) {
      reset();
      return std::nullopt;
    }
    if (!pending_.has_value() || *pending_ != hit->phrase) {
      pending_ = hit->phrase;
      since_ = audio_time;
      fired_ = false;
    }
    if (fired_ || audio_time - since_ < stable_) {
      return std::nullopt;
    }
    fired_ = true;
    return hit->phrase;
  }

  void reset() {
    pending_.reset();
    fired_ = false;
  }

 private:
  std::chrono::milliseconds stable_;
  std::optional<size_t> pending_;
  std::chrono::milliseconds since_{0};
  bool fired_{false};
};

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_UTTERANCE_HPP_
