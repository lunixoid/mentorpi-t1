#ifndef MENTORPI_VOICE_UTTERANCE_HPP_
#define MENTORPI_VOICE_UTTERANCE_HPP_

#include <algorithm>
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
  std::vector<double> word_conf;
};

struct CommandDecision {
  std::optional<std::string> command;
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
        out.word_conf.push_back(word.value("conf", 0.0));
      }
    }
  } catch (const std::exception&) {
  }
  return out;
}

inline double min_word_conf(const std::vector<double>& word_conf) {
  if (word_conf.empty()) {
    return 0.0;
  }
  return *std::min_element(word_conf.begin(), word_conf.end());
}

inline CommandDecision decide_command(const VoskUtterance& utterance,
                                      const std::vector<std::string>& phrases,
                                      const std::vector<std::string>& commands, double min_conf) {
  CommandDecision out;
  out.text = utterance.text;
  out.conf_min = min_word_conf(utterance.word_conf);
  if (normalize_phrase(utterance.text).empty()) {
    out.ignore_reason = "empty";
    return out;
  }
  const auto idx = match_phrase(utterance.text, phrases);
  if (!idx.has_value() || *idx >= commands.size()) {
    out.ignore_reason = "no match";
    return out;
  }
  if (!words_confident(utterance.word_conf, min_conf)) {
    out.ignore_reason = "low confidence";
    return out;
  }
  out.command = commands[*idx];
  return out;
}

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_UTTERANCE_HPP_
