#ifndef MENTORPI_VOICE_PHRASE_MATCH_HPP_
#define MENTORPI_VOICE_PHRASE_MATCH_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mentorpi_voice {

namespace detail {

inline void append_utf8(std::string& out, uint32_t cp) {
  if (cp < 0x80) {
    out.push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp < 0x10000) {
    out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

inline uint32_t decode_utf8(std::string_view text, size_t& i) {
  const auto b0 = static_cast<unsigned char>(text[i]);
  if (b0 < 0x80) {
    ++i;
    return b0;
  }
  if ((b0 & 0xE0) == 0xC0 && i + 1 < text.size()) {
    const auto b1 = static_cast<unsigned char>(text[i + 1]);
    if ((b1 & 0xC0) == 0x80) {
      i += 2;
      return (static_cast<uint32_t>(b0 & 0x1F) << 6) | (b1 & 0x3F);
    }
  }
  if ((b0 & 0xF0) == 0xE0 && i + 2 < text.size()) {
    const auto b1 = static_cast<unsigned char>(text[i + 1]);
    const auto b2 = static_cast<unsigned char>(text[i + 2]);
    if ((b1 & 0xC0) == 0x80 && (b2 & 0xC0) == 0x80) {
      i += 3;
      return (static_cast<uint32_t>(b0 & 0x0F) << 12) | (static_cast<uint32_t>(b1 & 0x3F) << 6) |
             (b2 & 0x3F);
    }
  }
  ++i;
  return b0;
}

inline bool is_ascii_space(uint32_t cp) {
  return cp == ' ' || cp == '\t' || cp == '\n' || cp == '\r';
}

inline uint32_t fold_case(uint32_t cp) {
  if (cp >= 'A' && cp <= 'Z') {
    return cp + 32;
  }
  // CYRILLIC CAPITAL/SMALL IO → CYRILLIC SMALL IE
  if (cp == 0x0401 || cp == 0x0451) {
    return 0x0435;
  }
  // CYRILLIC CAPITAL LETTER A..YA
  if (cp >= 0x0410 && cp <= 0x042F) {
    return cp + 0x20;
  }
  return cp;
}

}  // namespace detail

inline std::string normalize_phrase(std::string_view text) {
  std::string out;
  bool pending_space = false;
  size_t i = 0;
  while (i < text.size()) {
    const uint32_t cp = detail::fold_case(detail::decode_utf8(text, i));
    if (detail::is_ascii_space(cp)) {
      if (!out.empty()) {
        pending_space = true;
      }
      continue;
    }
    if (pending_space) {
      out.push_back(' ');
      pending_space = false;
    }
    detail::append_utf8(out, cp);
  }
  return out;
}

inline std::optional<size_t> match_phrase(std::string_view text,
                                          const std::vector<std::string>& phrases) {
  const std::string normalized = normalize_phrase(text);
  if (normalized.find("[unk]") != std::string::npos) {
    return std::nullopt;
  }
  for (size_t i = 0; i < phrases.size(); ++i) {
    if (normalize_phrase(phrases[i]) == normalized) {
      return i;
    }
  }
  return std::nullopt;
}

inline bool words_confident(const std::vector<double>& word_conf, double min_conf) {
  if (word_conf.empty()) {
    return false;
  }
  for (const double conf : word_conf) {
    if (conf < min_conf) {
      return false;
    }
  }
  return true;
}

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_PHRASE_MATCH_HPP_
