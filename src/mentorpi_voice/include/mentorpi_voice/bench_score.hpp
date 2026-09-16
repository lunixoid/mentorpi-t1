#ifndef MENTORPI_VOICE_BENCH_SCORE_HPP_
#define MENTORPI_VOICE_BENCH_SCORE_HPP_

#include <algorithm>
#include <cstddef>
#include <istream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace mentorpi_voice {

// SD033 I11: hits are the longest common subsequence of expected and detected commands, so order
// matters and every command is matched at most once.
struct Score {
  size_t expected{0};
  size_t detected{0};
  size_t hits{0};
  size_t missed{0};
  size_t extra{0};

  Score& operator+=(const Score& other) {
    expected += other.expected;
    detected += other.detected;
    hits += other.hits;
    missed += other.missed;
    extra += other.extra;
    return *this;
  }
};

inline Score score_sequence(const std::vector<std::string>& expected,
                            const std::vector<std::string>& detected) {
  const size_t n = expected.size();
  const size_t m = detected.size();
  std::vector<std::vector<size_t>> lcs(n + 1, std::vector<size_t>(m + 1, 0));
  for (size_t i = 1; i <= n; ++i) {
    for (size_t j = 1; j <= m; ++j) {
      lcs[i][j] = expected[i - 1] == detected[j - 1] ? lcs[i - 1][j - 1] + 1
                                                     : std::max(lcs[i - 1][j], lcs[i][j - 1]);
    }
  }
  Score score;
  score.expected = n;
  score.detected = m;
  score.hits = lcs[n][m];
  score.missed = n - score.hits;
  score.extra = m - score.hits;
  return score;
}

struct LabelRow {
  std::string file;
  std::vector<std::string> commands;
};

namespace detail {

inline std::string trim_spaces(const std::string& text) {
  const size_t first = text.find_first_not_of(" \t");
  if (first == std::string::npos) {
    return std::string();
  }
  const size_t last = text.find_last_not_of(" \t");
  return text.substr(first, last - first + 1);
}

}  // namespace detail

// `file<TAB>command command ...`; `#` starts a comment line; an empty command list is allowed.
inline std::optional<std::vector<LabelRow>> parse_labels(std::istream& in, std::string& error) {
  error.clear();
  std::vector<LabelRow> rows;
  std::string line;
  size_t number = 0;
  while (std::getline(in, line)) {
    ++number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const size_t first = line.find_first_not_of(" \t");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    const size_t tab = line.find('\t');
    if (tab == std::string::npos) {
      error = "line " + std::to_string(number) + ": no TAB between file and commands";
      return std::nullopt;
    }
    LabelRow row;
    row.file = detail::trim_spaces(line.substr(0, tab));
    if (row.file.empty()) {
      error = "line " + std::to_string(number) + ": empty file";
      return std::nullopt;
    }
    std::istringstream words(line.substr(tab + 1));
    std::string word;
    while (words >> word) {
      row.commands.push_back(word);
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_BENCH_SCORE_HPP_
