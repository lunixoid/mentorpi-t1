#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "mentorpi_voice/bench_score.hpp"

using mentorpi_voice::parse_labels;
using mentorpi_voice::Score;
using mentorpi_voice::score_sequence;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

void test_score() {
  const Score s = score_sequence({"F", "Fo", "M"}, {"F", "M", "M"});
  expect(s.expected == 3 && s.detected == 3, "counts");
  expect(s.hits == 2 && s.missed == 1 && s.extra == 1,
         "F Fo M vs F M M -> hits 2, missed 1, extra 1");

  const Score only_extra = score_sequence({}, {"F"});
  expect(only_extra.hits == 0 && only_extra.missed == 0 && only_extra.extra == 1,
         "empty expected, detected F -> extra 1");

  const Score order = score_sequence({"F", "M"}, {"M", "F"});
  expect(order.hits == 1 && order.missed == 1 && order.extra == 1, "order matters");

  Score total = s;
  total += only_extra;
  expect(total.expected == 3 && total.detected == 4 && total.extra == 2, "operator+=");
}

void test_parse_labels() {
  std::istringstream good(
      "# comment\n"
      "s1/a_raw.wav\tmode_forbid mode_follow\r\n"
      "\n"
      "s4/b_raw.wav\t\n");
  std::string error;
  const auto rows = parse_labels(good, error);
  expect(rows.has_value() && error.empty(), "good labels parse");
  if (rows.has_value()) {
    expect(rows->size() == 2, "two rows, comment and blank skipped");
    expect((*rows)[0].file == "s1/a_raw.wav" &&
               (*rows)[0].commands == std::vector<std::string>({"mode_forbid", "mode_follow"}),
           "row 1 with CRLF");
    expect((*rows)[1].file == "s4/b_raw.wav" && (*rows)[1].commands.empty(),
           "row 2 with empty command list");
  }

  std::istringstream bad("# comment\ns1/a_raw.wav\tmode_forbid\ns2/b_raw.wav mode_follow\n");
  const auto broken = parse_labels(bad, error);
  expect(!broken.has_value(), "line without TAB -> nullopt");
  expect(error.find("line 3") != std::string::npos, "error names line 3");
}

}  // namespace

int main() {
  test_score();
  test_parse_labels();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_bench_score: ok\n";
  return 0;
}
