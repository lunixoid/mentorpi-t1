#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "mentorpi_voice/phrase_match.hpp"

using mentorpi_voice::find_phrase;
using mentorpi_voice::normalize_phrase;
using mentorpi_voice::split_words;
using mentorpi_voice::words_confident;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

const std::vector<std::string> kPhrases = {"режим запрет", "режим следование", "режим ручной"};

void test_split_words() {
  expect(split_words("  Режим   ЗАПРЕТ ") == std::vector<std::string>({"режим", "запрет"}),
         "split_words normalizes and splits");
  expect(split_words("").empty(), "split_words empty");
  expect(split_words("[unk] режим") == std::vector<std::string>({"[unk]", "режим"}),
         "split_words keeps [unk] as a word");
}

void test_find_phrase_hits() {
  const auto with_unk = find_phrase(split_words("[unk] режим запрет"), kPhrases);
  expect(with_unk.has_value() && with_unk->phrase == 0 && with_unk->first_word == 1 &&
             with_unk->word_count == 2,
         "[unk] режим запрет -> phrase 0 at word 1, 2 words");

  const auto tail_unk = find_phrase(split_words("режим запрет [unk] [unk]"), kPhrases);
  expect(tail_unk.has_value() && tail_unk->phrase == 0 && tail_unk->first_word == 0,
         "режим запрет [unk] [unk] -> phrase 0 at word 0");

  const auto two = find_phrase(split_words("режим ручной режим запрет"), kPhrases);
  expect(two.has_value() && two->phrase == 2 && two->first_word == 0,
         "режим ручной режим запрет -> the earliest: режим ручной");

  const auto upper = find_phrase(split_words("РЕЖИМ Следование"), kPhrases);
  expect(upper.has_value() && upper->phrase == 1, "case folded -> режим следование");
}

void test_find_phrase_misses() {
  expect(!find_phrase(split_words("режим [unk] запрет"), kPhrases).has_value(),
         "режим [unk] запрет -> none (gap inside the phrase)");
  expect(!find_phrase(split_words("запрет режим"), kPhrases).has_value(), "запрет режим -> none");
  expect(!find_phrase(split_words("режим"), kPhrases).has_value(), "режим -> none");
  expect(!find_phrase({}, kPhrases).has_value(), "no words -> none");
}

void test_words_confident() {
  expect(words_confident({0.9, 0.7}, 0.6), "{0.9, 0.7} >= 0.6 -> true");
  expect(!words_confident({0.9, 0.5}, 0.6), "{0.9, 0.5} >= 0.6 -> false");
  expect(!words_confident({}, 0.6), "empty -> false");
}

void test_normalize_yo() {
  expect(normalize_phrase("Ёлка") == "елка", "Ё -> е");
  expect(normalize_phrase("слёдование") == "следование", "ё -> е");
}

}  // namespace

int main() {
  test_split_words();
  test_find_phrase_hits();
  test_find_phrase_misses();
  test_words_confident();
  test_normalize_yo();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_phrase_match: ok\n";
  return 0;
}
