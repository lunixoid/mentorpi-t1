#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "mentorpi_voice/phrase_match.hpp"

using mentorpi_voice::match_phrase;
using mentorpi_voice::normalize_phrase;
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

void test_match_known_phrases() {
  const auto forbid = match_phrase("режим запрет", kPhrases);
  expect(forbid.has_value() && *forbid == 0, "режим запрет -> 0");
  const auto spaced = match_phrase("  Режим   ЗАПРЕТ ", kPhrases);
  expect(spaced.has_value() && *spaced == 0, "  Режим   ЗАПРЕТ  -> 0");
  const auto follow = match_phrase("режим следование", kPhrases);
  expect(follow.has_value() && *follow == 1, "режим следование -> 1");
}

void test_match_rejects_partial_and_unk() {
  expect(!match_phrase("режим", kPhrases).has_value(), "режим -> none");
  expect(!match_phrase("запрет", kPhrases).has_value(), "запрет -> none");
  expect(!match_phrase("режим [unk]", kPhrases).has_value(), "режим [unk] -> none");
  expect(!match_phrase("режим запрет потом", kPhrases).has_value(), "режим запрет потом -> none");
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
  test_match_known_phrases();
  test_match_rejects_partial_and_unk();
  test_words_confident();
  test_normalize_yo();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_phrase_match: ok\n";
  return 0;
}
