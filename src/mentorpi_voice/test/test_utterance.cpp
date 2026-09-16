#include <chrono>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "mentorpi_voice/utterance.hpp"

using mentorpi_voice::CommandDecision;
using mentorpi_voice::decide_command;
using mentorpi_voice::parse_vosk_partial;
using mentorpi_voice::parse_vosk_result;
using mentorpi_voice::PartialTrigger;
using mentorpi_voice::VoskUtterance;
using std::chrono::milliseconds;

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

std::string partial_json(const char* text) {
  return std::string("{\"partial\" : \"") + text + "\"}";
}

void test_parse() {
  const VoskUtterance u = parse_vosk_result(
      R"({"result":[{"conf":0.2,"word":"[unk]"},{"conf":0.9,"word":"режим"}],"text":"[unk] режим"})");
  expect(u.text == "[unk] режим", "parse text");
  expect(u.words == std::vector<std::string>({"[unk]", "режим"}), "parse words");
  expect(u.word_conf == std::vector<double>({0.2, 0.9}), "parse conf");
  expect(parse_vosk_result("not json").text.empty(), "broken result -> empty");

  expect(parse_vosk_partial(R"({"partial" : "режим запрет"})") == "режим запрет", "parse partial");
  expect(parse_vosk_partial("{").empty(), "broken partial -> empty");
  expect(parse_vosk_partial("[1,2]").empty(), "partial not an object -> empty");
}

void test_decide_command() {
  VoskUtterance u;
  u.text = "[unk] режим запрет";
  u.words = {"[unk]", "режим", "запрет"};
  u.word_conf = {0.2, 0.9, 0.8};
  const CommandDecision ok = decide_command(u, kPhrases, kCommands, 0.6);
  expect(ok.command.has_value() && *ok.command == "mode_forbid", "[unk] ignored for confidence");
  expect(ok.phrase.has_value() && *ok.phrase == 0, "phrase index 0");
  expect(ok.conf_min > 0.79 && ok.conf_min < 0.81, "conf_min over phrase words = 0.8");

  u.word_conf = {0.2, 0.9, 0.5};
  const CommandDecision low = decide_command(u, kPhrases, kCommands, 0.6);
  expect(!low.command.has_value() && std::string(low.ignore_reason) == "low confidence",
         "phrase word 0.5 < 0.6 -> low confidence");

  VoskUtterance unk;
  unk.text = "[unk]";
  unk.words = {"[unk]"};
  unk.word_conf = {1.0};
  const CommandDecision none = decide_command(unk, kPhrases, kCommands, 0.6);
  expect(!none.command.has_value() && std::string(none.ignore_reason) == "no match",
         "[unk] -> no match");

  const CommandDecision empty = decide_command(VoskUtterance{}, kPhrases, kCommands, 0.6);
  expect(std::string(empty.ignore_reason) == "empty", "empty utterance -> empty");
}

void test_partial_trigger_stable() {
  PartialTrigger trigger(milliseconds(200));
  const std::string forbid = partial_json("режим запрет");
  expect(!trigger.on_partial(forbid, kPhrases, milliseconds(0)).has_value(), "0 ms -> nothing");
  expect(!trigger.on_partial(forbid, kPhrases, milliseconds(150)).has_value(), "150 ms -> nothing");
  const auto fired = trigger.on_partial(forbid, kPhrases, milliseconds(200));
  expect(fired.has_value() && *fired == 0, "200 ms -> phrase 0");
  expect(!trigger.on_partial(forbid, kPhrases, milliseconds(250)).has_value(), "fires only once");
}

void test_partial_trigger_restart() {
  PartialTrigger changed(milliseconds(200));
  changed.on_partial(partial_json("режим запрет"), kPhrases, milliseconds(0));
  changed.on_partial(partial_json("режим следование"), kPhrases, milliseconds(100));
  expect(!changed.on_partial(partial_json("режим следование"), kPhrases, milliseconds(250))
              .has_value(),
         "phrase changed at 100 ms -> nothing at 250 ms");
  const auto follow =
      changed.on_partial(partial_json("режим следование"), kPhrases, milliseconds(300));
  expect(follow.has_value() && *follow == 1, "phrase changed at 100 ms -> phrase 1 at 300 ms");

  PartialTrigger emptied(milliseconds(200));
  emptied.on_partial(partial_json("режим запрет"), kPhrases, milliseconds(0));
  emptied.on_partial(partial_json(""), kPhrases, milliseconds(100));
  expect(!emptied.on_partial(partial_json("режим запрет"), kPhrases, milliseconds(200)).has_value(),
         "empty partial at 100 ms restarts the wait");
  expect(emptied.on_partial(partial_json("режим запрет"), kPhrases, milliseconds(400)).has_value(),
         "after restart fires 200 ms later");

  PartialTrigger cleared(milliseconds(200));
  cleared.on_partial(partial_json("[unk] режим ручной"), kPhrases, milliseconds(0));
  cleared.reset();
  expect(!cleared.on_partial(partial_json("[unk] режим ручной"), kPhrases, milliseconds(200))
              .has_value(),
         "reset restarts the wait");
}

}  // namespace

int main() {
  test_parse();
  test_decide_command();
  test_partial_trigger_stable();
  test_partial_trigger_restart();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_utterance: ok\n";
  return 0;
}
