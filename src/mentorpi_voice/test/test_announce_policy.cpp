#include <chrono>
#include <iostream>
#include <optional>

#include "mentorpi_voice/announce_policy.hpp"

using mentorpi_voice::AnnouncePolicy;
using mentorpi_voice::MuteWindow;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

const auto kT0 = Clock::time_point(Ms(0));
const Ms kAnnounceTimeout{1000};
const Ms kPadWindow{1000};
const Ms kMuteTail{300};

void test_voice_wait_then_state() {
  AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
  expect(!p.on_state(2, kT0).has_value(), "seed state 2 -> none");
  p.on_voice_command(0, kT0 + Ms(10));
  const auto got = p.on_state(0, kT0 + Ms(210));
  expect(got.has_value() && *got == 0, "voice wait 0, on_state(0) at +200ms -> 0");
}

void test_voice_expire_then_late_state() {
  AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
  p.on_state(2, kT0);
  p.on_voice_command(0, kT0);
  const auto missed = p.expire(kT0 + kAnnounceTimeout);
  expect(missed.has_value() && *missed == 0, "expire after timeout -> 0");
  expect(!p.on_state(0, kT0 + kAnnounceTimeout + Ms(1)).has_value(),
         "on_state(0) after expire -> none");
}

void test_voice_already_on() {
  {
    AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
    p.on_voice_command(2, kT0);
    const auto got = p.on_state(2, kT0);
    expect(got.has_value() && *got == 2, "no prior state, first on_state(2) -> 2");
  }
  {
    AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
    expect(!p.on_state(2, kT0).has_value(), "seed already-on 2");
    p.on_voice_command(2, kT0 + Ms(1));
    const auto got = p.on_state(2, kT0 + Ms(2));
    expect(got.has_value() && *got == 2, "command for current mode, on_state(2) -> 2");
    expect(!p.on_state(2, kT0 + Ms(3)).has_value(), "repeat on_state(2) -> none");
  }
}

void test_pad_change_in_window() {
  AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
  p.on_state(2, kT0);
  p.on_pad_toggle(kT0 + Ms(10));
  const auto got = p.on_state(1, kT0 + Ms(210));
  expect(got.has_value() && *got == 1, "pad toggle then 2 -> 1 in window -> 1");
}

void test_pad_no_change() {
  AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
  p.on_state(0, kT0);
  p.on_pad_toggle(kT0 + Ms(10));
  expect(!p.on_state(0, kT0 + Ms(210)).has_value(), "pad toggle without change -> none");
  expect(!p.expire(kT0 + Ms(10) + kPadWindow).has_value(), "pad window expire -> none");
}

void test_state_change_without_trigger() {
  AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
  p.on_state(2, kT0);
  expect(!p.on_state(1, kT0 + Ms(50)).has_value(), "mode change without pad/voice -> none");
}

void test_first_on_state_silent() {
  AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
  expect(!p.on_state(2, kT0).has_value(), "first on_state after create -> none");
}

void test_newer_wait_replaces() {
  AnnouncePolicy p(kAnnounceTimeout, kPadWindow);
  p.on_voice_command(0, kT0);
  p.on_voice_command(1, kT0 + Ms(10));
  expect(!p.on_state(0, kT0 + Ms(20)).has_value(), "old voice wait replaced -> none");
  const auto got = p.on_state(1, kT0 + Ms(30));
  expect(got.has_value() && *got == 1, "new voice wait 1 -> 1");
}

void test_mute_window() {
  MuteWindow w(kMuteTail);
  expect(!w.muted(kT0), "before playback -> not muted");
  const auto start = kT0 + Ms(100);
  const auto end = kT0 + Ms(400);
  w.on_playback(start, end);
  expect(!w.muted(start - Ms(1)), "before start -> not muted");
  expect(w.muted(start), "at start -> muted");
  expect(w.muted(end), "at end -> muted");
  expect(w.muted(end + kMuteTail), "at end+tail -> muted");
  expect(!w.muted(end + kMuteTail + Ms(1)), "after end+tail -> not muted");
}

}  // namespace

int main() {
  test_voice_wait_then_state();
  test_voice_expire_then_late_state();
  test_voice_already_on();
  test_pad_change_in_window();
  test_pad_no_change();
  test_state_change_without_trigger();
  test_first_on_state_silent();
  test_newer_wait_replaces();
  test_mute_window();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_announce_policy: ok\n";
  return 0;
}
