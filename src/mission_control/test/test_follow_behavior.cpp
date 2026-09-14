#include <iostream>

#include "mission_control/follow_behavior.hpp"

using mission_control::evaluate_follow_behavior;
using mission_control::kControlAutoFollow;
using mission_control::kControlForbidden;
using mission_control::kControlManual;
using mission_control::kFollowFollowing;
using mission_control::kFollowHold;
using mission_control::kFollowInactive;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

void test_auto_follow_valid_fresh_following() {
  const uint8_t status = evaluate_follow_behavior(true, kControlAutoFollow, true, true);
  expect(status == kFollowFollowing, "AUTO_FOLLOW + valid + fresh -> FOLLOWING");
}

void test_track_change_same_flags_still_following() {
  const uint8_t first = evaluate_follow_behavior(true, kControlAutoFollow, true, true);
  const uint8_t second = evaluate_follow_behavior(true, kControlAutoFollow, true, true);
  expect(first == kFollowFollowing, "first call -> FOLLOWING");
  expect(second == kFollowFollowing, "second call (track change) -> FOLLOWING");
}

void test_auto_follow_invalid_hold() {
  const uint8_t status = evaluate_follow_behavior(true, kControlAutoFollow, false, true);
  expect(status == kFollowHold, "AUTO_FOLLOW + valid=false -> HOLD");
}

void test_auto_follow_stale_hold() {
  const uint8_t status = evaluate_follow_behavior(true, kControlAutoFollow, true, false);
  expect(status == kFollowHold, "AUTO_FOLLOW + fresh=false -> HOLD");
}

void test_manual_inactive_despite_valid_target() {
  const uint8_t status = evaluate_follow_behavior(true, kControlManual, true, true);
  expect(status == kFollowInactive, "MANUAL + valid + fresh -> INACTIVE");
}

void test_forbidden_inactive_despite_valid_target() {
  const uint8_t status = evaluate_follow_behavior(true, kControlForbidden, true, true);
  expect(status == kFollowInactive, "FORBIDDEN + valid + fresh -> INACTIVE");
}

void test_no_control_state_inactive_despite_valid_target() {
  const uint8_t status = evaluate_follow_behavior(false, kControlAutoFollow, true, true);
  expect(status == kFollowInactive, "no control state + valid + fresh -> INACTIVE");
}

}  // namespace

int main() {
  test_auto_follow_valid_fresh_following();
  test_track_change_same_flags_still_following();
  test_auto_follow_invalid_hold();
  test_auto_follow_stale_hold();
  test_manual_inactive_despite_valid_target();
  test_forbidden_inactive_despite_valid_target();
  test_no_control_state_inactive_despite_valid_target();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_follow_behavior: ok\n";
  return 0;
}
