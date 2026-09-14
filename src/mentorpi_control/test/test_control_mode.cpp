#include <iostream>

#include "mentorpi_control/control_mode.hpp"

using mentorpi_control::apply_mode_toggle;
using mentorpi_control::apply_set_control_mode;
using mentorpi_control::control_status_reason;
using mentorpi_control::ControlModeState;
using mentorpi_control::kControlAutoFollow;
using mentorpi_control::kControlForbidden;
using mentorpi_control::kControlManual;
using mentorpi_control::kOperatorHoldReason;
using mentorpi_control::SetControlModeResult;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

void test_boot_default() {
  const ControlModeState mode{};
  expect(mode.state == kControlAutoFollow, "boot default is AUTO_FOLLOW");
  expect(control_status_reason(mode).empty(), "boot reason is empty");
}

void test_toggle_manual_follow() {
  ControlModeState mode{};
  apply_mode_toggle(mode);
  expect(mode.state == kControlManual, "toggle AUTO_FOLLOW -> MANUAL");
  expect(control_status_reason(mode).empty(), "MANUAL reason is empty");

  apply_mode_toggle(mode);
  expect(mode.state == kControlAutoFollow, "toggle MANUAL -> AUTO_FOLLOW");
  expect(control_status_reason(mode).empty(), "AUTO_FOLLOW reason is empty");
}

void test_operator_hold() {
  ControlModeState mode{};
  const SetControlModeResult hold = apply_set_control_mode(mode, kControlForbidden);
  expect(hold.success, "FORBIDDEN service succeeds");
  expect(hold.active_state == kControlForbidden, "active_state is FORBIDDEN");
  expect(hold.reason == kOperatorHoldReason, "service reason is operator");
  expect(mode.state == kControlForbidden, "mode stores FORBIDDEN");
  expect(control_status_reason(mode) == kOperatorHoldReason,
         "status reason is operator after hold");
}

void test_toggle_does_not_leave_forbidden() {
  ControlModeState mode{};
  apply_set_control_mode(mode, kControlForbidden);
  apply_mode_toggle(mode);
  expect(mode.state == kControlForbidden, "toggle does not leave FORBIDDEN");
  expect(control_status_reason(mode) == kOperatorHoldReason, "toggle keeps operator reason");
}

void test_service_auto_follow_leaves_forbidden() {
  ControlModeState mode{};
  apply_set_control_mode(mode, kControlForbidden);
  const SetControlModeResult allow = apply_set_control_mode(mode, kControlAutoFollow);
  expect(allow.success, "AUTO_FOLLOW service succeeds");
  expect(allow.active_state == kControlAutoFollow, "service returns AUTO_FOLLOW");
  expect(allow.reason.empty(), "service reason empty after AUTO_FOLLOW");
  expect(mode.state == kControlAutoFollow, "AUTO_FOLLOW leaves FORBIDDEN");
  expect(control_status_reason(mode).empty(), "status reason empty after allow");
}

void test_service_manual_from_forbidden() {
  ControlModeState mode{};
  apply_set_control_mode(mode, kControlForbidden);
  const SetControlModeResult manual = apply_set_control_mode(mode, kControlManual);
  expect(manual.success, "MANUAL service succeeds from FORBIDDEN");
  expect(manual.active_state == kControlManual, "service returns MANUAL");
  expect(manual.reason.empty(), "MANUAL reason is empty");
  expect(mode.state == kControlManual, "MANUAL leaves FORBIDDEN");
}

void test_invalid_target_rejected() {
  ControlModeState mode{};
  apply_set_control_mode(mode, kControlManual);
  const SetControlModeResult bad = apply_set_control_mode(mode, 9);
  expect(!bad.success, "invalid target_state is rejected");
  expect(bad.active_state == kControlManual, "rejected call keeps MANUAL");
  expect(bad.reason.empty(), "rejected MANUAL keeps empty reason");
  expect(mode.state == kControlManual, "mode unchanged on invalid target");

  apply_set_control_mode(mode, kControlForbidden);
  const SetControlModeResult bad_hold = apply_set_control_mode(mode, 255);
  expect(!bad_hold.success, "invalid target in FORBIDDEN is rejected");
  expect(bad_hold.active_state == kControlForbidden, "rejected hold stays FORBIDDEN");
  expect(bad_hold.reason == kOperatorHoldReason, "rejected hold still reports operator");
}

}  // namespace

int main() {
  test_boot_default();
  test_toggle_manual_follow();
  test_operator_hold();
  test_toggle_does_not_leave_forbidden();
  test_service_auto_follow_leaves_forbidden();
  test_service_manual_from_forbidden();
  test_invalid_target_rejected();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_control_mode: ok\n";
  return 0;
}
