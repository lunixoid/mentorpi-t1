#include <iostream>

#include "mentorpi_control/control_mux_gate.hpp"

using mentorpi_control::gate_mux;
using mentorpi_control::kControlAutoFollow;
using mentorpi_control::kControlForbidden;
using mentorpi_control::kControlManual;
using mentorpi_control::Twist2d;
using mentorpi_control::twist2d_is_zero;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

void test_manual_ignores_follow() {
  const Twist2d manual{0.4, -1.2};
  const Twist2d follow{0.9, 0.5};
  const Twist2d out = gate_mux(kControlManual, false, manual, follow);
  expect(out.linear_x == 0.4, "MANUAL passes manual linear");
  expect(out.angular_z == -1.2, "MANUAL passes manual angular");
}

void test_follow_ignores_manual() {
  const Twist2d manual{0.4, -1.2};
  const Twist2d follow{0.9, 0.5};
  const Twist2d out = gate_mux(kControlAutoFollow, false, manual, follow);
  expect(out.linear_x == 0.9, "AUTO_FOLLOW passes follow linear");
  expect(out.angular_z == 0.5, "AUTO_FOLLOW passes follow angular");
}

void test_stop_zeros_both_modes() {
  const Twist2d manual{0.4, -1.2};
  const Twist2d follow{0.9, 0.5};
  expect(twist2d_is_zero(gate_mux(kControlManual, true, manual, follow)), "stop zeros MANUAL");
  expect(twist2d_is_zero(gate_mux(kControlAutoFollow, true, manual, follow)),
         "stop zeros AUTO_FOLLOW");
}

void test_stop_release_keeps_mode() {
  uint8_t state = kControlManual;
  const Twist2d manual{0.4, -1.2};
  const Twist2d follow{0.9, 0.5};

  expect(twist2d_is_zero(gate_mux(state, true, manual, follow)), "stop zeros while MANUAL");
  expect(state == kControlManual, "stop does not change MANUAL");

  const Twist2d resumed = gate_mux(state, false, manual, follow);
  expect(state == kControlManual, "clearing stop does not change MANUAL");
  expect(resumed.linear_x == 0.4, "cleared stop restores manual linear");
  expect(resumed.angular_z == -1.2, "cleared stop restores manual angular");

  state = kControlAutoFollow;
  expect(twist2d_is_zero(gate_mux(state, true, manual, follow)), "stop zeros while AUTO_FOLLOW");
  const Twist2d follow_resumed = gate_mux(state, false, manual, follow);
  expect(state == kControlAutoFollow, "clearing stop does not change AUTO_FOLLOW");
  expect(follow_resumed.linear_x == 0.9, "cleared stop restores follow linear");
  expect(follow_resumed.angular_z == 0.5, "cleared stop restores follow angular");
}

void test_forbidden_zeros() {
  const Twist2d manual{0.4, -1.2};
  const Twist2d follow{0.9, 0.5};
  expect(twist2d_is_zero(gate_mux(kControlForbidden, false, manual, follow)),
         "FORBIDDEN zeros without stop");
  expect(twist2d_is_zero(gate_mux(kControlForbidden, true, manual, follow)),
         "FORBIDDEN stays zero with stop");
}

void test_unknown_state_zeros() {
  const Twist2d manual{0.4, -1.2};
  const Twist2d follow{0.9, 0.5};
  expect(twist2d_is_zero(gate_mux(9, false, manual, follow)), "unknown state zeros");
}

}  // namespace

int main() {
  test_manual_ignores_follow();
  test_follow_ignores_manual();
  test_stop_zeros_both_modes();
  test_stop_release_keeps_mode();
  test_forbidden_zeros();
  test_unknown_state_zeros();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_control_mux: ok\n";
  return 0;
}
