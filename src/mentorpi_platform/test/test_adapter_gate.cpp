#include <chrono>
#include <iostream>

#include "mentorpi_platform/adapter_gate.hpp"
#include "mentorpi_platform/publish_cadence.hpp"

using mentorpi_platform::AdapterOutput;
using mentorpi_platform::gate_adapter;
using mentorpi_platform::timer_keepalive_due;
using mentorpi_platform::Twist2d;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

}  // namespace

int main() {
  const Twist2d cmd{0.4, -1.5};

  const AdapterOutput live = gate_adapter(true, false, false, false, cmd);
  expect(!live.command_timeout, "fresh cmd is not timeout");
  expect(!live.forbidden, "no state is not Forbidden");
  expect(live.chassis.linear_x == 0.4, "fresh cmd passes linear");
  expect(live.chassis.angular_z == -1.5, "fresh cmd passes angular");

  const AdapterOutput watchdog = gate_adapter(true, true, false, false, cmd);
  expect(watchdog.command_timeout, "stale cmd sets command_timeout");
  expect(watchdog.chassis.linear_x == 0.0, "watchdog zeros linear");
  expect(watchdog.chassis.angular_z == 0.0, "watchdog zeros angular");

  const AdapterOutput missing = gate_adapter(false, false, false, false, cmd);
  expect(missing.command_timeout, "no cmd is timeout");
  expect(missing.chassis.linear_x == 0.0, "no cmd zeros chassis");

  const AdapterOutput forbidden = gate_adapter(true, false, true, true, cmd);
  expect(forbidden.forbidden, "Forbidden flag");
  expect(!forbidden.command_timeout, "Forbidden can still have a fresh cmd");
  expect(forbidden.chassis.linear_x == 0.0, "Forbidden zeros linear");
  expect(forbidden.chassis.angular_z == 0.0, "Forbidden zeros angular");

  // Adapter does not pick MANUAL vs AUTO_FOLLOW; mux already chose the source.
  const AdapterOutput follow = gate_adapter(true, false, true, false, cmd);
  expect(!follow.forbidden, "AUTO_FOLLOW is not Forbidden");
  expect(follow.chassis.linear_x == 0.4, "non-Forbidden state passes cmd");

  using clock = std::chrono::steady_clock;
  using ms = std::chrono::milliseconds;
  const auto t0 = clock::time_point{};
  const auto period = ms{50};
  expect(timer_keepalive_due(false, t0, t0, period), "first timer publishes before any event");
  expect(!timer_keepalive_due(true, t0 + ms{10}, t0, period),
         "timer skipped within period after event");
  expect(timer_keepalive_due(true, t0 + ms{50}, t0, period), "timer keepalive after one period");
  expect(timer_keepalive_due(true, t0 + ms{51}, t0, period), "timer keepalive past period");

  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_adapter_gate: ok\n";
  return 0;
}
