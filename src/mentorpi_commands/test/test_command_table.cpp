#include <iostream>
#include <optional>

#include "mentorpi_commands/command_table.hpp"

using mentorpi_commands::control_mode_for_command;
using mentorpi_commands::kControlAutoFollow;
using mentorpi_commands::kControlForbidden;
using mentorpi_commands::kControlManual;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

void test_known_commands() {
  const auto forbid = control_mode_for_command("mode_forbid");
  expect(forbid.has_value() && *forbid == kControlForbidden, "mode_forbid -> 0");
  const auto follow = control_mode_for_command("mode_follow");
  expect(follow.has_value() && *follow == kControlAutoFollow, "mode_follow -> 2");
  const auto manual = control_mode_for_command("mode_manual");
  expect(manual.has_value() && *manual == kControlManual, "mode_manual -> 1");
}

void test_unknown_commands() {
  expect(!control_mode_for_command("").has_value(), "empty -> none");
  expect(!control_mode_for_command("MODE_FORBID").has_value(), "MODE_FORBID -> none");
  expect(!control_mode_for_command("mode_stop").has_value(), "mode_stop -> none");
}

}  // namespace

int main() {
  test_known_commands();
  test_unknown_commands();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_command_table: ok\n";
  return 0;
}
