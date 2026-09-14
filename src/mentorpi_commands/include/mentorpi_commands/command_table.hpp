#ifndef MENTORPI_COMMANDS_COMMAND_TABLE_HPP_
#define MENTORPI_COMMANDS_COMMAND_TABLE_HPP_

#include <cstdint>
#include <optional>
#include <string_view>

namespace mentorpi_commands {

// Wire values match mentorpi_msgs/msg/ControlState.
constexpr uint8_t kControlForbidden = 0;
constexpr uint8_t kControlManual = 1;
constexpr uint8_t kControlAutoFollow = 2;

inline std::optional<uint8_t> control_mode_for_command(std::string_view name) {
  if (name == "mode_forbid") {
    return kControlForbidden;
  }
  if (name == "mode_follow") {
    return kControlAutoFollow;
  }
  if (name == "mode_manual") {
    return kControlManual;
  }
  return std::nullopt;
}

}  // namespace mentorpi_commands

#endif  // MENTORPI_COMMANDS_COMMAND_TABLE_HPP_
