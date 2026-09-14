#ifndef MENTORPI_VOICE_ANNOUNCE_POLICY_HPP_
#define MENTORPI_VOICE_ANNOUNCE_POLICY_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace mentorpi_voice {

class AnnouncePolicy {
 public:
  using time_point = std::chrono::steady_clock::time_point;

  AnnouncePolicy(std::chrono::milliseconds announce_timeout,
                 std::chrono::milliseconds pad_announce_window)
      : announce_timeout_(announce_timeout), pad_announce_window_(pad_announce_window) {}

  void on_voice_command(uint8_t expected_state, time_point now) {
    wait_ = Wait::Voice;
    expected_state_ = expected_state;
    deadline_ = now + announce_timeout_;
  }

  void on_pad_toggle(time_point now) {
    wait_ = Wait::Pad;
    deadline_ = now + pad_announce_window_;
  }

  std::optional<uint8_t> on_state(uint8_t state, time_point now) {
    const bool had_state = have_state_;
    const uint8_t previous = last_state_;
    have_state_ = true;
    last_state_ = state;

    if (wait_ == Wait::None || now >= deadline_) {
      return std::nullopt;
    }
    if (wait_ == Wait::Voice) {
      if (state == expected_state_) {
        wait_ = Wait::None;
        return state;
      }
      return std::nullopt;
    }
    if (had_state && state != previous) {
      wait_ = Wait::None;
      return state;
    }
    return std::nullopt;
  }

  std::optional<uint8_t> expire(time_point now) {
    if (wait_ == Wait::None || now < deadline_) {
      return std::nullopt;
    }
    if (wait_ == Wait::Voice) {
      const uint8_t missed = expected_state_;
      wait_ = Wait::None;
      return missed;
    }
    wait_ = Wait::None;
    return std::nullopt;
  }

 private:
  enum class Wait { None, Voice, Pad };

  std::chrono::milliseconds announce_timeout_;
  std::chrono::milliseconds pad_announce_window_;
  Wait wait_{Wait::None};
  uint8_t expected_state_{0};
  time_point deadline_{};
  bool have_state_{false};
  uint8_t last_state_{0};
};

class MuteWindow {
 public:
  using time_point = std::chrono::steady_clock::time_point;

  explicit MuteWindow(std::chrono::milliseconds mute_tail) : mute_tail_(mute_tail) {}

  void on_playback(time_point start, time_point end) {
    start_ = start;
    mute_until_ = end + mute_tail_;
    armed_ = true;
  }

  bool muted(time_point now) const { return armed_ && now >= start_ && now <= mute_until_; }

 private:
  std::chrono::milliseconds mute_tail_;
  time_point start_{};
  time_point mute_until_{};
  bool armed_{false};
};

}  // namespace mentorpi_voice

#endif  // MENTORPI_VOICE_ANNOUNCE_POLICY_HPP_
