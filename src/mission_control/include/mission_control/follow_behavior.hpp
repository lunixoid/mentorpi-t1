#ifndef MISSION_CONTROL_FOLLOW_BEHAVIOR_HPP_
#define MISSION_CONTROL_FOLLOW_BEHAVIOR_HPP_

#include <cstdint>

namespace mission_control {

// Wire values match mentorpi_msgs/msg/FollowPersonStatus.
constexpr uint8_t kFollowInactive = 0;
constexpr uint8_t kFollowHold = 1;
constexpr uint8_t kFollowFollowing = 2;

// Wire values match mentorpi_msgs/msg/ControlState (distinct from follow status).
constexpr uint8_t kControlForbidden = 0;
constexpr uint8_t kControlManual = 1;
constexpr uint8_t kControlAutoFollow = 2;

// Pure decision: no ROS, no track_id, no hysteresis.
inline uint8_t evaluate_follow_behavior(bool have_control_state, uint8_t control_state,
                                        bool nearest_valid, bool nearest_fresh) {
  if (!have_control_state || control_state != kControlAutoFollow) {
    return kFollowInactive;
  }
  if (nearest_valid && nearest_fresh) {
    return kFollowFollowing;
  }
  return kFollowHold;
}

}  // namespace mission_control

#endif  // MISSION_CONTROL_FOLLOW_BEHAVIOR_HPP_
