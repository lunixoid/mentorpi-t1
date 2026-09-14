#ifndef MENTORPI_PERCEPTION_PERSON_TARGET_LOCK_HPP_
#define MENTORPI_PERCEPTION_PERSON_TARGET_LOCK_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace mentorpi_perception {

struct PersonObservation {
  std::int32_t track_id{-1};
  float x{0.0f};
  float y{0.0f};
  float range{0.0f};
  float confidence{0.0f};
};

struct NearestPersonResult {
  bool valid{false};
  bool coasting{false};
  PersonObservation person;
};

struct PersonTargetLockConfig {
  float switch_margin_m{0.30f};
  double target_lost_s{2.0};
  double challenger_dwell_s{2.0};
  float handover_radius_m{0.7f};
};

struct PersonTargetLockState {
  bool has_lock{false};
  PersonObservation locked;
  double last_seen_s{0.0};
  bool has_challenger{false};
  std::int32_t challenger_id{-1};
  double dwell_start_s{0.0};
};

enum class PersonTargetLockEvent {
  kNone,
  kFirstLock,
  kHandover,
  kChallengerSwitch,
  kLostRelock,
  kLost,
};

struct PersonTargetLockUpdateResult {
  std::vector<PersonObservation> persons;
  NearestPersonResult nearest;
  PersonTargetLockEvent event{PersonTargetLockEvent::kNone};
  std::int32_t previous_track_id{-1};
  float handover_distance_m{0.0f};
  double absent_s{0.0};
};

inline const PersonObservation* find_observation(const std::vector<PersonObservation>& observations,
                                                 std::int32_t track_id) {
  for (const PersonObservation& obs : observations) {
    if (obs.track_id == track_id) {
      return &obs;
    }
  }
  return nullptr;
}

inline const PersonObservation* find_min_range_observation(
    const std::vector<PersonObservation>& observations) {
  if (observations.empty()) {
    return nullptr;
  }
  const PersonObservation* best = &observations.front();
  for (const PersonObservation& obs : observations) {
    if (obs.range < best->range) {
      best = &obs;
    }
  }
  return best;
}

inline void reset_challenger(PersonTargetLockState* state) {
  if (state == nullptr) {
    return;
  }
  state->has_challenger = false;
  state->challenger_id = -1;
  state->dwell_start_s = 0.0;
}

inline void clear_lock(PersonTargetLockState* state) {
  if (state == nullptr) {
    return;
  }
  state->has_lock = false;
  state->locked = PersonObservation{};
  state->last_seen_s = 0.0;
  reset_challenger(state);
}

inline void lock_to_observation(PersonTargetLockState* state, const PersonObservation& obs,
                                double now_s) {
  if (state == nullptr) {
    return;
  }
  state->has_lock = true;
  state->locked = obs;
  state->last_seen_s = now_s;
  reset_challenger(state);
}

inline bool observation_in_list(const std::vector<PersonObservation>& persons,
                                std::int32_t track_id) {
  return find_observation(persons, track_id) != nullptr;
}

inline void append_coast_if_needed(std::vector<PersonObservation>* persons,
                                   const PersonObservation& locked) {
  if (persons == nullptr) {
    return;
  }
  if (!observation_in_list(*persons, locked.track_id)) {
    persons->push_back(locked);
  }
}

inline const PersonObservation* find_best_challenger(
    const std::vector<PersonObservation>& observations, const PersonObservation& locked,
    float switch_margin_m) {
  const PersonObservation* best = nullptr;
  const float max_challenger_range = locked.range - switch_margin_m;
  for (const PersonObservation& obs : observations) {
    if (obs.track_id == locked.track_id) {
      continue;
    }
    if (obs.range > max_challenger_range) {
      continue;
    }
    if (best == nullptr || obs.range < best->range) {
      best = &obs;
    }
  }
  return best;
}

inline const PersonObservation* find_nearest_xy(const std::vector<PersonObservation>& observations,
                                                const PersonObservation& origin,
                                                std::int32_t exclude_id) {
  const PersonObservation* best = nullptr;
  float best_dist = 0.0f;
  for (const PersonObservation& obs : observations) {
    if (obs.track_id == exclude_id) {
      continue;
    }
    const float dist = std::hypot(obs.x - origin.x, obs.y - origin.y);
    if (best == nullptr || dist < best_dist) {
      best = &obs;
      best_dist = dist;
    }
  }
  return best;
}

inline void update_challenger_dwell(PersonTargetLockState* state,
                                    const PersonTargetLockConfig& config, double now_s,
                                    const PersonObservation* challenger) {
  if (state == nullptr) {
    return;
  }
  if (challenger == nullptr) {
    reset_challenger(state);
    return;
  }
  if (!state->has_challenger || challenger->track_id != state->challenger_id) {
    state->has_challenger = true;
    state->challenger_id = challenger->track_id;
    state->dwell_start_s = now_s;
    return;
  }
  if ((now_s - state->dwell_start_s) >= config.challenger_dwell_s) {
    lock_to_observation(state, *challenger, now_s);
  }
}

// D3: nearest lock with hysteresis. Monotonic seconds, no ROS.
inline PersonTargetLockUpdateResult update_person_target_lock(
    PersonTargetLockState& state, const PersonTargetLockConfig& config, double now_s,
    const std::vector<PersonObservation>& observations) {
  PersonTargetLockUpdateResult result;
  result.persons = observations;

  if (!state.has_lock) {
    // D3.1: first lock on min range, no dwell.
    const PersonObservation* pick = find_min_range_observation(observations);
    if (pick == nullptr) {
      result.nearest.valid = false;
      result.nearest.coasting = false;
      return result;
    }
    lock_to_observation(&state, *pick, now_s);
    result.event = PersonTargetLockEvent::kFirstLock;
    result.nearest.valid = true;
    result.nearest.coasting = false;
    result.nearest.person = state.locked;
    return result;
  }

  const PersonObservation* locked_now = find_observation(observations, state.locked.track_id);
  if (locked_now != nullptr) {
    // D3.2: locked present — hysteresis before switching.
    const std::int32_t previous_id = state.locked.track_id;
    state.locked = *locked_now;
    state.last_seen_s = now_s;
    const PersonObservation* challenger =
        find_best_challenger(observations, state.locked, config.switch_margin_m);
    update_challenger_dwell(&state, config, now_s, challenger);
    result.nearest.valid = true;
    result.nearest.coasting = false;
    result.nearest.person = state.locked;
    if (state.locked.track_id != previous_id) {
      result.event = PersonTargetLockEvent::kChallengerSwitch;
      result.previous_track_id = previous_id;
    }
    return result;
  }

  reset_challenger(&state);
  const double absent_s = now_s - state.last_seen_s;
  result.absent_s = absent_s;
  if (absent_s < config.target_lost_s) {
    if (config.handover_radius_m > 0.0f) {
      const PersonObservation* candidate =
          find_nearest_xy(observations, state.locked, state.locked.track_id);
      if (candidate != nullptr) {
        const float dist = std::hypot(candidate->x - state.locked.x, candidate->y - state.locked.y);
        if (dist <= config.handover_radius_m) {
          result.event = PersonTargetLockEvent::kHandover;
          result.previous_track_id = state.locked.track_id;
          result.handover_distance_m = dist;
          lock_to_observation(&state, *candidate, now_s);
          result.nearest.valid = true;
          result.nearest.coasting = false;
          result.nearest.person = state.locked;
          return result;
        }
      }
    }
    // D3.3: coast last measured hypothesis.
    result.nearest.valid = true;
    result.nearest.coasting = true;
    result.nearest.person = state.locked;
    // D3.5: keep published list aligned with nearest.
    append_coast_if_needed(&result.persons, state.locked);
    return result;
  }

  // D3.4: lost long enough — re-lock or clear.
  result.previous_track_id = state.locked.track_id;
  clear_lock(&state);
  const PersonObservation* pick = find_min_range_observation(observations);
  if (pick == nullptr) {
    result.event = PersonTargetLockEvent::kLost;
    result.nearest.valid = false;
    result.nearest.coasting = false;
    return result;
  }
  lock_to_observation(&state, *pick, now_s);
  result.event = PersonTargetLockEvent::kLostRelock;
  result.nearest.valid = true;
  result.nearest.coasting = false;
  result.nearest.person = state.locked;
  return result;
}

}  // namespace mentorpi_perception

#endif  // MENTORPI_PERCEPTION_PERSON_TARGET_LOCK_HPP_
