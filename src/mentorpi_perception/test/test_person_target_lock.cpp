// T2: person_target_lock — no rclcpp, monotonic seconds, header-only.
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "mentorpi_perception/person_target_lock.hpp"

using mentorpi_perception::PersonObservation;
using mentorpi_perception::PersonTargetLockConfig;
using mentorpi_perception::PersonTargetLockEvent;
using mentorpi_perception::PersonTargetLockState;
using mentorpi_perception::PersonTargetLockUpdateResult;
using mentorpi_perception::update_person_target_lock;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

bool near(float a, float b) { return std::fabs(a - b) < 1.0e-5f; }
bool near(double a, double b) { return std::fabs(a - b) < 1.0e-5; }

PersonObservation make_obs(std::int32_t track_id, float x, float y, float range,
                           float confidence = 0.9f) {
  PersonObservation obs;
  obs.track_id = track_id;
  obs.x = x;
  obs.y = y;
  obs.range = range;
  obs.confidence = confidence;
  return obs;
}

PersonTargetLockUpdateResult step(PersonTargetLockState* state,
                                  const PersonTargetLockConfig& config, double now_s,
                                  const std::vector<PersonObservation>& observations) {
  return update_person_target_lock(*state, config, now_s, observations);
}

// D3.1: first appearance locks min range immediately.
void test_first_lock_min_range() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  const std::vector<PersonObservation> obs = {
      make_obs(1, 2.0f, 0.0f, 2.0f),
      make_obs(2, 0.8f, 0.6f, 1.0f),
  };
  const PersonTargetLockUpdateResult result = step(&state, config, 0.0, obs);
  expect(result.nearest.valid, "first lock valid");
  expect(!result.nearest.coasting, "first lock not coasting");
  expect(result.nearest.person.track_id == 2, "first lock min range id");
  expect(near(result.nearest.person.range, 1.0f), "first lock min range");
  expect(result.event == PersonTargetLockEvent::kFirstLock, "first lock event");
  expect(result.previous_track_id == -1, "first lock previous id");
}

// AC1: margin < 0.30 m — nearest id unchanged.
void test_ac1_margin_too_small() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  const std::vector<PersonObservation> t0 = {
      make_obs(1, 1.0f, 0.0f, 1.0f),
      make_obs(2, 0.8f, 0.6f, 1.2f),
  };
  step(&state, config, 0.0, t0);
  expect(state.locked.track_id == 1, "AC1 initial lock id=1");

  const std::vector<PersonObservation> t1 = {
      make_obs(1, 1.0f, 0.0f, 1.0f),
      make_obs(2, 0.75f, 0.66f, 0.75f),  // only 0.25 m closer
  };
  const PersonTargetLockUpdateResult result = step(&state, config, 3.0, t1);
  expect(result.nearest.person.track_id == 1, "AC1 margin < 0.30 m keeps id");
  expect(!result.nearest.coasting, "AC1 margin observed not coasting");
  expect(!state.has_challenger, "AC1 margin too small resets challenger");
  expect(result.event == PersonTargetLockEvent::kNone, "AC1 no lock event");
}

// AC1: dwell < 2 s — nearest id unchanged.
void test_ac1_dwell_too_short() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 1.0f, 0.0f, 1.0f)});
  expect(state.locked.track_id == 1, "AC1 dwell initial lock id=1");

  const std::vector<PersonObservation> challenger = {
      make_obs(1, 1.0f, 0.0f, 1.0f),
      make_obs(2, 0.4f, 0.0f, 0.4f),  // 0.60 m closer
  };
  step(&state, config, 0.0, challenger);
  expect(state.has_challenger, "AC1 dwell challenger armed");
  expect(state.challenger_id == 2, "AC1 dwell challenger id");

  const PersonTargetLockUpdateResult result = step(&state, config, 1.9, challenger);
  expect(result.nearest.person.track_id == 1, "AC1 dwell < 2 s keeps id");
  expect(!result.nearest.coasting, "AC1 dwell observed not coasting");
}

// AC2: challenger ≥ 0.30 m closer for 2 s — switch.
void test_ac2_switch_after_dwell() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 1.0f, 0.0f, 1.0f)});

  const std::vector<PersonObservation> challenger = {
      make_obs(1, 1.0f, 0.0f, 1.0f),
      make_obs(2, 0.4f, 0.0f, 0.4f),
  };
  step(&state, config, 0.0, challenger);
  const PersonTargetLockUpdateResult at_dwell = step(&state, config, 2.0, challenger);
  expect(at_dwell.nearest.person.track_id == 2, "AC2 switches after 2 s dwell");
  expect(!at_dwell.nearest.coasting, "AC2 switch observed not coasting");
  expect(near(at_dwell.nearest.person.range, 0.4f), "AC2 nearest range updated");
  expect(at_dwell.event == PersonTargetLockEvent::kChallengerSwitch, "AC2 challenger event");
  expect(at_dwell.previous_track_id == 1, "AC2 previous id");
}

// Challenger change resets dwell timer.
void test_challenger_change_resets_dwell() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 2.0f, 0.0f, 2.0f)});

  const std::vector<PersonObservation> id2 = {
      make_obs(1, 2.0f, 0.0f, 2.0f),
      make_obs(2, 0.5f, 0.0f, 0.5f),
  };
  step(&state, config, 0.0, id2);
  expect(state.challenger_id == 2, "challenger id2 armed");

  const std::vector<PersonObservation> id3 = {
      make_obs(1, 2.0f, 0.0f, 2.0f),
      make_obs(3, 0.4f, 0.0f, 0.4f),
  };
  step(&state, config, 1.5, id3);
  expect(state.challenger_id == 3, "challenger switched to id3");
  expect(state.dwell_start_s == 1.5, "challenger dwell reset on id change");

  const PersonTargetLockUpdateResult before = step(&state, config, 3.4, id3);
  expect(before.nearest.person.track_id == 1, "dwell not met after challenger swap");

  const PersonTargetLockUpdateResult after = step(&state, config, 3.5, id3);
  expect(after.nearest.person.track_id == 3, "switch after full dwell for id3");
  expect(after.event == PersonTargetLockEvent::kChallengerSwitch, "id3 challenger event");
  expect(after.previous_track_id == 1, "id3 previous id");
}

// AC3: absent 1.9 s — old id and last range; at 2.0 s — other or empty.
void test_ac3_coast_then_relock() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  const std::vector<PersonObservation> locked_only = {
      make_obs(1, 1.2f, 0.3f, 1.5f, 0.95f),
  };
  step(&state, config, 0.0, locked_only);

  // Other person beyond handover_radius_m 0.7 (dist ≈ 1.04 m).
  const std::vector<PersonObservation> other_only = {
      make_obs(2, 0.2f, 0.0f, 0.2f),
  };
  const PersonTargetLockUpdateResult coast = step(&state, config, 1.9, other_only);
  expect(coast.nearest.valid, "AC3 coast valid at 1.9 s");
  expect(coast.nearest.coasting, "AC3 coast flag at 1.9 s");
  expect(coast.nearest.person.track_id == 1, "AC3 coast keeps old id");
  expect(near(coast.nearest.person.range, 1.5f), "AC3 coast keeps last range");
  expect(near(coast.nearest.person.x, 1.2f), "AC3 coast keeps last x");
  expect(coast.event == PersonTargetLockEvent::kNone, "AC3 coast no event");

  const PersonTargetLockUpdateResult relock = step(&state, config, 2.0, other_only);
  expect(relock.nearest.valid, "AC3 relock valid at 2.0 s");
  expect(!relock.nearest.coasting, "AC3 relock not coasting");
  expect(relock.nearest.person.track_id == 2, "AC3 relock switches to other");
  expect(relock.event == PersonTargetLockEvent::kLostRelock, "AC3 relock event");
  expect(relock.previous_track_id == 1, "AC3 relock previous id");
  expect(near(relock.absent_s, 2.0), "AC3 relock absent");

  PersonTargetLockState state2;
  step(&state2, config, 0.0, locked_only);
  const PersonTargetLockUpdateResult lost_empty = step(&state2, config, 2.0, {});
  expect(!lost_empty.nearest.valid, "AC3 empty list after lost → invalid");
  expect(!lost_empty.nearest.coasting, "AC3 empty after lost not coasting");
  expect(lost_empty.event == PersonTargetLockEvent::kLost, "AC3 empty lost event");
  expect(lost_empty.previous_track_id == 1, "AC3 empty previous id");
}

// AC4: coast adds locked to output list when missing from input.
void test_ac4_coast_appends_to_persons() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  const std::vector<PersonObservation> locked_only = {
      make_obs(1, 1.0f, 0.0f, 1.0f),
  };
  step(&state, config, 0.0, locked_only);

  // Other person beyond handover_radius_m 0.7 (dist 0.8 m).
  const std::vector<PersonObservation> other_only = {
      make_obs(2, 0.2f, 0.0f, 0.2f),
  };
  const PersonTargetLockUpdateResult result = step(&state, config, 1.0, other_only);
  expect(result.persons.size() == 2U, "AC4 coast adds locked to persons");
  expect(result.persons[0].track_id == 2, "AC4 observed person kept");
  expect(result.persons[1].track_id == 1, "AC4 coasted locked appended");
  expect(result.nearest.person.track_id == 1, "AC4 nearest still coasted id");
}

// Empty list after target_lost_s with no observations → invalid.
void test_empty_after_lost_invalid() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 1.0f, 0.0f, 1.0f)});
  const PersonTargetLockUpdateResult result = step(&state, config, 2.0, {});
  expect(!result.nearest.valid, "empty after lost invalid");
  expect(!result.nearest.coasting, "empty after lost not coasting");
  expect(result.persons.empty(), "empty persons after lost with no obs");
  expect(!state.has_lock, "lock cleared after lost with no obs");
  expect(result.event == PersonTargetLockEvent::kLost, "empty after lost event");
  expect(result.previous_track_id == 1, "empty after lost previous id");
  expect(near(result.absent_s, 2.0), "empty after lost absent");
}

// Challenger disappearance resets dwell.
void test_challenger_disappears_resets_dwell() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 1.0f, 0.0f, 1.0f)});

  const std::vector<PersonObservation> both = {
      make_obs(1, 1.0f, 0.0f, 1.0f),
      make_obs(2, 0.4f, 0.0f, 0.4f),
  };
  step(&state, config, 0.0, both);
  expect(state.challenger_id == 2, "challenger armed");

  const std::vector<PersonObservation> locked_only = {
      make_obs(1, 1.0f, 0.0f, 1.0f),
  };
  step(&state, config, 1.0, locked_only);
  expect(!state.has_challenger, "challenger cleared when absent");

  const PersonTargetLockUpdateResult result = step(&state, config, 3.0, both);
  expect(result.nearest.person.track_id == 1, "no switch without continuous dwell");
}

// Target returns during coast window — coasting clears, lock held.
void test_return_from_coast() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  const std::vector<PersonObservation> locked_only = {
      make_obs(1, 1.2f, 0.3f, 1.5f),
  };
  step(&state, config, 0.0, locked_only);

  const std::vector<PersonObservation> other_only = {
      make_obs(2, 0.2f, 0.0f, 0.2f),
  };
  const PersonTargetLockUpdateResult coast = step(&state, config, 1.0, other_only);
  expect(coast.nearest.coasting, "return-from-coast coast active");

  const std::vector<PersonObservation> locked_back = {
      make_obs(1, 1.1f, 0.2f, 1.4f),
      make_obs(2, 0.2f, 0.0f, 0.2f),
  };
  const PersonTargetLockUpdateResult observed = step(&state, config, 1.5, locked_back);
  expect(observed.nearest.valid, "return-from-coast valid");
  expect(!observed.nearest.coasting, "return-from-coast not coasting");
  expect(observed.nearest.person.track_id == 1, "return-from-coast keeps locked id");
  expect(near(observed.nearest.person.range, 1.4f), "return-from-coast updated range");
}

// SD028 AC1: other id within radius → handover, not coasting.
void test_handover_within_radius() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 1.0f, 0.0f, 1.0f)});

  const std::vector<PersonObservation> other = {make_obs(2, 1.5f, 0.0f, 1.5f)};
  const PersonTargetLockUpdateResult result = step(&state, config, 0.5, other);
  expect(result.nearest.valid, "handover valid");
  expect(!result.nearest.coasting, "handover not coasting");
  expect(result.nearest.person.track_id == 2, "handover id");
  expect(result.event == PersonTargetLockEvent::kHandover, "handover event");
  expect(result.previous_track_id == 1, "handover previous id");
  expect(near(result.handover_distance_m, 0.5f), "handover distance");
  expect(near(result.absent_s, 0.5), "handover absent");
  expect(!state.has_challenger, "handover resets challenger");
  expect(result.persons.size() == 1U, "handover does not append old target");
  expect(result.persons[0].track_id == 2, "handover persons is new id");
}

// SD028 AC2: candidate outside radius coasts, then lost_relock by range.
void test_handover_outside_radius_then_lost_relock() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 1.0f, 0.0f, 1.0f)});

  const std::vector<PersonObservation> other = {make_obs(2, 1.8f, 0.0f, 1.8f)};
  const PersonTargetLockUpdateResult coast = step(&state, config, 1.9, other);
  expect(coast.nearest.coasting, "outside radius coasts");
  expect(coast.nearest.person.track_id == 1, "outside radius keeps id");
  expect(coast.event == PersonTargetLockEvent::kNone, "outside radius no handover");

  const PersonTargetLockUpdateResult relock = step(&state, config, 2.0, other);
  expect(relock.nearest.valid, "lost relock valid");
  expect(!relock.nearest.coasting, "lost relock not coasting");
  expect(relock.nearest.person.track_id == 2, "lost relock min range");
  expect(relock.event == PersonTargetLockEvent::kLostRelock, "lost relock event");
  expect(relock.previous_track_id == 1, "lost relock previous id");
  expect(near(relock.absent_s, 2.0), "lost relock absent");
}

// SD028 AC3: among in-radius candidates pick nearest to last pose, not min range.
void test_handover_picks_nearest_to_last_pose() {
  PersonTargetLockConfig config;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 1.0f, 0.0f, 1.0f)});

  const std::vector<PersonObservation> others = {
      make_obs(2, 1.3f, 0.0f, 1.3f),  // dist 0.3 m to last pose
      make_obs(3, 0.4f, 0.0f, 0.4f),  // dist 0.6 m, smaller range
  };
  const PersonTargetLockUpdateResult result = step(&state, config, 0.4, others);
  expect(result.event == PersonTargetLockEvent::kHandover, "two-candidate handover");
  expect(result.nearest.person.track_id == 2, "handover nearest to last pose");
  expect(near(result.handover_distance_m, 0.3f), "handover dist to last pose");
}

// SD028 AC4: radius 0 disables handover; SD017 coast at 0.1 m.
void test_handover_disabled_at_zero_radius() {
  PersonTargetLockConfig config;
  config.handover_radius_m = 0.0f;
  PersonTargetLockState state;
  step(&state, config, 0.0, {make_obs(1, 1.0f, 0.0f, 1.0f)});

  const std::vector<PersonObservation> other = {make_obs(2, 1.1f, 0.0f, 1.1f)};
  const PersonTargetLockUpdateResult coast = step(&state, config, 1.0, other);
  expect(coast.nearest.coasting, "radius 0 coasts");
  expect(coast.nearest.person.track_id == 1, "radius 0 keeps old id");
  expect(coast.event == PersonTargetLockEvent::kNone, "radius 0 no handover event");
  expect(coast.persons.size() == 2U, "radius 0 appends coasted target");
  expect(coast.persons[1].track_id == 1, "radius 0 persons appends id 1");

  const PersonTargetLockUpdateResult relock = step(&state, config, 2.0, other);
  expect(relock.event == PersonTargetLockEvent::kLostRelock, "radius 0 lost relock");
  expect(relock.nearest.person.track_id == 2, "radius 0 relock id");
}

}  // namespace

int main() {
  test_first_lock_min_range();
  test_ac1_margin_too_small();
  test_ac1_dwell_too_short();
  test_ac2_switch_after_dwell();
  test_challenger_change_resets_dwell();
  test_ac3_coast_then_relock();
  test_ac4_coast_appends_to_persons();
  test_empty_after_lost_invalid();
  test_challenger_disappears_resets_dwell();
  test_return_from_coast();
  test_handover_within_radius();
  test_handover_outside_radius_then_lost_relock();
  test_handover_picks_nearest_to_last_pose();
  test_handover_disabled_at_zero_radius();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_person_target_lock: ok\n";
  return 0;
}
