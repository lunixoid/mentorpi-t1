#include <poll.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "mentorpi_control/joy_freshness.hpp"
#include "mentorpi_control/joy_twist_mapping.hpp"
#include "mentorpi_control/js_read.hpp"
#include "mentorpi_control/pad_command.hpp"
#include "mentorpi_control/publish_cadence.hpp"

using mentorpi_control::axis_or_zero;
using mentorpi_control::AxisLatch;
using mentorpi_control::AxisShapeParams;
using mentorpi_control::classify_js_read;
using mentorpi_control::decide_pad;
using mentorpi_control::default_cmd_freshness_ms;
using mentorpi_control::is_fresh;
using mentorpi_control::joy_axes_to_twist;
using mentorpi_control::JoyShapeState;
using mentorpi_control::JoyTwistParams;
using mentorpi_control::js_poll_lost;
using mentorpi_control::JsIoResult;
using mentorpi_control::kJsIdleMeansRfLost;
using mentorpi_control::PadCommand;
using mentorpi_control::scale_axis;
using mentorpi_control::shape_axis;
using mentorpi_control::timer_keepalive_due;
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

void expect_near(double a, double b, const char* what, double eps = 1e-9) {
  if (std::abs(a - b) > eps) {
    std::cerr << "FAIL: " << what << " got " << a << " expected " << b << '\n';
    ++g_fails;
  }
}

JoyTwistParams stand_params() {
  JoyTwistParams p;
  p.linear.enter = 0.10;
  p.linear.release = 0.06;
  p.linear.min_out = 0.10;
  p.linear.max_out = 0.5;
  p.angular.enter = 0.10;
  p.angular.release = 0.06;
  p.angular.min_out = 0.40;
  p.angular.max_out = 2.0;
  return p;
}

PadCommand pad(bool manual, bool command_fresh, bool remote_fresh, const std::vector<float>& axes,
               JoyShapeState& state) {
  return decide_pad(manual, command_fresh, remote_fresh, axes, 1, 2, stand_params(), state);
}

void test_mapping() {
  constexpr double dz = 0.1;
  constexpr double max_lin = 0.5;
  constexpr double max_ang = 2.0;

  expect(scale_axis(0.0, dz) == 0.0, "center is zero");
  expect(scale_axis(0.05, dz) == 0.0, "inside deadzone is zero");
  expect(scale_axis(-0.05, dz) == 0.0, "inside deadzone negative is zero");
  expect(scale_axis(0.1, dz) == 0.0, "deadzone edge is zero");
  expect_near(scale_axis(1.0, dz), 1.0, "full +1 stays 1 after renormalize");
  expect_near(scale_axis(-1.0, dz), -1.0, "full -1 stays -1 after renormalize");
  expect_near(scale_axis(0.55, dz), 0.5, "mid remaining range");
  expect_near(scale_axis(-0.55, dz), -0.5, "mid remaining range negative");
  expect_near(scale_axis(1.2, dz), 1.0, "clamp above 1");
  expect_near(scale_axis(-1.2, dz), -1.0, "clamp below -1");

  const Twist2d center = joy_axes_to_twist(0.0, 0.0, max_lin, max_ang, dz);
  expect(twist2d_is_zero(center), "neutral twist is fully zero");

  const Twist2d release = joy_axes_to_twist(0.05, -0.09, max_lin, max_ang, dz);
  expect(twist2d_is_zero(release), "release / deadzone is fully zero");

  const Twist2d fwd = joy_axes_to_twist(-1.0, 0.0, max_lin, max_ang, dz);
  expect_near(fwd.linear_x, max_lin, "full forward matches max_linear sign");
  expect_near(fwd.angular_z, 0.0, "linear does not leak into angular");

  const Twist2d back = joy_axes_to_twist(1.0, 0.0, max_lin, max_ang, dz);
  expect_near(back.linear_x, -max_lin, "full back matches inverted linear");

  const Twist2d yaw = joy_axes_to_twist(0.0, -1.0, max_lin, max_ang, dz);
  expect_near(yaw.linear_x, 0.0, "angular does not leak into linear");
  expect_near(yaw.angular_z, max_ang, "full yaw matches max_angular sign");

  const Twist2d both = joy_axes_to_twist(-1.0, 1.0, max_lin, max_ang, dz);
  expect_near(both.linear_x, max_lin, "independent linear at corner");
  expect_near(both.angular_z, -max_ang, "independent angular at corner");

  std::vector<float> axes = {0.f, -1.f, 1.f};
  const Twist2d indexed = joy_axes_to_twist(axes, 1, 2, max_lin, max_ang, dz);
  expect_near(indexed.linear_x, max_lin, "axis 1 linear");
  expect_near(indexed.angular_z, -max_ang, "axis 2 angular");
  expect(axis_or_zero(axes, 9) == 0.0, "missing axis is zero");
}

void test_shape_floor_and_hysteresis() {
  const JoyTwistParams p = stand_params();
  JoyShapeState st;
  AxisLatch latch;

  expect(shape_axis(0.0, p.linear, latch) == 0.0, "below enter is zero");
  expect(shape_axis(0.10, p.linear, latch) == 0.0, "enter edge stays zero");
  expect(!latch.active, "not latched at enter edge");

  const double just_over = 0.10 + 1e-4;
  const double u0 = shape_axis(just_over, p.linear, latch);
  expect(latch.active, "crossing enter latches");
  const double min_frac = p.linear.min_out / p.linear.max_out;
  const double t0 = (just_over - p.linear.enter) / (1.0 - p.linear.enter);
  expect_near(std::abs(u0), min_frac + t0 * (1.0 - min_frac),
              "crossing enter jumps to min fraction", 1e-9);
  expect(std::abs(u0) > 0.15, "first unit is the floor, not a tiny remnant");

  double prev = std::abs(shape_axis(-0.11, p.linear, latch));
  for (double a = 0.20; a <= 1.0 + 1e-12; a += 0.05) {
    const double mag = std::abs(shape_axis(-a, p.linear, latch));
    expect(mag + 1e-12 >= prev, "shaped magnitude is monotonic to max");
    prev = mag;
  }
  expect_near(shape_axis(-1.0, p.linear, latch), -1.0, "full stick is unit 1");
  expect_near(joy_axes_to_twist(-1.0, 0.0, p, st).linear_x, p.linear.max_out,
              "full forward still hits max_linear");

  st.reset();
  const Twist2d at_min = joy_axes_to_twist(-just_over, 0.0, p, st);
  const double t_enter = (just_over - p.linear.enter) / (1.0 - p.linear.enter);
  expect_near(at_min.linear_x, p.linear.min_out + t_enter * (p.linear.max_out - p.linear.min_out),
              "first twist equals linear_min");
  expect(at_min.linear_x > 0.09, "first linear command is the floor, not ~0");
  expect_near(at_min.angular_z, 0.0, "linear activate does not leak yaw");

  const double mid_raw = 0.55;
  const double t = (mid_raw - p.linear.enter) / (1.0 - p.linear.enter);
  const double expected_mid = p.linear.min_out + t * (p.linear.max_out - p.linear.min_out);
  expect_near(joy_axes_to_twist(-mid_raw, 0.0, p, st).linear_x, expected_mid,
              "remaining stick scales to max");

  expect_near(joy_axes_to_twist(-0.08, 0.0, p, st).linear_x, p.linear.min_out,
              "hysteresis band holds min until release");
  const Twist2d dropped = joy_axes_to_twist(-0.06, 0.0, p, st);
  expect(twist2d_is_zero(dropped), "at release threshold output is zero");
  expect(twist2d_is_zero(joy_axes_to_twist(-0.08, 0.0, p, st)),
         "after release, enter must be recrossed");
}

void test_shape_sign_and_independence() {
  const JoyTwistParams p = stand_params();
  JoyShapeState st;

  const Twist2d pos = joy_axes_to_twist(-0.5, 0.0, p, st);
  st.reset();
  const Twist2d neg = joy_axes_to_twist(0.5, 0.0, p, st);
  expect_near(pos.linear_x, -neg.linear_x, "linear sign symmetry");
  expect(pos.linear_x > 0.0, "forward stick still positive linear.x");

  st.reset();
  const Twist2d yaw_pos = joy_axes_to_twist(0.0, -0.5, p, st);
  st.reset();
  const Twist2d yaw_neg = joy_axes_to_twist(0.0, 0.5, p, st);
  expect_near(yaw_pos.angular_z, -yaw_neg.angular_z, "angular sign symmetry");
  expect(yaw_pos.angular_z > 0.0, "yaw stick polarity unchanged");

  st.reset();
  const Twist2d lin_only = joy_axes_to_twist(-0.4, 0.0, p, st);
  expect(lin_only.linear_x != 0.0, "linear active");
  expect_near(lin_only.angular_z, 0.0, "angular stays zero independently");

  st.reset();
  const Twist2d yaw_only = joy_axes_to_twist(0.0, -0.4, p, st);
  expect_near(yaw_only.linear_x, 0.0, "linear stays zero independently");
  expect(yaw_only.angular_z != 0.0, "angular active");

  st.reset();
  AxisLatch lin_latch;
  AxisLatch ang_latch;
  shape_axis(-0.3, p.linear, lin_latch);
  expect(shape_axis(0.0, p.angular, ang_latch) == 0.0, "angular latch independent of linear");
}

void test_shape_resets() {
  std::vector<float> held = {0.f, -0.8f, 0.4f};
  std::vector<float> band = {0.f, -0.08f, 0.f};
  std::vector<float> center = {0.f, 0.0f, 0.0f};

  JoyShapeState st;
  const PadCommand moving = pad(true, true, true, held, st);
  expect(!twist2d_is_zero(moving.twist), "held stick is non-zero");
  expect(st.linear.active && st.angular.active, "both axes latched");

  const PadCommand centered = pad(true, true, true, center, st);
  expect(twist2d_is_zero(centered.twist), "physical center is zero");
  expect(!st.linear.active && !st.angular.active, "center clears latches");
  expect(twist2d_is_zero(pad(true, true, true, band, st).twist),
         "after center, hysteresis band does not hold command");

  st.reset();
  pad(true, true, true, held, st);
  const PadCommand follow = pad(false, true, true, held, st);
  expect(!follow.publish_cmd, "Follow mutes cmd publisher");
  expect(twist2d_is_zero(follow.twist), "Follow zeros twist");
  expect(!st.linear.active && !st.angular.active, "Follow resets latches");
  expect(twist2d_is_zero(pad(true, true, true, band, st).twist),
         "Follow reset does not keep min output in the band");

  st.reset();
  pad(true, true, true, held, st);
  const PadCommand stale = pad(true, false, true, held, st);
  expect(stale.publish_cmd, "Manual keeps publisher on stale command");
  expect(twist2d_is_zero(stale.twist), "stale command is zero, not last non-zero");
  expect(!st.linear.active, "stale resets linear latch");
  expect(twist2d_is_zero(pad(true, true, true, band, st).twist),
         "stale/disconnect reset drops hysteresis");

  st.reset();
  pad(true, true, true, held, st);
  const PadCommand lost = pad(true, false, false, held, st);
  expect(twist2d_is_zero(lost.twist), "disconnect is zero Twist");
  expect(!lost.remote, "disconnect clears remote after joy_timeout");
  expect(!st.linear.active && !st.angular.active, "disconnect resets latches");
}

void test_no_halt_chatter() {
  const JoyTwistParams p = stand_params();
  JoyShapeState st;
  int falling = 0;
  bool was_nz = false;
  auto step = [&](double lin, double ang) {
    const Twist2d t = joy_axes_to_twist(lin, ang, p, st);
    const bool nz = !twist2d_is_zero(t);
    if (was_nz && !nz) {
      ++falling;
    }
    was_nz = nz;
    return t;
  };

  step(0.0, 0.0);
  expect(twist2d_is_zero(step(-0.15, 0.0)) == false, "activate linear");
  for (int i = 0; i < 40; ++i) {
    const double a = (i % 2 == 0) ? -0.07 : -0.09;
    expect(!twist2d_is_zero(step(a, 0.0)), "hysteresis band stays non-zero");
  }
  expect(falling == 0, "jitter in band does not create halt edges");
  expect(twist2d_is_zero(step(0.0, 0.0)), "center is one zero");
  expect(falling == 1, "physical center is a single falling edge");
  step(0.0, 0.0);
  step(0.0, 0.0);
  step(0.02, 0.0);
  expect(falling == 1, "zeros at center do not add halt edges");
}

void test_release_and_follow() {
  std::vector<float> held = {0.f, -0.8f, 0.4f};
  std::vector<float> idle = {0.f, 0.02f, -0.03f};
  JoyShapeState st;

  const PadCommand follow = pad(false, true, true, held, st);
  expect(!follow.publish_cmd, "Follow mutes cmd publisher");
  expect(twist2d_is_zero(follow.twist), "Follow does not emit stick twist");
  expect(follow.remote, "Follow still reports remote when joy is fresh");

  const PadCommand released = pad(true, true, true, idle, st);
  expect(released.publish_cmd, "Manual still publishes on release");
  expect(twist2d_is_zero(released.twist), "Manual release is zero Twist");

  const PadCommand moving = pad(true, true, true, held, st);
  expect(moving.publish_cmd, "Manual publishes while moving");
  expect(!twist2d_is_zero(moving.twist), "held stick is non-zero");
}

void test_freshness() {
  using clock = std::chrono::steady_clock;
  using ms = std::chrono::milliseconds;
  const auto t0 = clock::time_point{};

  expect(default_cmd_freshness_ms(20.0) == 100, "default cmd freshness is 2/rate_hz seconds in ms");
  expect(default_cmd_freshness_ms(10.0) == 200, "ratio tracks rate_hz");

  expect(is_fresh(true, t0, t0 + ms{100}, ms{100}), "command fresh at window");
  expect(!is_fresh(true, t0, t0 + ms{101}, ms{100}),
         "command stale just after window — do not hold last Twist");
  expect(is_fresh(true, t0, t0 + ms{101}, ms{1000}), "remote still present after command stale");
  expect(!is_fresh(true, t0, t0 + ms{1001}, ms{1000}), "remote inactive after joy_timeout_ms");
  expect(!is_fresh(false, t0, t0, ms{100}), "no receipt is not fresh");

  std::vector<float> held = {0.f, -1.f, 0.f};
  JoyShapeState st;
  const PadCommand stale_cmd = pad(true, false, true, held, st);
  expect(stale_cmd.publish_cmd, "Manual keeps publisher on stale command");
  expect(twist2d_is_zero(stale_cmd.twist), "stale command is zero, not last non-zero");
  expect(stale_cmd.remote, "remote can stay true while command is stale");

  const PadCommand lost = pad(true, false, false, held, st);
  expect(twist2d_is_zero(lost.twist), "disconnect is zero Twist");
  expect(!lost.remote, "disconnect clears remote after joy_timeout");
}

void test_timer_event_dedupe() {
  using clock = std::chrono::steady_clock;
  using ms = std::chrono::milliseconds;
  const auto t0 = clock::time_point{};
  const auto period = ms{50};

  expect(timer_keepalive_due(false, t0, t0, period), "first timer publishes before any event");
  expect(!timer_keepalive_due(true, t0 + ms{1}, t0, period), "event then early timer skipped");
  expect(!timer_keepalive_due(true, t0 + ms{49}, t0, period), "timer not due before period");
  expect(timer_keepalive_due(true, t0 + ms{50}, t0, period), "timer keepalive after period");
}

void test_js_semantics() {
  constexpr size_t kEv = 8;
  expect(classify_js_read(static_cast<ssize_t>(kEv), 0, kEv) == JsIoResult::GotEvent,
         "full read is an event");
  expect(classify_js_read(-1, EAGAIN, kEv) == JsIoResult::NoEvent,
         "EAGAIN is idle, not disconnect (held stick)");
  expect(classify_js_read(-1, EWOULDBLOCK, kEv) == JsIoResult::NoEvent, "EWOULDBLOCK is idle");
  expect(classify_js_read(-1, EINTR, kEv) == JsIoResult::NoEvent, "EINTR retry");
  expect(classify_js_read(0, 0, kEv) == JsIoResult::DeviceLost, "EOF is lost");
  expect(classify_js_read(-1, ENODEV, kEv) == JsIoResult::DeviceLost,
         "ENODEV is USB/joydev teardown");
  expect(classify_js_read(-1, EIO, kEv) == JsIoResult::DeviceLost, "EIO is lost");
  expect(!kJsIdleMeansRfLost, "JS API cannot treat HID idle as RF disconnect");
  expect(js_poll_lost(POLLERR), "POLLERR is lost");
  expect(js_poll_lost(POLLHUP), "POLLHUP is lost");
  expect(!js_poll_lost(POLLIN), "POLLIN alone is not lost");
}

}  // namespace

int main() {
  test_mapping();
  test_shape_floor_and_hysteresis();
  test_shape_sign_and_independence();
  test_shape_resets();
  test_no_halt_chatter();
  test_release_and_follow();
  test_freshness();
  test_timer_event_dedupe();
  test_js_semantics();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_joy_control: ok\n";
  return 0;
}
