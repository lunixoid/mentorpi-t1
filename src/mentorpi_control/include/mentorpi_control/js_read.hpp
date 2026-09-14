#ifndef MENTORPI_CONTROL_JS_READ_HPP_
#define MENTORPI_CONTROL_JS_READ_HPP_

#include <poll.h>
#include <sys/types.h>

#include <cerrno>
#include <cstddef>

namespace mentorpi_control {

// Linux joydev (/dev/input/js*) reports *changes* only (plus JS_EVENT_INIT
// on open). A held stick produces no events. js_event.time is not a
// heartbeat. EAGAIN means the queue is empty, not that the device is gone.
//
// USB unplug / joydev teardown: read returns 0, ENODEV, EIO, or poll HUP/ERR.
//
// 2.4G RF power-off with the USB dongle still enumerated is not visible
// through this API: the fd stays open, last axes remain, no events. That
// is indistinguishable from a held stick. Do not fake it with Joy stamps.

enum class JsIoResult {
  GotEvent,
  NoEvent,
  DeviceLost,
  Error,
};

inline JsIoResult classify_js_read(ssize_t n, int err, size_t want) {
  if (n == static_cast<ssize_t>(want)) {
    return JsIoResult::GotEvent;
  }
  if (n < 0 && (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)) {
    return JsIoResult::NoEvent;
  }
  if (n == 0) {
    return JsIoResult::DeviceLost;
  }
  if (n < 0 && (err == ENODEV || err == ENXIO || err == EIO || err == ESHUTDOWN || err == EBADF)) {
    return JsIoResult::DeviceLost;
  }
  if (n < 0) {
    return JsIoResult::Error;
  }
  return JsIoResult::Error;
}

inline bool js_poll_lost(short revents) { return (revents & (POLLERR | POLLHUP | POLLNVAL)) != 0; }

// Idle HID queue while the fd is healthy is NOT a disconnect.
inline constexpr bool kJsIdleMeansRfLost = false;

}  // namespace mentorpi_control

#endif  // MENTORPI_CONTROL_JS_READ_HPP_
