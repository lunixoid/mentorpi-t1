"""Edge-triggered T1 hardware halt for the sole /dev/rrc owner.

A speed frame of +0.0 does not unlatch STM32. The proven stop is the same
UART init as a successful t1ctl restart (JGB37 x2, battery 0x1af4, full
zero IDs 1-4). This module only decides *when* to run that sequence.

State machine (one decision per incoming motor frame):
  process start: startup_preflight = True, had_nonzero = False
  extra zeros while disarmed     -> UART silent (halt=False, send_speed=False)
  first nonzero of this process  -> halt then original speed (startup preflight);
                                    consume startup_preflight
  full zero IDs 1-4, armed       -> halt once, disarm (init already zeros)
  next nonzero after runtime halt -> speed only (preflight already consumed)
  all four signs flipped,
  no intermediate zero           -> halt, then the new speed frame (one lock)
  same sign / magnitude only     -> speed only

Startup preflight and post-halt disarmed are distinct: both have
had_nonzero=False, but only the former still has startup_preflight.
Constructor UART init can race STM32/serial-open; the proven working
path is type×2+battery+zero immediately before a real speed (runtime halt).
Idle after that halt does not re-arm preflight (no init-storm).

ROS zero topics are not UART zero frames. After startup init or a hardware
halt, full-zero speed frames stay off the wire until the next real nonzero.
Signs of the caller speed list are never rewritten here.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Dict, Mapping, Optional, Sequence, Union

EPS = 1e-9
MOTOR_IDS = (1, 2, 3, 4)

SpeedItem = Sequence[Union[int, float]]
SpeedList = Sequence[SpeedItem]


@dataclass(frozen=True)
class MotorDecision:
    halt: bool
    send_speed: bool
    startup_preflight: bool = False
    idle_preflight: bool = False


def speeds_by_id(speeds: SpeedList) -> Dict[int, float]:
    out: Dict[int, float] = {}
    for item in speeds:
        out[int(item[0])] = float(item[1])
    return out


def is_full_zero(by_id: Mapping[int, float], eps: float = EPS) -> bool:
    return all(i in by_id and abs(by_id[i]) <= eps for i in MOTOR_IDS)


def any_nonzero(by_id: Mapping[int, float], eps: float = EPS) -> bool:
    return any(abs(v) > eps for v in by_id.values())


def all_four_signs_flipped(
    prev: Mapping[int, float],
    new: Mapping[int, float],
    eps: float = EPS,
) -> bool:
    """True only if every ID 1-4 is nonzero in both frames and each product < 0."""
    for motor_id in MOTOR_IDS:
        a = float(prev.get(motor_id, 0.0))
        b = float(new.get(motor_id, 0.0))
        if abs(a) <= eps or abs(b) <= eps:
            return False
        if a * b >= 0.0:
            return False
    return True


class T1MotorGate:
    """Edge detector. Not a UART writer; the Board applies the decision.

    idle_preflight_s: if > 0, a disarmed nonzero after that many seconds
    since the last runtime halt also gets halt+speed. Default 0: disabled.
    Post-halt first nonzero is already the proven-correct path; do not
    enable this on the stand (init-storm / extra zero pulse on every stick).
    """

    def __init__(
        self,
        idle_preflight_s: float = 0.0,
        clock: Optional[Callable[[], float]] = None,
    ) -> None:
        self.had_nonzero = False
        self.startup_preflight = True
        self.idle_preflight_s = float(idle_preflight_s)
        if clock is None and self.idle_preflight_s > 0.0:
            self._clock = time.monotonic
        else:
            self._clock = clock
        self._last_halt_at: Optional[float] = None
        self.last: Dict[int, float] = {i: 0.0 for i in MOTOR_IDS}
        self.halt_count = 0
        self.speed_sent_count = 0
        self.zero_suppressed_count = 0
        self.preflight_count = 0

    def _now(self) -> float:
        if self._clock is None:
            return 0.0
        return float(self._clock())

    def _idle_preflight_due(self) -> bool:
        if self.idle_preflight_s <= 0.0:
            return False
        if self.had_nonzero or self.startup_preflight:
            return False
        if self._last_halt_at is None:
            return False
        return (self._now() - self._last_halt_at) >= self.idle_preflight_s

    def observe(self, speeds: SpeedList) -> MotorDecision:
        by_id = speeds_by_id(speeds)
        if is_full_zero(by_id):
            halt = self.had_nonzero
            self.had_nonzero = False
            self.last = {i: 0.0 for i in MOTOR_IDS}
            if halt:
                self.halt_count += 1
                self._last_halt_at = self._now()
            else:
                self.zero_suppressed_count += 1
            # Init already writes the full zero. Later ROS zeros must not
            # become UART speed-zero chatter.
            return MotorDecision(halt=halt, send_speed=False)

        reverse = self.had_nonzero and all_four_signs_flipped(self.last, by_id)
        startup_preflight = self.startup_preflight
        idle_preflight = self._idle_preflight_due()
        halt = reverse or startup_preflight or idle_preflight
        if startup_preflight:
            self.startup_preflight = False
        if startup_preflight or idle_preflight:
            self.preflight_count += 1
        if halt:
            self.halt_count += 1
        if any_nonzero(by_id):
            self.had_nonzero = True
        self.last = {i: by_id.get(i, 0.0) for i in MOTOR_IDS}
        self.speed_sent_count += 1
        return MotorDecision(
            halt=halt,
            send_speed=True,
            startup_preflight=startup_preflight,
            idle_preflight=idle_preflight,
        )
