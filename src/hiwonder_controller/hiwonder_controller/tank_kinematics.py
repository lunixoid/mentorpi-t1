"""MentorPi_Tank wheel mapping (stock controller mecanum.set_velocity signs).

Stock MentorPi odom uses tank geometry and inverts motors 1/2 (not 3/4).
Overlay JetRover_Mecanum inverted 3/4, which produced IEEE -0.0 on the right
track at Twist zero. This module is the T1 mapping; it does not talk UART.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

# Stock odom_publisher_node for MentorPi_Tank
TANK_WHEELBASE = 0.1368
TANK_TRACK_WIDTH = 0.1446
TANK_WHEEL_DIAMETER = 0.075


def _speed_to_rps(speed_m_s: float, wheel_diameter: float) -> float:
    return speed_m_s / (math.pi * wheel_diameter)


def tank_wheel_rps(
    linear_x: float,
    angular_z: float,
    *,
    wheelbase: float = TANK_WHEELBASE,
    track_width: float = TANK_TRACK_WIDTH,
    wheel_diameter: float = TANK_WHEEL_DIAMETER,
) -> List[float]:
    """Return [m1, m2, m3, m4] RPS. linear_y is always 0 on a tank."""
    # Stock MentorPi_Tank: invert motors 1/2 only. Do not extra-invert linear.x
    # here; pad_teleop already maps stick-forward to +Twist.linear.x.
    # Angular/pivot signs stay stock. Physical forward/back is operator QA.
    pivot = angular_z * (wheelbase + track_width) / 2.0
    motor1 = linear_x - pivot
    motor2 = linear_x - pivot
    motor3 = linear_x + pivot
    motor4 = linear_x + pivot
    linear = [-motor1, -motor2, motor3, motor4]
    # Unary minus on +0.0 is IEEE -0.0. STM32 treated that as "keep last RPS"
    # on JetRover mecanum invert 3/4. UART init is the stop; mapping still
    # must not re-latch a signed zero after init.
    return [0.0 if rps == 0.0 else rps for rps in (_speed_to_rps(v, wheel_diameter) for v in linear)]


def tank_motor_speeds(
    linear_x: float,
    angular_z: float,
) -> List[Tuple[int, float]]:
    rps = tank_wheel_rps(linear_x, angular_z)
    return [(i + 1, float(rps[i])) for i in range(4)]


def all_stopped(speeds: Sequence[Tuple[int, float]], eps: float = 1e-12) -> bool:
    ids = [motor_id for motor_id, _ in speeds]
    return ids == [1, 2, 3, 4] and all(abs(rps) <= eps for _, rps in speeds)
