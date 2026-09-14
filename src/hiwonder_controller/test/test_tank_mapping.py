#!/usr/bin/env python3
"""Tank stop/reverse mapping without ROS or hardware."""
import math
import os
import sys
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from hiwonder_controller.tank_kinematics import all_stopped, tank_motor_speeds, tank_wheel_rps  # noqa: E402


class TankMappingTest(unittest.TestCase):
    def test_stop_is_four_id_zero_rps(self):
        speeds = tank_motor_speeds(0.0, 0.0)
        self.assertTrue(all_stopped(speeds))
        for motor_id, rps in speeds:
            self.assertIn(motor_id, (1, 2, 3, 4))
            self.assertEqual(rps, 0.0)

    def test_forward_stick_uses_stock_tank_signs(self):
        # Stick forward = +linear.x (pad_teleop). Extra linear invert removed.
        # Stock MentorPi_Tank inverts motors 1/2 only: 1/2 negative, 3/4 positive.
        # Physical forward/back is operator QA; yaw signs are unchanged below.
        rps = tank_wheel_rps(0.2, 0.0)
        self.assertEqual(len(rps), 4)
        self.assertLess(rps[0], 0.0)
        self.assertLess(rps[1], 0.0)
        self.assertGreater(rps[2], 0.0)
        self.assertGreater(rps[3], 0.0)
        self.assertAlmostEqual(abs(rps[0]), abs(rps[2]), places=9)
        self.assertAlmostEqual(abs(rps[0]), abs(rps[1]), places=9)
        self.assertAlmostEqual(abs(rps[2]), abs(rps[3]), places=9)
        speeds = tank_motor_speeds(0.2, 0.0)
        by_id = dict(speeds)
        self.assertEqual(sorted(by_id), [1, 2, 3, 4])
        self.assertLess(by_id[1], 0.0)
        self.assertLess(by_id[2], 0.0)
        self.assertGreater(by_id[3], 0.0)
        self.assertGreater(by_id[4], 0.0)

    def test_reverse_flips_all_four_signs(self):
        fwd = tank_wheel_rps(0.2, 0.0)
        rev = tank_wheel_rps(-0.2, 0.0)
        for i in range(4):
            self.assertAlmostEqual(rev[i], -fwd[i], places=9)
        self.assertGreater(rev[0], 0.0)
        self.assertLess(rev[2], 0.0)

    def test_positive_yaw_signs_unchanged(self):
        rps = tank_wheel_rps(0.0, 0.5)
        left = (rps[0] + rps[1]) / 2.0
        right = (rps[2] + rps[3]) / 2.0
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)
        # Left inverted, so +wz (CCW) drives left track "backward" in RPS
        # and right track forward — magnitudes match on a symmetric tank.
        self.assertAlmostEqual(abs(left), abs(right), places=9)

    def test_not_mecanum_right_side_invert_on_stop(self):
        # Mecanum overlay used [m1, m2, -m3, -m4] → IEEE -0.0 on 3/4 at stop.
        # Tank inverts 1/2, so 3/4 must not carry a negative zero from that.
        rps = tank_wheel_rps(0.0, 0.0)
        self.assertTrue(all(v == 0.0 for v in rps))
        self.assertFalse(any(math.copysign(1.0, v) < 0.0 for v in rps[2:]))

    def test_stop_positive_zero_on_all_four_motors(self):
        rps = tank_wheel_rps(0.0, 0.0)
        self.assertEqual(len(rps), 4)
        for v in rps:
            self.assertEqual(v, 0.0)
            self.assertFalse(math.copysign(1.0, v) < 0.0)


if __name__ == "__main__":
    unittest.main()
