#!/usr/bin/env python3
"""T1 runtime halt gate + UART order/lock. No ROS, no hardware."""
import os
import sys
import threading
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from ros_robot_controller.t1_protocol import frame_set_motor_speed, t1_init_frames  # noqa: E402
from ros_robot_controller.t1_runtime import (  # noqa: E402
    T1MotorGate,
    all_four_signs_flipped,
    is_full_zero,
    speeds_by_id,
)

POS = [[1, 0.4], [2, 0.4], [3, -0.4], [4, -0.4]]
NEG = [[1, -0.4], [2, -0.4], [3, 0.4], [4, 0.4]]
ZERO = [[1, 0.0], [2, 0.0], [3, 0.0], [4, 0.0]]
FASTER = [[1, 0.7], [2, 0.7], [3, -0.7], [4, -0.7]]
POS_TUPLES = [(int(i), float(rps)) for i, rps in POS]
NEG_TUPLES = [(int(i), float(rps)) for i, rps in NEG]


class FakeClock:
    def __init__(self, t=0.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


class T1MotorGateTest(unittest.TestCase):
    def test_full_zero_requires_ids_1_to_4(self):
        self.assertTrue(is_full_zero(speeds_by_id(ZERO)))
        self.assertFalse(is_full_zero({1: 0.0, 2: 0.0, 3: 0.0}))
        self.assertFalse(is_full_zero(speeds_by_id(POS)))

    def test_startup_zeros_dropped(self):
        gate = T1MotorGate()
        for _ in range(8):
            decision = gate.observe(ZERO)
            self.assertFalse(decision.halt)
            self.assertFalse(decision.send_speed)
            self.assertFalse(decision.startup_preflight)
        self.assertTrue(gate.startup_preflight)
        self.assertEqual(gate.halt_count, 0)
        self.assertEqual(gate.speed_sent_count, 0)
        self.assertEqual(gate.zero_suppressed_count, 8)
        self.assertEqual(gate.preflight_count, 0)

    def test_startup_zeros_do_not_halt(self):
        gate = T1MotorGate()
        for _ in range(5):
            decision = gate.observe(ZERO)
            self.assertFalse(decision.halt)
            self.assertFalse(decision.send_speed)

    def test_first_nonzero_is_startup_preflight_then_speed(self):
        gate = T1MotorGate()
        first = gate.observe(POS)
        self.assertTrue(first.halt)
        self.assertTrue(first.send_speed)
        self.assertTrue(first.startup_preflight)
        self.assertFalse(first.idle_preflight)
        self.assertFalse(gate.startup_preflight)
        self.assertEqual(gate.preflight_count, 1)
        self.assertEqual(gate.halt_count, 1)
        self.assertEqual(gate.speed_sent_count, 1)

    def test_second_same_sign_is_speed_only(self):
        gate = T1MotorGate()
        self.assertTrue(gate.observe(POS).startup_preflight)
        second = gate.observe(FASTER)
        self.assertFalse(second.halt)
        self.assertTrue(second.send_speed)
        self.assertFalse(second.startup_preflight)
        self.assertEqual(gate.preflight_count, 1)
        self.assertEqual(gate.halt_count, 1)
        self.assertEqual(gate.speed_sent_count, 2)

    def test_nonzero_then_zero_halts_once_without_extra_speed(self):
        gate = T1MotorGate()
        first = gate.observe(POS)
        self.assertTrue(first.halt)
        self.assertTrue(first.startup_preflight)
        self.assertTrue(first.send_speed)
        edge = gate.observe(ZERO)
        self.assertTrue(edge.halt)
        self.assertFalse(edge.startup_preflight)
        self.assertFalse(edge.send_speed)
        again = gate.observe(ZERO)
        self.assertFalse(again.halt)
        self.assertFalse(again.send_speed)
        third = gate.observe(ZERO)
        self.assertFalse(third.halt)
        self.assertFalse(third.send_speed)
        self.assertEqual(gate.halt_count, 2)
        self.assertEqual(gate.preflight_count, 1)

    def test_halt_then_n_zeros_no_uart_decision(self):
        gate = T1MotorGate()
        gate.observe(POS)
        gate.observe(ZERO)
        for _ in range(20):
            decision = gate.observe(ZERO)
            self.assertFalse(decision.halt)
            self.assertFalse(decision.send_speed)
        self.assertEqual(gate.halt_count, 2)
        self.assertEqual(gate.speed_sent_count, 1)
        self.assertEqual(gate.preflight_count, 1)

    def test_repeated_zeros_do_not_init_storm(self):
        gate = T1MotorGate()
        gate.observe(POS)
        gate.observe(ZERO)
        halts = sum(1 for _ in range(20) if gate.observe(ZERO).halt)
        self.assertEqual(halts, 0)
        self.assertEqual(gate.preflight_count, 1)

    def test_next_nonzero_after_release_is_speed_only(self):
        gate = T1MotorGate()
        gate.observe(POS)
        gate.observe(ZERO)
        for _ in range(5):
            self.assertFalse(gate.observe(ZERO).send_speed)
        resume = gate.observe(POS)
        self.assertFalse(resume.halt)
        self.assertFalse(resume.startup_preflight)
        self.assertFalse(resume.idle_preflight)
        self.assertTrue(resume.send_speed)
        self.assertEqual(gate.preflight_count, 1)

    def test_watchdog_zeros_still_halt_once(self):
        # Adapter watchdog / Forbidden still publish ROS zeros; the first
        # full-zero after motion is the halt edge. Later zeros stay silent.
        gate = T1MotorGate()
        first = gate.observe(POS)
        self.assertTrue(first.send_speed)
        self.assertTrue(first.startup_preflight)
        edge = gate.observe(ZERO)
        self.assertTrue(edge.halt)
        self.assertFalse(edge.send_speed)
        for _ in range(10):
            later = gate.observe(ZERO)
            self.assertFalse(later.halt)
            self.assertFalse(later.send_speed)
        self.assertEqual(gate.halt_count, 2)

    def test_positive_to_negative_halts_then_sends_reverse(self):
        gate = T1MotorGate()
        first = gate.observe(POS)
        self.assertTrue(first.startup_preflight)
        decision = gate.observe(NEG)
        self.assertTrue(decision.halt)
        self.assertTrue(decision.send_speed)
        self.assertFalse(decision.startup_preflight)
        self.assertTrue(all_four_signs_flipped(speeds_by_id(POS), speeds_by_id(NEG)))
        self.assertEqual(gate.preflight_count, 1)
        self.assertEqual(gate.halt_count, 2)

    def test_reverse_preserved_after_zero_gap(self):
        gate = T1MotorGate()
        gate.observe(POS)
        gate.observe(ZERO)
        # Already halted; reverse after silence is just a new nonzero.
        decision = gate.observe(NEG)
        self.assertFalse(decision.halt)
        self.assertTrue(decision.send_speed)

    def test_same_sign_does_not_halt(self):
        gate = T1MotorGate()
        self.assertTrue(gate.observe(POS).halt)
        decision = gate.observe(FASTER)
        self.assertFalse(decision.halt)
        self.assertTrue(decision.send_speed)

    def test_yaw_without_linear_flip_does_not_halt(self):
        # +wz stock: all four RPS same sign after tank invert of 1/2.
        gate = T1MotorGate()
        self.assertTrue(gate.observe(POS).startup_preflight)
        yaw = [[1, 0.3], [2, 0.3], [3, 0.3], [4, 0.3]]
        decision = gate.observe(yaw)
        self.assertFalse(decision.halt)
        self.assertTrue(decision.send_speed)

    def test_new_gate_after_restart_preflights_again(self):
        first = T1MotorGate()
        self.assertTrue(first.observe(POS).startup_preflight)
        first.observe(ZERO)
        second = T1MotorGate()
        again = second.observe(POS)
        self.assertTrue(again.halt)
        self.assertTrue(again.startup_preflight)
        self.assertTrue(again.send_speed)
        self.assertEqual(second.preflight_count, 1)
        self.assertEqual(first.preflight_count, 1)

    def test_default_idle_does_not_rearm_after_halt(self):
        clock = FakeClock()
        gate = T1MotorGate(idle_preflight_s=0.0, clock=clock)
        gate.observe(POS)
        gate.observe(ZERO)
        clock.advance(3600.0)
        resume = gate.observe(POS)
        self.assertFalse(resume.halt)
        self.assertFalse(resume.idle_preflight)
        self.assertEqual(gate.preflight_count, 1)

    def test_optional_idle_preflight_once_without_storm(self):
        clock = FakeClock()
        gate = T1MotorGate(idle_preflight_s=5.0, clock=clock)
        self.assertTrue(gate.observe(POS).startup_preflight)
        gate.observe(ZERO)
        for _ in range(12):
            self.assertFalse(gate.observe(ZERO).halt)
        clock.advance(4.9)
        early = gate.observe(POS)
        self.assertFalse(early.halt)
        self.assertFalse(early.idle_preflight)
        gate.observe(ZERO)
        clock.advance(5.0)
        idle = gate.observe(POS)
        self.assertTrue(idle.halt)
        self.assertTrue(idle.idle_preflight)
        self.assertTrue(idle.send_speed)
        self.assertFalse(idle.startup_preflight)
        same = gate.observe(FASTER)
        self.assertFalse(same.halt)
        self.assertEqual(gate.preflight_count, 2)
        zeros = sum(1 for _ in range(20) if gate.observe(ZERO).halt)
        self.assertEqual(zeros, 1)


class T1DispatchSerialTest(unittest.TestCase):
    @staticmethod
    def _ensure_serial_stub():
        import types

        if "serial" in sys.modules:
            return
        stub = types.ModuleType("serial")

        class Serial:  # pragma: no cover - import stub only
            def __init__(self, *args, **kwargs):
                pass

        stub.Serial = Serial
        sys.modules["serial"] = stub

    def _board(self, port):
        self._ensure_serial_stub()
        from ros_robot_controller.ros_robot_controller_sdk import Board

        board = Board.__new__(Board)
        board.port = port
        board.write_lock = threading.RLock()
        board._in_t1_init = False
        board.uart_runtime_speed_frames = 0
        board.uart_runtime_speed_zeros = 0
        board.uart_halts = 0
        return board

    def _drive(self, board, gate, speeds):
        decision = gate.observe(speeds)
        board.dispatch_motors(halt=decision.halt, speeds=speeds if decision.send_speed else None)
        return decision

    def test_halt_then_reverse_frame_order(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        board.dispatch_motors(halt=True, speeds=NEG)
        reverse = frame_set_motor_speed(NEG_TUPLES)
        self.assertEqual(port.writes, t1_init_frames(zero_repeats=1) + [reverse])
        self.assertEqual(board.uart_runtime_speed_zeros, 0)

    def test_first_nonzero_writes_init_then_original_speed_once(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        gate = T1MotorGate()
        for _ in range(6):
            self._drive(board, gate, ZERO)
        self.assertEqual(port.writes, [])
        first = self._drive(board, gate, POS)
        self.assertTrue(first.startup_preflight)
        self.assertTrue(first.halt)
        self.assertTrue(first.send_speed)
        speed = frame_set_motor_speed(POS_TUPLES)
        self.assertEqual(port.writes, t1_init_frames(zero_repeats=1) + [speed])
        self.assertEqual(board.uart_halts, 1)
        self.assertEqual(board.uart_runtime_speed_zeros, 0)
        after_preflight = list(port.writes)
        second = self._drive(board, gate, FASTER)
        self.assertFalse(second.halt)
        self.assertTrue(second.send_speed)
        self.assertEqual(
            port.writes, after_preflight + [frame_set_motor_speed([(int(i), float(rps)) for i, rps in FASTER])]
        )

    def test_release_then_next_nonzero_is_speed_only(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        gate = T1MotorGate()
        self._drive(board, gate, POS)
        after_preflight = len(port.writes)
        self._drive(board, gate, ZERO)
        self.assertEqual(port.writes[after_preflight:], t1_init_frames(zero_repeats=1))
        after_halt = len(port.writes)
        resume = self._drive(board, gate, POS)
        self.assertFalse(resume.halt)
        self.assertTrue(resume.send_speed)
        self.assertEqual(port.writes[after_halt:], [frame_set_motor_speed(POS_TUPLES)])

    def test_halt_then_n_zeros_no_uart(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        gate = T1MotorGate()
        self._drive(board, gate, POS)
        after_motion = len(port.writes)
        self.assertGreater(after_motion, 0)
        self._drive(board, gate, ZERO)
        after_halt = len(port.writes)
        self.assertEqual(port.writes[after_motion:], t1_init_frames(zero_repeats=1))
        for _ in range(12):
            self._drive(board, gate, ZERO)
        self.assertEqual(len(port.writes), after_halt)
        self.assertEqual(board.uart_runtime_speed_zeros, 0)
        self.assertEqual(board.uart_halts, 2)

    def test_startup_zeros_dropped_no_uart(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        gate = T1MotorGate()
        for _ in range(6):
            self._drive(board, gate, ZERO)
        self.assertEqual(port.writes, [])
        self.assertEqual(board.uart_runtime_speed_zeros, 0)
        resume = self._drive(board, gate, POS)
        self.assertTrue(resume.send_speed)
        self.assertTrue(resume.startup_preflight)
        self.assertEqual(port.writes, t1_init_frames(zero_repeats=1) + [frame_set_motor_speed(POS_TUPLES)])

    def test_restart_new_gate_preflight_again_on_uart(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        self._drive(board, T1MotorGate(), POS)
        n_first = len(port.writes)
        self._drive(board, T1MotorGate(), POS)
        self.assertEqual(len(port.writes), 2 * n_first)
        self.assertEqual(port.writes[n_first:], t1_init_frames(zero_repeats=1) + [frame_set_motor_speed(POS_TUPLES)])

    def test_reverse_path_remains_halt_then_reverse(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        gate = T1MotorGate()
        self._drive(board, gate, POS)
        after_preflight = len(port.writes)
        reverse = self._drive(board, gate, NEG)
        self.assertTrue(reverse.halt)
        self.assertTrue(reverse.send_speed)
        self.assertFalse(reverse.startup_preflight)
        self.assertEqual(
            port.writes[after_preflight:], t1_init_frames(zero_repeats=1) + [frame_set_motor_speed(NEG_TUPLES)]
        )

    def test_no_init_storm_on_same_sign_or_zeros(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        gate = T1MotorGate()
        self._drive(board, gate, POS)
        after_preflight = len(port.writes)
        for _ in range(8):
            self._drive(board, gate, FASTER)
        self.assertEqual(len(port.writes), after_preflight + 8)
        self._drive(board, gate, ZERO)
        after_halt = len(port.writes)
        for _ in range(20):
            self._drive(board, gate, ZERO)
        self.assertEqual(len(port.writes), after_halt)
        self.assertEqual(board.uart_halts, 2)

    def test_shutdown_halt_is_init_sequence(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        board.dispatch_motors(halt=True, speeds=None)
        self.assertEqual(port.writes, t1_init_frames(zero_repeats=1))
        self.assertEqual(board.uart_runtime_speed_zeros, 0)
        self.assertEqual(board.uart_halts, 1)

    def test_silent_dispatch_writes_nothing(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        board.dispatch_motors(halt=False, speeds=None)
        self.assertEqual(port.writes, [])

    def test_writes_hold_the_serial_lock(self):
        class FakePort:
            def __init__(self, board_ref):
                self.writes = []
                self.board_ref = board_ref
                self.flushes = 0

            def write(self, data):
                if not self.board_ref["board"].write_lock.locked():
                    raise AssertionError("UART write without write_lock")
                self.writes.append(bytes(data))

            def flush(self):
                if not self.board_ref["board"].write_lock.locked():
                    raise AssertionError("UART flush without write_lock")
                self.flushes += 1

        box = {"board": None}
        port = FakePort(box)
        board = self._board(port)
        box["board"] = board
        board.dispatch_motors(halt=True, speeds=POS)
        self.assertEqual(port.writes, t1_init_frames(zero_repeats=1) + [frame_set_motor_speed(POS_TUPLES)])
        self.assertGreaterEqual(port.flushes, 1)

    def test_startup_init_is_type_type_battery_zero(self):
        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        port = FakePort()
        board = self._board(port)
        board.apply_t1_motor_init(zero_repeats=1)
        self.assertEqual(port.writes, t1_init_frames(zero_repeats=1))
        self.assertEqual(board.uart_runtime_speed_zeros, 0)


if __name__ == "__main__":
    unittest.main()
