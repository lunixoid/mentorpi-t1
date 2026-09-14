#!/usr/bin/env python3
"""Byte-level T1 motor init/halt frames. No serial, no ROS."""
import math
import os
import struct
import sys
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from ros_robot_controller.t1_protocol import (  # noqa: E402
    BATTERY_LEVEL_T1,
    CRC8_TABLE,
    MOTOR_TYPE_JGB37,
    PACKET_FUNC_MOTOR,
    PACKET_FUNC_SYS,
    checksum_crc8,
    frame_set_battery_level,
    frame_set_motor_speed,
    frame_set_motor_type,
    full_zero_speeds,
    payload_set_motor_speed,
    t1_init_frames,
    wrap_frame,
)


class T1InitPacketsTest(unittest.TestCase):
    def test_wrap_crc_covers_func_len_payload(self):
        payload = b"\x05\x01"
        frame = wrap_frame(PACKET_FUNC_MOTOR, payload)
        self.assertEqual(frame[0:2], b"\xaa\x55")
        self.assertEqual(frame[2], PACKET_FUNC_MOTOR)
        self.assertEqual(frame[3], len(payload))
        self.assertEqual(frame[4:-1], payload)
        self.assertEqual(frame[-1], checksum_crc8(frame[2:-1]))

    def test_motor_type_jgb37_payload_and_order(self):
        frame = frame_set_motor_type(MOTOR_TYPE_JGB37)
        self.assertEqual(frame.hex(), "aa550302050166")
        self.assertEqual(list(frame[4:6]), [0x05, 0x01])

    def test_battery_0x1af4_little_endian(self):
        frame = frame_set_battery_level(BATTERY_LEVEL_T1)
        self.assertEqual(frame.hex(), "aa55000301f41a62")
        self.assertEqual(BATTERY_LEVEL_T1, 0x1AF4)
        self.assertEqual(frame[4:7], bytes([0x01, 0xF4, 0x1A]))
        self.assertEqual(frame[2], PACKET_FUNC_SYS)

    def test_full_zero_frame_has_ids_1_to_4(self):
        speeds = full_zero_speeds()
        self.assertEqual(speeds, [(1, 0.0), (2, 0.0), (3, 0.0), (4, 0.0)])
        payload = payload_set_motor_speed(speeds)
        self.assertEqual(payload[0], 0x01)
        self.assertEqual(payload[1], 4)
        offset = 2
        for expected_index in range(4):
            motor_index, rps = struct.unpack_from("<Bf", payload, offset)
            self.assertEqual(motor_index, expected_index)
            self.assertEqual(rps, 0.0)
            self.assertFalse(math.copysign(1.0, rps) < 0.0)
            offset += 5
        frame = frame_set_motor_speed(speeds)
        self.assertEqual(
            frame.hex(),
            "aa5503160104000000000001000000000200000000030000000007",
        )

    def test_init_order_type_type_battery_zero(self):
        frames = t1_init_frames(zero_repeats=1)
        self.assertEqual(len(frames), 4)
        self.assertEqual(frames[0], frame_set_motor_type(MOTOR_TYPE_JGB37))
        self.assertEqual(frames[1], frame_set_motor_type(MOTOR_TYPE_JGB37))
        self.assertEqual(frames[2], frame_set_battery_level(BATTERY_LEVEL_T1))
        self.assertEqual(frames[3], frame_set_motor_speed(full_zero_speeds()))

    def test_halt_repeats_full_zero_frames(self):
        frames = t1_init_frames(zero_repeats=3)
        self.assertEqual(len(frames), 6)
        zero = frame_set_motor_speed(full_zero_speeds())
        self.assertEqual(frames[3:], [zero, zero, zero])

    def test_sdk_crc_table_matches_protocol(self):
        self._ensure_serial_stub()
        from ros_robot_controller.ros_robot_controller_sdk import crc8_table

        self.assertEqual(list(crc8_table), CRC8_TABLE)

    def test_sdk_apply_t1_writes_protocol_frames(self):
        self._ensure_serial_stub()
        from ros_robot_controller.ros_robot_controller_sdk import Board

        class FakePort:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(bytes(data))

            def flush(self):
                pass

        board = Board.__new__(Board)
        board.port = FakePort()
        board.apply_t1_motor_init(zero_repeats=2)
        self.assertEqual(board.port.writes, t1_init_frames(zero_repeats=2))

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


if __name__ == "__main__":
    unittest.main()
