"""UART frames for MentorPi T1 motor init/halt (Hiwonder RRC 2025 board API).

Frame: 0xAA 0x55 <func> <len> <payload...> <crc8(func..payload)>
Stock MentorPi_Tank startup (after Board/enable reception):
  set_motor_type(0x01 JGB37) twice, set_battery_level(0x1af4),
  then a full set_motor_speed zero with IDs 1..4.
A topic of +0.0 is not a hardware stop; these frames are.
"""

from __future__ import annotations

import struct
from typing import Iterable, List, Sequence, Tuple

PACKET_FUNC_SYS = 0
PACKET_FUNC_MOTOR = 3

MOTOR_SUB_SPEED = 0x01
MOTOR_SUB_TYPE = 0x05
SYS_SUB_BATTERY = 0x01

MOTOR_TYPE_JGB37 = 0x01
BATTERY_LEVEL_T1 = 0x1AF4
MOTOR_IDS = (1, 2, 3, 4)

# Copied from ros_robot_controller_sdk.crc8_table (must stay byte-identical).
CRC8_TABLE = [
    0,
    94,
    188,
    226,
    97,
    63,
    221,
    131,
    194,
    156,
    126,
    32,
    163,
    253,
    31,
    65,
    157,
    195,
    33,
    127,
    252,
    162,
    64,
    30,
    95,
    1,
    227,
    189,
    62,
    96,
    130,
    220,
    35,
    125,
    159,
    193,
    66,
    28,
    254,
    160,
    225,
    191,
    93,
    3,
    128,
    222,
    60,
    98,
    190,
    224,
    2,
    92,
    223,
    129,
    99,
    61,
    124,
    34,
    192,
    158,
    29,
    67,
    161,
    255,
    70,
    24,
    250,
    164,
    39,
    121,
    155,
    197,
    132,
    218,
    56,
    102,
    229,
    187,
    89,
    7,
    219,
    133,
    103,
    57,
    186,
    228,
    6,
    88,
    25,
    71,
    165,
    251,
    120,
    38,
    196,
    154,
    101,
    59,
    217,
    135,
    4,
    90,
    184,
    230,
    167,
    249,
    27,
    69,
    198,
    152,
    122,
    36,
    248,
    166,
    68,
    26,
    153,
    199,
    37,
    123,
    58,
    100,
    134,
    216,
    91,
    5,
    231,
    185,
    140,
    210,
    48,
    110,
    237,
    179,
    81,
    15,
    78,
    16,
    242,
    172,
    47,
    113,
    147,
    205,
    17,
    79,
    173,
    243,
    112,
    46,
    204,
    146,
    211,
    141,
    111,
    49,
    178,
    236,
    14,
    80,
    175,
    241,
    19,
    77,
    206,
    144,
    114,
    44,
    109,
    51,
    209,
    143,
    12,
    82,
    176,
    238,
    50,
    108,
    142,
    208,
    83,
    13,
    239,
    177,
    240,
    174,
    76,
    18,
    145,
    207,
    45,
    115,
    202,
    148,
    118,
    40,
    171,
    245,
    23,
    73,
    8,
    86,
    180,
    234,
    105,
    55,
    213,
    139,
    87,
    9,
    235,
    181,
    54,
    104,
    138,
    212,
    149,
    203,
    41,
    119,
    244,
    170,
    72,
    22,
    233,
    183,
    85,
    11,
    136,
    214,
    52,
    106,
    43,
    117,
    151,
    201,
    74,
    20,
    246,
    168,
    116,
    42,
    200,
    150,
    21,
    75,
    169,
    247,
    182,
    232,
    10,
    84,
    215,
    137,
    107,
    53,
]


def checksum_crc8(data: bytes) -> int:
    check = 0
    for byte in data:
        check = CRC8_TABLE[check ^ byte]
    return check & 0x00FF


def wrap_frame(func: int, payload: bytes) -> bytes:
    buf = bytes([0xAA, 0x55, int(func) & 0xFF, len(payload)]) + bytes(payload)
    return buf + bytes([checksum_crc8(buf[2:])])


def payload_set_motor_type(motor_type: int = MOTOR_TYPE_JGB37) -> bytes:
    return struct.pack("<BB", MOTOR_SUB_TYPE, int(motor_type) & 0xFF)


def payload_set_battery_level(level: int = BATTERY_LEVEL_T1) -> bytes:
    return struct.pack("BBB", SYS_SUB_BATTERY, level & 0xFF, (level >> 8) & 0xFF)


def payload_set_motor_speed(speeds: Sequence[Tuple[int, float]]) -> bytes:
    data = bytes([MOTOR_SUB_SPEED, len(speeds)])
    for motor_id, rps in speeds:
        data += struct.pack("<Bf", int(motor_id) - 1, float(rps))
    return data


def frame_set_motor_type(motor_type: int = MOTOR_TYPE_JGB37) -> bytes:
    return wrap_frame(PACKET_FUNC_MOTOR, payload_set_motor_type(motor_type))


def frame_set_battery_level(level: int = BATTERY_LEVEL_T1) -> bytes:
    return wrap_frame(PACKET_FUNC_SYS, payload_set_battery_level(level))


def frame_set_motor_speed(speeds: Sequence[Tuple[int, float]]) -> bytes:
    return wrap_frame(PACKET_FUNC_MOTOR, payload_set_motor_speed(speeds))


def full_zero_speeds() -> List[Tuple[int, float]]:
    return [(motor_id, 0.0) for motor_id in MOTOR_IDS]


def t1_init_frames(*, zero_repeats: int = 1) -> List[bytes]:
    """Stock T1 order: type, type, battery, then one or more full zero frames."""
    frames = [
        frame_set_motor_type(MOTOR_TYPE_JGB37),
        frame_set_motor_type(MOTOR_TYPE_JGB37),
        frame_set_battery_level(BATTERY_LEVEL_T1),
    ]
    zero = frame_set_motor_speed(full_zero_speeds())
    for _ in range(max(1, int(zero_repeats))):
        frames.append(zero)
    return frames


def frames_hex(frames: Iterable[bytes]) -> List[str]:
    return [bytes(frame).hex() for frame in frames]
