# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright 2026 邓陈龙
#
"""Feetech SCS bus protocol — frame codec for HD-1910 servos (ticket #04).

Handwritten frame protocol (AGENTS.md constraint: stdlib only, no scservo_sdk —
WSL HTTPS networking makes pip installs unreliable on this host).

Frame layout (FT-SCS proprietary protocol, half-duplex TTL bus):

    Instruction: FF FF ID LEN INSTR P1 .. PN CHECKSUM
    Status:      FF FF ID LEN ERROR P1 .. PN CHECKSUM

    LEN      = N + 2  (parameters + instruction + checksum)
    CHECKSUM = ~(ID + LEN + INSTR + P1 + ... + PN) & 0xFF
    Broadcast ID = 0xFE

16-bit fields are little-endian on the wire for the STS/SMS application layer
(HD-1910 is a magnetically-encoded STS-family servo: 12-bit position, two-byte
fields low byte first). BIT15 on position/speed is the direction bit.

Key register addresses (STS memory table):
    40 (0x28) torque switch         41 (0x29) acceleration
    42 (0x2A) goal position (2 B)   46 (0x2E) running speed (2 B)
    56 (0x38) present position (2)  58 (0x3A) present speed (2)
    60 (0x3C) present load (2)      62 (0x3E) present voltage (1, 0.1 V)
    63 (0x3F) present temperature   69 (0x45) present current (2, 6.5 mA/LSB)

Instructions: PING 0x01, READ 0x02, WRITE 0x03, REG WRITE 0x04, ACTION 0x05,
SYNC READ 0x82, SYNC WRITE 0x83.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

HEADER = b"\xff\xff"
BROADCAST_ID = 0xFE

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83

# STS memory-table register addresses used by the Microduck driver.
REG_TORQUE_ENABLE = 40       # 0x28 torque switch (1 byte)
REG_ACCELERATION = 41        # 0x29 (1 byte)
REG_GOAL_POSITION = 42       # 0x2A (2 bytes, little-endian, bit15 = direction)
REG_GOAL_SPEED = 46          # 0x2E (2 bytes)
REG_PRESENT_POSITION = 56    # 0x38 (2 bytes)
REG_PRESENT_SPEED = 58       # 0x3A (2 bytes)
REG_PRESENT_LOAD = 60        # 0x3C (2 bytes)
REG_PRESENT_VOLTAGE = 62     # 0x3E (1 byte, 0.1 V/LSB)
REG_PRESENT_TEMPERATURE = 63 # 0x3F (1 byte)
REG_PRESENT_CURRENT = 69     # 0x45 (2 bytes, 6.5 mA/LSB)

# Feedback block read by sync_read: position..voltage covers 56..62 inclusive.
FEEDBACK_START = REG_PRESENT_POSITION
FEEDBACK_END = REG_PRESENT_VOLTAGE
FEEDBACK_LENGTH = FEEDBACK_END - FEEDBACK_START + 1  # 7 bytes

MAX_ID = 253
MAX_PACKET_DATA = 256 - 8  # LEN is a single byte: header(2)+id+len+instr+data+checksum


class ProtocolError(ValueError):
    """Malformed frame (bad checksum, truncated data, illegal field)."""


class TimeoutError_(TimeoutError):
    """A request that expects a status packet got none."""


# ---------------------------------------------------------------------------
# Checksum / 16-bit helpers
# ---------------------------------------------------------------------------

def checksum(payload: bytes) -> int:
    """SCS checksum: ~(ID + LEN + INSTR + params...) & 0xFF.

    `payload` is everything after the FF FF header, minus the checksum byte
    itself (i.e. ID, LEN, INSTR, parameters).
    """
    return (~sum(payload)) & 0xFF


def u16_to_le(value: int) -> bytes:
    """Host 16-bit -> two wire bytes, low byte first (STS byte order)."""
    if not 0 <= value <= 0xFFFF:
        raise ProtocolError(f"u16 out of range: {value}")
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def le_to_u16(data: bytes, offset: int = 0) -> int:
    """Two wire bytes -> host 16-bit, low byte first."""
    return data[offset] | (data[offset + 1] << 8)


# Signed 11-bit magnitude + sign: bit15 (position/speed) or bit10 (load) is the
# direction bit; the remaining 11 bits are the magnitude (same encoding the
# STS family uses for negative values).
DIRECTION_BIT = 15
LOAD_DIRECTION_BIT = 10


def encode_signed(value: int, direction_bit: int = DIRECTION_BIT) -> int:
    """Signed value -> STS encoded u16 (magnitude + direction bit)."""
    mask = (1 << direction_bit) - 1
    if value < 0:
        return (abs(value) & mask) | (1 << direction_bit)
    return value & mask


def decode_signed(value: int, direction_bit: int = DIRECTION_BIT) -> int:
    """STS encoded u16 -> signed value (direction bit -> negative)."""
    mask = (1 << direction_bit) - 1
    mag = value & mask
    if value & (1 << direction_bit):
        return -mag
    return mag


# ---------------------------------------------------------------------------
# Instruction packet builders
# ---------------------------------------------------------------------------

def build_instruction(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    """One instruction packet: FF FF ID LEN INSTR P1..PN CHECKSUM."""
    if not 0 <= servo_id <= 0xFE:
        raise ProtocolError(f"illegal servo id {servo_id}")
    length = len(params) + 2
    if length > 0xFF:
        raise ProtocolError("packet too long for LEN byte")
    body = bytes((servo_id, length, instruction)) + params
    return HEADER + body + bytes((checksum(body),))


def build_ping(servo_id: int) -> bytes:
    """PING (0x01): no parameters. A servo answers with a status packet."""
    return build_instruction(servo_id, INST_PING)


def build_read(servo_id: int, start: int, length: int) -> bytes:
    """READ (0x02): params = start address, byte count."""
    if not 1 <= length <= 0xFD:
        raise ProtocolError(f"illegal read length {length}")
    return build_instruction(servo_id, INST_READ, bytes((start, length)))


def build_write(servo_id: int, start: int, data: bytes) -> bytes:
    """WRITE (0x03): params = start address + data bytes."""
    return build_instruction(servo_id, INST_WRITE, bytes((start,)) + data)


def build_reg_write(servo_id: int, start: int, data: bytes) -> bytes:
    """REG WRITE (0x04): staged write, executed on ACTION."""
    return build_instruction(servo_id, INST_REG_WRITE, bytes((start,)) + data)


def build_action() -> bytes:
    """ACTION (0x05), broadcast."""
    return build_instruction(BROADCAST_ID, INST_ACTION)


def build_sync_write(start: int, data_length: int, entries: list[tuple[int, bytes]]) -> bytes:
    """SYNC WRITE (0x83), broadcast ID 0xFE.

    Params: start address, per-servo data length L, then per servo:
    ID + L data bytes. All servos share start address and L.

    Wire length field = (L + 1) * n + 4 per the FT-SCS spec — note the spec's
    Length counts parameters + instruction + checksum, which is what
    build_instruction computes; the "+4" form in the wiki is the same number
    (2 + n*(L+1) + 2 == (L+1)*n + 4).
    """
    if not entries:
        raise ProtocolError("sync_write needs at least one servo")
    if not 1 <= data_length <= 0xFD:
        raise ProtocolError(f"illegal sync_write data length {data_length}")
    params = bytes((start, data_length))
    for servo_id, data in entries:
        if len(data) != data_length:
            raise ProtocolError(
                f"sync_write entry id {servo_id}: got {len(data)} bytes, expected {data_length}"
            )
        params += bytes((servo_id,)) + data
    return build_instruction(BROADCAST_ID, INST_SYNC_WRITE, params)


def build_sync_read(servo_ids: list[int], start: int, length: int) -> bytes:
    """SYNC READ (0x82), broadcast ID 0xFE.

    Params: start address, byte count, then the participating servo IDs.
    Each addressed servo replies with its own status packet (in ID order).
    """
    if not servo_ids:
        raise ProtocolError("sync_read needs at least one servo")
    if len(servo_ids) > 0xFD:
        raise ProtocolError("too many servos for one sync_read")
    params = bytes((start, length)) + bytes(servo_ids)
    return build_instruction(BROADCAST_ID, INST_SYNC_READ, params)


# ---------------------------------------------------------------------------
# Status packet parser
# ---------------------------------------------------------------------------

def parse_status(frame: bytes) -> tuple[int, int, bytes]:
    """Parse a status packet. Returns (servo_id, error, parameters).

    Raises ProtocolError on malformed input:
    - truncated frame (shorter than 6 bytes / LEN shorter than the payload)
    - missing FF FF header
    - checksum mismatch
    """
    if len(frame) < 6:
        raise ProtocolError(f"truncated status frame ({len(frame)} bytes)")
    if frame[0] != 0xFF or frame[1] != 0xFF:
        raise ProtocolError("status frame missing FF FF header")
    servo_id = frame[2]
    length = frame[3]
    total = 4 + length  # header(2) + id + len + (length-1) body bytes incl. checksum
    if len(frame) < total:
        raise ProtocolError(f"truncated status frame: declared {total}, got {len(frame)}")
    # checksum covers ID + LEN + ERROR + parameters (everything before itself)
    expected = checksum(frame[2:2 + length + 1])
    if frame[2 + length + 1] != expected:
        raise ProtocolError(
            f"checksum mismatch: got 0x{frame[2 + length + 1]:02x}, expected 0x{expected:02x}"
        )
    error = frame[4]
    params = frame[5:2 + length + 1]
    return servo_id, error, params


def strip_leading_noise(buffer: bytes) -> bytes:
    """Drop bytes before the first FF FF header (half-duplex line noise)."""
    idx = buffer.find(HEADER)
    return buffer[idx:] if idx >= 0 else b""


# ---------------------------------------------------------------------------
# Microduck feedback decoding
# ---------------------------------------------------------------------------

def decode_feedback(params: bytes) -> dict[str, float]:
    """Decode the 7-byte feedback block (registers 56..62) into physical units.

    Returns dict with:
        position:  raw 12-bit servo position (0..4095, signed via bit15)
        speed:     raw signed speed (bit15 = direction)
        load:      raw signed load duty (bit10 = direction)
        voltage:   volts (0.1 V/LSB)
        temperature: degrees C
    """
    if len(params) < FEEDBACK_LENGTH:
        raise ProtocolError(f"feedback block needs {FEEDBACK_LENGTH} bytes, got {len(params)}")
    position = le_to_u16(params, 0)
    speed = le_to_u16(params, 2)
    load = le_to_u16(params, 4)
    return {
        "position_raw": decode_signed(position),
        "position": decode_signed(position) * (360.0 / 4096.0),  # deg, 0.088 deg/LSB
        "speed_raw": decode_signed(speed),
        "load_raw": decode_signed(load, LOAD_DIRECTION_BIT),
        "voltage": params[6] * 0.1,
        "temperature": params[7] if len(params) > 7 else 0,
    }
