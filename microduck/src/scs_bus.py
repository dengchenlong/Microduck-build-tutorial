# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright 2026 邓陈龙
#
"""High-level Feetech SCS bus driver for the Microduck 14-servo HD-1910 build.

Wraps `scs_protocol` frame codecs over a `serial.Serial`-like port (pyserial
is the only third-party import at runtime; the unit tests swap in the mock).
stdlib-only constraint applies to the protocol layer itself.

Bus semantics (half-duplex TTL, status level 0):
- WRITE / REG WRITE / SYNC WRITE at response level 0 produce no reply.
- PING, READ and SYNC READ each produce one status packet per addressed servo.
- SYNC READ returns packets in the order the servos answer (ascending ID on
  a healthy bus); the caller maps responses back by the ID byte, not position.
"""

from __future__ import annotations

import glob
import os
from typing import Any

import scs_protocol as sp


# OpenRB-150 acts as USB-CDC passthrough; by-id symlink is the stable handle.
OPENRB_BY_ID_GLOB = "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150*"
# Fallback order when no by-id entry exists (OpenRB enumerates as ttyACM).
SERIAL_FALLBACKS = ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0")
DEFAULT_PORT = "/dev/ttyACM0"


def resolve_serial_port(serial_port: str = "auto") -> str:
    """Pick the OpenRB-150 serial device.

    "auto" probes, in order:
      1. /dev/serial/by-id/usb-ROBOTIS_OpenRB-150* (stable, sorted)
      2. ttyACM0/1, ttyUSB0 fallbacks
    Explicit paths pass through untouched.
    """
    if serial_port != "auto":
        return serial_port
    candidates = sorted(glob.glob(OPENRB_BY_ID_GLOB)) + list(SERIAL_FALLBACKS)
    for path in candidates:
        if os.path.exists(path):
            return path
    return DEFAULT_PORT


class ScsBus:
    """Instruction-layer driver over one serial port."""

    def __init__(self, serial_port: str = "auto", baudrate: int = 1_000_000,
                 timeout: float = 0.05) -> None:
        self.port_path = resolve_serial_port(serial_port)
        import serial  # pyserial; deferred so tests can run without a port
        self.ser: Any = serial.Serial(self.port_path, baudrate=baudrate, timeout=timeout)
        self._rx = bytearray()

    # -- low-level ------------------------------------------------------------

    def _transact(self, frame: bytes, expect: int) -> list[tuple[int, int, bytes]]:
        """Send one instruction frame, collect `expect` status packets."""
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        statuses: list[tuple[int, int, bytes]] = []
        # 6-byte minimal frame; grow until `expect` complete packets or give up.
        import time
        deadline = time.monotonic() + max(self.ser.timeout * (expect + 2), 0.1)
        while len(statuses) < expect and time.monotonic() < deadline:
            chunk = self.ser.read(64)
            if chunk:
                self._rx.extend(chunk)
                statuses.extend(self._drain())
        return statuses

    def _drain(self) -> list[tuple[int, int, bytes]]:
        out: list[tuple[int, int, bytes]] = []
        while True:
            buf = bytes(self._rx)
            idx = buf.find(sp.HEADER)
            if idx < 0:
                self._rx.clear()
                return out
            if idx:
                del self._rx[:idx]
                continue
            if len(self._rx) < 6:
                return out
            declared = 4 + self._rx[3]
            if len(self._rx) < declared:
                return out
            frame = bytes(self._rx[:declared])
            del self._rx[:declared]
            out.append(sp.parse_status(frame))

    # -- instructions ----------------------------------------------------------

    def ping(self, servo_id: int) -> bool:
        """True when the servo answers PING."""
        statuses = self._transact(sp.build_ping(servo_id), 1)
        return any(sid == servo_id and err == 0 for sid, err, _ in statuses)

    def scan(self, ids: range | list[int] = range(1, 15)) -> list[int]:
        """Return the subset of `ids` that answers PING."""
        return [sid for sid in ids if self.ping(sid)]

    def write(self, servo_id: int, start: int, data: bytes) -> None:
        self._transact(sp.build_write(servo_id, start, data), 0)

    def write_byte(self, servo_id: int, reg: int, value: int) -> None:
        self.write(servo_id, reg, bytes((value & 0xFF,)))

    def sync_write(self, start: int, data_length: int,
                   entries: list[tuple[int, bytes]]) -> None:
        self._transact(sp.build_sync_write(start, data_length, entries), 0)

    def read(self, servo_id: int, start: int, length: int) -> bytes:
        statuses = self._transact(sp.build_read(servo_id, start, length), 1)
        if not statuses:
            raise sp.TimeoutError_(f"no status from servo {servo_id}")
        sid, err, params = statuses[0]
        if sid != servo_id:
            raise sp.ProtocolError(f"status from id {sid}, expected {servo_id}")
        if err:
            raise sp.ProtocolError(f"servo {servo_id} error status {err}")
        return params

    def sync_read(self, servo_ids: list[int], start: int, length: int) -> dict[int, bytes]:
        """SYNC READ: one transaction, returns {id: params} for answering servos."""
        frame = sp.build_sync_read(servo_ids, start, length)
        statuses = self._transact(frame, len(servo_ids))
        got: dict[int, bytes] = {}
        for sid, _err, params in statuses:
            if sid in servo_ids:
                got[sid] = params
        return got

    def sync_write_goal_position(self, ids: list[int], positions: list[int]) -> None:
        """positions: raw servo units (12-bit scale, signed). 3 bytes/servo:
        acceleration(41)=0 (max), goal position(42..43) little-endian."""
        entries = [
            (sid, bytes((0,)) + sp.u16_to_le(sp.encode_signed(pos) & 0xFFFF))
            for sid, pos in zip(ids, positions)
        ]
        self.sync_write(sp.REG_ACCELERATION, 3, entries)

    def sync_read_feedback(self, ids: list[int]) -> dict[int, dict[str, float]]:
        """position/speed/load/voltage (physical-ish units) in one bus round."""
        raw = self.sync_read(ids, sp.FEEDBACK_START, sp.FEEDBACK_LENGTH)
        return {sid: sp.decode_feedback(params) for sid, params in raw.items()}

    def set_torque(self, ids: list[int], enable: bool) -> None:
        for sid in ids:
            self.write_byte(sid, sp.REG_TORQUE_ENABLE, 1 if enable else 0)

    def close(self) -> None:
        self.ser.close()
