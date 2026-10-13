# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright 2026 邓陈龙
#
"""Mock serial transport for the SCS driver layer unit tests.

Implements the servo side of the half-duplex TTL bus: it consumes instruction
packets written by the host, maintains a virtual register file per servo ID
(STS memory table subset), and produces the status packets the real servos
would return. The host-side driver (`ScsBus`) is tested against this without
any hardware.
"""

from __future__ import annotations

from scs_protocol import (
    BROADCAST_ID,
    HEADER,
    INST_PING,
    INST_READ,
    INST_REG_WRITE,
    INST_WRITE,
    INST_SYNC_READ,
    INST_SYNC_WRITE,
    REG_PRESENT_POSITION,
    checksum,
    parse_status,
)


class MockSerial:
    """In-memory serial port the host writes to and reads from.

    `Serial` on the host is used through write() / read(); this mock captures
    written instruction packets, updates the virtual servos, and queues the
    response bytes so read() returns them as a real serial port would.
    """

    def __init__(self, servos: list[int] | None = None) -> None:
        # id -> register dict (address -> int value byte or 16-bit value)
        self.registers: dict[int, dict[int, int]] = {}
        for sid in servos or []:
            self.add_servo(sid)
        self._rx: bytearray = bytearray()
        self.written_frames: list[bytes] = []
        self.open = True
        self.timeout = 0.01  # matches serial.Serial(timeout=...) semantics

    # -- virtual servo setup -------------------------------------------------

    def add_servo(self, servo_id: int, position: int = 2048, voltage: int = 60) -> None:
        """Register a virtual servo with plausible STS defaults."""
        self.registers[servo_id] = {
            5: servo_id,            # servo ID
            6: 0,                   # baud rate (1 Mbps)
            8: 0,                   # response status level 0 (no write ack)
            40: 0,                  # torque switch off
            41: 0,                  # acceleration
            42: position,           # goal position (16-bit)
            46: 0,                  # running speed
            REG_PRESENT_POSITION: position,
            58: 0,                  # present speed
            60: 0,                  # present load
            62: voltage,            # present voltage (0.1 V)
            63: 25,                 # present temperature
            69: 0,                  # present current
        }

    def set_feedback(self, servo_id: int, position: int | None = None,
                     speed: int | None = None, voltage: int | None = None,
                     load: int | None = None, current: int | None = None) -> None:
        """Poke feedback values as the bench would produce them."""
        regs = self.registers[servo_id]
        if position is not None:
            regs[REG_PRESENT_POSITION] = position & 0xFFFF
        if speed is not None:
            regs[58] = speed & 0xFFFF
        if voltage is not None:
            regs[62] = voltage & 0xFF
        if load is not None:
            regs[60] = load & 0xFFFF
        if current is not None:
            regs[69] = current & 0xFFFF

    # -- serial-port surface ---------------------------------------------------

    def write(self, data: bytes) -> int:
        if not self.open:
            raise OSError("port closed")
        self.written_frames.append(bytes(data))
        self._handle(bytes(data))
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if not self.open:
            raise OSError("port closed")
        out = bytes(self._rx[:size])
        del self._rx[:size]
        return out

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def close(self) -> None:
        self.open = False

    # -- servo-side packet handling --------------------------------------------

    def _status(self, servo_id: int, error: int, params: bytes = b"") -> None:
        length = len(params) + 2
        body = bytes((servo_id, length, error)) + params
        self._rx += HEADER + body + bytes((checksum(body),))

    def _handle(self, data: bytes) -> None:
        """Parse one instruction packet and emulate the servo responses."""
        # Tolerate leading noise like the real firmware parser
        idx = data.find(HEADER)
        if idx < 0:
            return
        frame = data[idx:]
        if len(frame) < 6:
            return
        servo_id = frame[2]
        length = frame[3]
        instruction = frame[4]
        # LEN counts instruction + params + checksum; params end just before it
        params = frame[5:2 + length + 1]
        if instruction == INST_PING:
            # Response level 0 still answers PING; unknown IDs stay silent
            if servo_id in self.registers:
                self._status(servo_id, 0)
        elif instruction == INST_READ:
            start, count = params[0], params[1]
            regs = self.registers.get(servo_id)
            if regs is None:
                return  # no such servo: silence
            payload = self._read_regs(regs, start, count)
            self._status(servo_id, 0, payload)
        elif instruction == INST_WRITE:
            start = params[0]
            self._write_regs(servo_id, start, params[1:])
        elif instruction == INST_REG_WRITE:
            start = params[0]
            # staged: real firmware stores it; effect identical for the mock
            self._write_regs(servo_id, start, params[1:])
        elif instruction == INST_SYNC_WRITE:
            start, dlen = params[0], params[1]
            body = params[2:]
            for i in range(0, len(body), dlen + 1):
                sid = body[i]
                self._write_regs(sid, start, body[i + 1:i + 1 + dlen])
            # broadcast: no response
        elif instruction == INST_SYNC_READ:
            start, count = params[0], params[1]
            for sid in params[2:]:
                regs = self.registers.get(sid)
                if regs is None:
                    continue  # absent servo stays silent, like the real bus
                payload = self._read_regs(regs, start, count)
                self._status(sid, 0, payload)
        # ACTION and others: no response at status level 0

    @staticmethod
    def _read_regs(regs: dict[int, int], start: int, count: int) -> bytes:
        """Read `count` bytes starting at `start`, two-byte regs little-endian.

        The STS table lays out: pos_lo(56) pos_hi(57) spd_lo(58) spd_hi(59)
        load_lo(60) load_hi(61) volt(62) temp(63) ... cur_lo(69) cur_hi(70).
        """
        out = bytearray()
        i = start
        while i < start + count:
            # 16-bit registers: high byte only read when the read spans both
            if i in (56, 58, 60, 67, 69) and i + 1 < start + count:
                val = regs.get(i, 0)
                out += bytes((val & 0xFF, (val >> 8) & 0xFF))
                i += 2
            else:
                out.append(regs.get(i, 0) & 0xFF)
                i += 1
        return bytes(out)

    def _write_regs(self, servo_id: int, start: int, data: bytes) -> None:
        regs = self.registers.get(servo_id)
        if regs is None:
            return
        i = start
        j = 0
        while j < len(data):
            if i in (42, 46, 16, 48) and j + 1 < len(data):
                regs[i] = data[j] | (data[j + 1] << 8)
                if i == 42:
                    # goal position write also moves present position (ideal servo)
                    regs[REG_PRESENT_POSITION] = regs[42]
                j += 2
                i += 2
            else:
                regs[i] = data[j]
                j += 1
                i += 1


def parse_all_status(rx: bytes) -> list[tuple[int, int, bytes]]:
    """Split a concatenated RX stream into status packets."""
    frames: list[tuple[int, int, bytes]] = []
    buf = rx
    while True:
        buf = buf.lstrip(b"\x00")  # tolerate inter-frame gaps
        idx = buf.find(HEADER)
        if idx < 0 or len(buf) < idx + 6:
            break
        buf = buf[idx:]
        length = buf[3]
        total = 4 + length
        if len(buf) < total:
            break
        sid, err, params = parse_status(buf[:total])
        frames.append((sid, err, params))
        buf = buf[total:]
    return frames
