# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright 2026 邓陈龙
#
"""Unit tests for the SCS frame protocol layer (ticket #04).

Runs against the in-memory MockSerial — no servo hardware, no real port.
Execute from the repo with:

    cd microduck && python3 -m unittest discover -s tests -v

(works on any Python >= 3.10; only stdlib is imported by the codec)
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
sys.path.insert(0, str(SRC))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import scs_protocol as sp  # noqa: E402
from mock_serial import MockSerial, parse_all_status  # noqa: E402


def hexs(b: bytes) -> str:
    return b.hex(" ")


class ChecksumTests(unittest.TestCase):
    """Checksum definition: ~(ID+LEN+INSTR+params) & 0xFF, per FT-SCS spec."""

    def test_checksum_wiki_example_syncwrite(self):
        # Wiki example 7: sync write to 4 servos, ID=0xFE, LEN=(2+1)*4+4=16,
        # instr 0x83, start 0x2A, dlen 2, then 4x(ID + 0x00 0x08 0x00 0xE8...).
        # Reconstruct a 1-servo variant and verify the checksum definition
        # against a hand-computed value instead.
        # ID=0x01, LEN=0x05, INSTR=0x03, params 0x2A 0x00 0x08:
        # sum = 1+5+3+0x2A+0+8 = 0x3B -> ~ = 0xC4
        body = bytes((0x01, 0x05, 0x03, 0x2A, 0x00, 0x08))
        self.assertEqual(sp.checksum(body), 0xC4)

    def test_checksum_overflow_uses_low_byte(self):
        # Sum 0x2FD -> low byte 0xFD -> ~ -> 0x02
        self.assertEqual(sp.checksum(bytes((0xFF, 0xFF, 0xFF, 0x00))), 0x02)

    def test_checksum_inverts_simple_sum(self):
        self.assertEqual(sp.checksum(bytes((1, 2, 3))), 0xF9)

    def test_ping_frame_layout(self):
        # PING id 1: FF FF 01 02 01 + ~(1+2+1)=FB
        self.assertEqual(hexs(sp.build_ping(1)), "ff ff 01 02 01 fb")
        # PING id 14: ~(14+2+1)=~17=0xEE
        self.assertEqual(hexs(sp.build_ping(14)), "ff ff 0e 02 01 ee")


class FrameConstructionTests(unittest.TestCase):
    def test_length_field_counts_instr_and_checksum(self):
        # WRITE 2 bytes at reg 40: params = [start, d0, d1] -> LEN = 3+2 = 5
        f = sp.build_write(3, 40, b"\x01\x00")
        self.assertEqual(f[3], 5)
        self.assertEqual(len(f), 4 + 5)

    def test_read_frame(self):
        # READ 7 bytes from reg 56 on id 2: params 0x38 0x07, LEN 4
        # checksum = ~(2+4+2+0x38+7) = ~0x47 = 0xB8
        f = sp.build_read(2, 56, 7)
        self.assertEqual(hexs(f), "ff ff 02 04 02 38 07 b8")

    def test_reg_write_and_action(self):
        rw = sp.build_reg_write(5, 42, b"\x00\x08")
        self.assertEqual(rw[4], sp.INST_REG_WRITE)
        act = sp.build_action()
        self.assertEqual(act[2], sp.BROADCAST_ID)
        self.assertEqual(act[4], sp.INST_ACTION)

    def test_sync_write_layout(self):
        # 2 servos, 3 data bytes each, start 41 (0x29):
        # params = 29 03 | id1 d d d | id2 d d d (8 bytes) -> LEN = 8+2 = 10? No:
        # LEN counts params(8) + instr(1) + checksum(1) = 10... spec: (L+1)*n+4
        # = 4*2+4 = 12 == params(8) + instr(1) + checksum(1) + id(1) + len(1) + 0xFE? = 12
        f = sp.build_sync_write(41, 3, [(1, b"\x00\x00\x08"), (2, b"\x00\x00\x00")])
        self.assertEqual(f[2], 0xFE)
        self.assertEqual(f[3], 12)
        self.assertEqual(f[4], 0x83)
        self.assertEqual(f[5:7], b"\x29\x03")
        self.assertEqual(f[7], 1)
        self.assertEqual(f[8:11], b"\x00\x00\x08")
        self.assertEqual(f[11], 2)

    def test_sync_write_rejects_mixed_lengths(self):
        with self.assertRaises(sp.ProtocolError):
            sp.build_sync_write(41, 3, [(1, b"\x00\x00"), (2, b"\x00\x00\x08")])

    def test_sync_write_rejects_empty(self):
        with self.assertRaises(sp.ProtocolError):
            sp.build_sync_write(41, 3, [])

    def test_sync_read_layout(self):
        # sync read ids [1,2,14], start 56, len 7:
        # params = 38 07 01 02 0E -> LEN = 5+2 = 7
        f = sp.build_sync_read([1, 2, 14], 56, 7)
        self.assertEqual(f[2], 0xFE)
        self.assertEqual(f[3], 7)
        self.assertEqual(f[4], 0x82)
        self.assertEqual(f[5], 56)
        self.assertEqual(f[6], 7)
        self.assertEqual(list(f[7:10]), [1, 2, 14])
        # checksum covers id+len+instr+params
        self.assertEqual(f[-1], sp.checksum(f[2:-1]))

    def test_sync_read_single_id(self):
        f = sp.build_sync_read([7], 40, 1)
        self.assertEqual(f[3], 5)  # params 28 01 07 = 3 bytes -> LEN 5

    def test_sync_read_needs_ids(self):
        with self.assertRaises(sp.ProtocolError):
            sp.build_sync_read([], 56, 7)

    def test_id_range_enforced(self):
        with self.assertRaises(sp.ProtocolError):
            sp.build_ping(254 + 1)
        with self.assertRaises(sp.ProtocolError):
            sp.build_ping(-1)


class ByteOrderTests(unittest.TestCase):
    def test_u16_little_endian_on_wire(self):
        # STS/SMS layer: low byte first. 0x0800 -> 00 08
        self.assertEqual(sp.u16_to_le(0x0800), b"\x00\x08")
        self.assertEqual(sp.u16_to_le(0x00FF), b"\xff\x00")
        with self.assertRaises(sp.ProtocolError):
            sp.u16_to_le(0x10000)

    def test_signed_position_roundtrip(self):
        # bit15 = direction bit, 11-bit magnitude
        for v in (0, 1, 2047, -1, -2047):
            enc = sp.encode_signed(v)
            self.assertLessEqual(enc, 0xFFFF)
            self.assertEqual(sp.decode_signed(enc), v)
        # -100 -> magnitude 100 + bit15
        self.assertEqual(sp.encode_signed(-100), 0x8064)
        self.assertEqual(sp.decode_signed(0x8064), -100)

    def test_load_uses_bit10(self):
        self.assertEqual(sp.decode_signed(0x0400 | 5, sp.LOAD_DIRECTION_BIT), -5)
        self.assertEqual(sp.decode_signed(5, sp.LOAD_DIRECTION_BIT), 5)


class StatusParsingTests(unittest.TestCase):
    def test_parse_minimal_status(self):
        # PING reply from id 3: FF FF 03 02 00 checksum
        frame = bytes.fromhex("ff ff 03 02 00 fa")
        sid, err, params = sp.parse_status(frame)
        self.assertEqual((sid, err, params), (3, 0, b""))

    def test_parse_status_with_payload(self):
        # READ reply: 2 bytes 0x00 0x08 -> checksum ~(2+4+0+0+8)=~0x0E=0xF1
        frame = bytes.fromhex("ff ff 02 04 00 00 08 f1")
        sid, err, params = sp.parse_status(frame)
        self.assertEqual(sid, 2)
        self.assertEqual(err, 0)
        self.assertEqual(params, b"\x00\x08")

    def test_checksum_mismatch_raises(self):
        frame = bytearray(bytes.fromhex("ff ff 03 02 00 fb"))
        frame[-1] ^= 0xFF
        with self.assertRaises(sp.ProtocolError):
            sp.parse_status(bytes(frame))

    def test_truncated_frame_raises(self):
        with self.assertRaises(sp.ProtocolError):
            sp.parse_status(b"\xff\xff\x03")
        # declared length longer than actual bytes
        with self.assertRaises(sp.ProtocolError):
            sp.parse_status(bytes.fromhex("ff ff 03 06 00 00"))

    def test_missing_header_raises(self):
        with self.assertRaises(sp.ProtocolError):
            sp.parse_status(b"\xaa\xbb\x03\x02\x00\xfb")

    def test_strip_leading_noise(self):
        buf = b"\x00\xaa\xff" + bytes.fromhex("ff ff 03 02 00 fb")
        cleaned = sp.strip_leading_noise(buf)
        self.assertEqual(cleaned[:2], b"\xff\xff")


class MockBusTransactionTests(unittest.TestCase):
    """End-to-end through the mock: instruction in, status out."""

    def setUp(self):
        self.ser = MockSerial([1, 2, 3, 14])
        self.ser.set_feedback(1, position=1024, voltage=62)
        self.ser.set_feedback(2, position=0x8000 | 20, speed=100)  # bit15: dir bit

    def test_ping_gets_reply(self):
        self.ser.write(sp.build_ping(1))
        frames = parse_all_status(bytes(self.ser.read(64)))
        self.assertEqual([(s, e) for s, e, _ in frames], [(1, 0)])

    def test_ping_unknown_id_silent(self):
        self.ser.write(sp.build_ping(9))
        self.assertEqual(self.ser.read(64), b"")

    def test_read_returns_registers(self):
        self.ser.write(sp.build_read(1, 62, 1))  # present voltage
        (_, _, params), = parse_all_status(bytes(self.ser.read(64)))
        self.assertEqual(params, b"\x3e")  # 62 * 0.1 V = 6.2 V
        self.assertAlmostEqual(params[0] * 0.1, 6.2)

    def test_write_goal_position_updates_feedback(self):
        # write goal position 0x0800 (=2048) at reg 42, little-endian
        self.ser.write(sp.build_write(1, 42, sp.u16_to_le(0x0800)))
        self.ser.read(64)  # status level 0: no reply expected
        self.assertEqual(self.ser.registers[1][42], 0x0800)
        self.assertEqual(self.ser.registers[1][sp.REG_PRESENT_POSITION], 0x0800)

    def test_sync_write_hits_all_servos(self):
        entries = [(sid, bytes((0,)) + sp.u16_to_le(sp.encode_signed(-100) & 0xFFFF))
                   for sid in (1, 2, 3)]
        self.ser.write(sp.build_sync_write(41, 3, entries))
        self.ser.read(64)
        expected = sp.encode_signed(-100)  # 0x8064
        for sid in (1, 2, 3):
            self.assertEqual(self.ser.registers[sid][42], expected)
            self.assertEqual(self.ser.registers[sid][sp.REG_PRESENT_POSITION], expected)
        # untouched servo
        self.assertEqual(self.ser.registers[14][42], 2048)

    def test_sync_read_one_packet_per_servo(self):
        self.ser.write(sp.build_sync_read([1, 2, 14], 56, 7))
        frames = parse_all_status(bytes(self.ser.read(64)))
        self.assertEqual([s for s, _, _ in frames], [1, 2, 14])
        by_id = {s: p for s, _, p in frames}
        # feedback block: pos_lo pos_hi spd_lo spd_hi load_lo load_hi volt
        self.assertEqual(sp.le_to_u16(by_id[1], 0), 1024)
        self.assertEqual(by_id[1][6], 62)  # voltage byte
        self.assertEqual(sp.le_to_u16(by_id[2], 0), 0x8014)
        self.assertEqual(sp.decode_signed(sp.le_to_u16(by_id[2], 0)), -20)
        self.assertEqual(sp.le_to_u16(by_id[2], 2), 100)  # speed

    def test_sync_read_skips_absent_servo(self):
        self.ser.write(sp.build_sync_read([1, 99], 56, 7))
        frames = parse_all_status(bytes(self.ser.read(64)))
        self.assertEqual([s for s, _, _ in frames], [1])

    def test_full_feedback_decode(self):
        self.ser.set_feedback(3, position=3000, speed=-5, voltage=74, load=0x0400 | 9)
        self.ser.write(sp.build_read(3, sp.FEEDBACK_START, sp.FEEDBACK_LENGTH))
        (_, _, params), = parse_all_status(bytes(self.ser.read(64)))
        fb = sp.decode_feedback(params)
        self.assertEqual(fb["position_raw"], 3000)
        self.assertAlmostEqual(fb["position"], 3000 * 360.0 / 4096.0)
        # mock stores speed as raw u16 (0xFFFB = 65531); driver re-decodes it
        self.assertEqual(fb["speed_raw"], sp.decode_signed(0xFFFB))
        self.assertEqual(fb["load_raw"], -9)
        self.assertAlmostEqual(fb["voltage"], 7.4)


class SyncReadPackingTests(unittest.TestCase):
    """Ticket acceptance: sync_read 组包正确 — byte-exact packet checks."""

    def test_sync_read_all_14_motors(self):
        ids = list(range(1, 15))
        f = sp.build_sync_read(ids, sp.FEEDBACK_START, sp.FEEDBACK_LENGTH)
        self.assertEqual(f[0:2], b"\xff\xff")
        self.assertEqual(f[2], 0xFE)                       # broadcast
        self.assertEqual(f[3], len(ids) + 4)               # n + 4
        self.assertEqual(f[4], 0x82)                       # SYNC READ
        self.assertEqual(f[5], sp.FEEDBACK_START)          # 56
        self.assertEqual(f[6], sp.FEEDBACK_LENGTH)         # 7
        self.assertEqual(list(f[7:21]), ids)               # id list 1..14
        self.assertEqual(f[-1], sp.checksum(f[2:-1]))      # trailing checksum

    def test_sync_read_packet_matches_mock_expectation(self):
        ser = MockSerial([1, 2])
        ser.write(sp.build_sync_read([1, 2], 56, 7))
        # mock must have parsed the instruction (i.e. framing is self-consistent)
        frames = parse_all_status(bytes(ser.read(64)))
        self.assertEqual(len(frames), 2)

    def test_sync_write_goal_position_packet(self):
        f = sp.build_sync_write(41, 3, [(1, b"\x00\x40\x0f"), (14, b"\x00\xc0\x00")])
        self.assertEqual(f[4], 0x83)
        self.assertEqual(f[5], 41)   # start = acceleration reg (3-byte block)
        self.assertEqual(f[6], 3)    # per-servo data length
        # entry 1: id, acc, pos_lo, pos_hi = 0x0F40 = 3904
        self.assertEqual(list(f[7:11]), [1, 0, 0x40, 0x0F])
        # entry 2: pos 0x00C0 = 192
        self.assertEqual(list(f[11:15]), [14, 0, 0xC0, 0x00])


class SerialPortDetectionTests(unittest.TestCase):
    """Ticket acceptance: 设备探测按 OpenRB-150 串口路径."""

    def _with_fake_dev(self, paths: list[str]):
        import scs_bus

        seen = []

        def fake_glob(pattern):
            seen.append(pattern)
            return [p for p in paths if "by-id" in pattern and p.startswith("/dev/serial")]

        def fake_exists(p):
            return p in paths

        orig_glob, orig_exists = scs_bus.glob.glob, scs_bus.os.path.exists
        scs_bus.glob.glob = fake_glob
        scs_bus.os.path.exists = fake_exists
        try:
            return scs_bus.resolve_serial_port("auto"), seen
        finally:
            scs_bus.glob.glob = orig_glob
            scs_bus.os.path.exists = orig_exists

    def test_prefers_by_id_openrb_path(self):
        port, seen = self._with_fake_dev(
            ["/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_12345-if00", "/dev/ttyACM0"]
        )
        self.assertEqual(port, "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_12345-if00")
        self.assertIn(scs_bus_g := "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150*", seen)

    def test_falls_back_to_ttyacm(self):
        port, _ = self._with_fake_dev(["/dev/ttyACM0"])
        self.assertEqual(port, "/dev/ttyACM0")

    def test_falls_back_to_ttyusb(self):
        port, _ = self._with_fake_dev(["/dev/ttyUSB0"])
        self.assertEqual(port, "/dev/ttyUSB0")

    def test_default_when_nothing_found(self):
        import scs_bus
        port, _ = self._with_fake_dev([])
        self.assertEqual(port, scs_bus.DEFAULT_PORT)

    def test_explicit_path_passthrough(self):
        import scs_bus
        self.assertEqual(scs_bus.resolve_serial_port("/dev/ttyACM2"), "/dev/ttyACM2")

    def test_glob_pattern_matches_openrb_not_robotis_only(self):
        # by-id glob must cover OpenRB-150 variants (different suffixes)
        import scs_bus
        import fnmatch
        self.assertTrue(fnmatch.fnmatch(
            "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_8AC123-if00", scs_bus.OPENRB_BY_ID_GLOB))
        self.assertTrue(fnmatch.fnmatch(
            "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150-if00", scs_bus.OPENRB_BY_ID_GLOB))


if __name__ == "__main__":
    unittest.main(verbosity=2)
