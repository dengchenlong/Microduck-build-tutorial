# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright 2026 邓陈龙
#
"""Driver-level tests: ScsBus over the mock serial port (no pyserial, no HW).

ScsBus imports pyserial lazily inside __init__, so tests inject a fake module
whose Serial() returns the MockSerial. This exercises the real transaction
framing (request -> status packets -> parse) end to end.
"""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
sys.path.insert(0, str(SRC))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import scs_protocol as sp  # noqa: E402
from mock_serial import MockSerial  # noqa: E402


def install_fake_pyserial(mock: MockSerial) -> None:
    fake = types.ModuleType("serial")
    fake.Serial = lambda *a, **kw: mock
    sys.modules["serial"] = fake


class ScsBusOverMockTests(unittest.TestCase):
    def setUp(self):
        # fresh module state so the fake pyserial does not leak between tests
        sys.modules.pop("serial", None)
        sys.modules.pop("scs_bus", None)
        self.mock = MockSerial([1, 2, 3, 14])
        self.mock.set_feedback(1, position=1024)
        self.mock.set_feedback(2, position=2048, voltage=65)
        install_fake_pyserial(self.mock)
        import scs_bus
        self.scs_bus = scs_bus

    def tearDown(self):
        sys.modules.pop("serial", None)
        sys.modules.pop("scs_bus", None)

    def _make(self):
        return self.scs_bus.ScsBus(serial_port="/dev/ttyFAKE", timeout=0.01)

    def test_ping_true_for_present_servo(self):
        bus = self._make()
        self.assertTrue(bus.ping(1))
        self.assertTrue(bus.ping(14))
        bus.close()

    def test_ping_false_for_absent_servo(self):
        bus = self._make()
        self.assertFalse(bus.ping(9))
        bus.close()

    def test_scan_returns_only_present_ids(self):
        bus = self._make()
        self.assertEqual(bus.scan(range(1, 6)), [1, 2, 3])
        bus.close()

    def test_write_byte_reaches_register(self):
        bus = self._make()
        bus.write_byte(1, sp.REG_TORQUE_ENABLE, 1)
        self.assertEqual(self.mock.registers[1][40], 1)
        bus.close()

    def test_sync_write_goal_position(self):
        bus = self._make()
        ids = [1, 2, 3, 14]
        positions = [0, 100, -100, 2048]
        bus.sync_write_goal_position(ids, positions)
        for sid, pos in zip(ids, positions):
            self.assertEqual(self.mock.registers[sid][42], sp.encode_signed(pos) & 0xFFFF)
        bus.close()

    def test_sync_read_feedback_returns_all_motors(self):
        bus = self._make()
        ids = [1, 2, 14]
        fb = bus.sync_read_feedback(ids)
        self.assertEqual(set(fb), set(ids))
        self.assertEqual(fb[1]["position_raw"], 1024)
        self.assertEqual(fb[2]["position_raw"], 2048)
        self.assertAlmostEqual(fb[2]["voltage"], 6.5)
        bus.close()

    def test_sync_read_single_transaction_on_wire(self):
        bus = self._make()
        bus.sync_read_feedback([1, 2, 3, 14])
        # exactly ONE instruction packet written (one bus round)
        self.assertEqual(len(self.mock.written_frames), 1)
        frame = self.mock.written_frames[0]
        self.assertEqual(frame[4], sp.INST_SYNC_READ)
        self.assertEqual(frame[2], sp.BROADCAST_ID)
        bus.close()

    def test_set_torque_roundtrip(self):
        bus = self._make()
        ids = [1, 2, 3]
        bus.set_torque(ids, True)
        for sid in ids:
            self.assertEqual(self.mock.registers[sid][40], 1)
        bus.set_torque(ids, False)
        for sid in ids:
            self.assertEqual(self.mock.registers[sid][40], 0)
        bus.close()

    def test_read_timeout_raises(self):
        bus = self._make()
        with self.assertRaises(sp.TimeoutError_):
            bus.read(99, 56, 7)  # absent servo -> no status
        bus.close()

    def test_torque_then_goal_motion_is_observed(self):
        """Full control round: torque on, command position, read it back."""
        bus = self._make()
        ids = [1, 2, 3, 14]
        bus.set_torque(ids, True)
        bus.sync_write_goal_position(ids, [10] * len(ids))
        fb = bus.sync_read_feedback(ids)
        for sid in ids:
            self.assertEqual(fb[sid]["position_raw"], 10)
        bus.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
