"""Run: python -m unittest discover -s GroundStation/tests -v (no board needed)."""
import os
import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt6.QtWidgets import QApplication, QMessageBox
from core.packet_encoder import encode_ready
from core.packet_decoder import decode_packet, decode_ack, TELEM_FORMAT, ACK_REJECTED
from ui.widgets.arm_panel import ArmPanel
from ui.widgets.command_panel import CommandPanel
from ui.widgets.pyro_panel import PyroPanel


def telemetry(state):
    values = [0] * 29
    values[0:5] = [0xAA, 0x55, 1, 1000, state]
    values[15] = 32767
    values[22:25] = [1, 1, 1]
    values[26] = 0x7F  # healthy sensors, present/recording SD
    raw = bytearray(struct.pack(TELEM_FORMAT, *values))
    struct.pack_into('<H', raw, len(raw) - 2, sum(raw[:-2]) & 0xFFFF)
    return decode_packet(bytes(raw))


class ReadyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_ready_packet_and_state(self):
        packet = encode_ready(seq=42)
        self.assertEqual(packet[:5], bytes([0xBB, 0x44, 0x15, 0, 42]))
        self.assertEqual(struct.unpack_from('<H', packet, 5)[0], sum(packet[:5]))
        self.assertEqual(telemetry(7).state_name, 'READY_FOR_FLIGHT')

    def test_rejection_ack_round_trip(self):
        prefix = bytes([0xAC, 0x4B, 42, 0x14 | ACK_REJECTED])
        ack = decode_ack(prefix + struct.pack('<H', sum(prefix)))
        self.assertEqual(ack.cmd_seq, 42)
        self.assertEqual(ack.cmd_type & 0x7F, 0x14)
        self.assertTrue(ack.cmd_type & ACK_REJECTED)

    def test_arm_ready_cancel_and_airborne_controls(self):
        panel = ArmPanel()
        for state, arm, ready, disarm, attitude in [
            (0, True, False, False, True),
            (1, False, True, True, True),
            (7, False, False, True, False),
            (2, False, False, False, False),
            (5, False, False, False, False),
            (6, False, False, False, False),
            (0, True, False, False, True),
        ]:
            panel.update_data(telemetry(state))
            self.assertEqual([panel._arm_btn.isEnabled(), panel._flight_ready_btn.isEnabled(),
                              panel._disarm_btn.isEnabled(), panel._att_switch.isEnabled()],
                             [arm, ready, disarm, attitude])
        panel.close()

    def test_ready_requires_explicit_confirmation(self):
        panel = ArmPanel()
        panel.update_data(telemetry(1))
        sent = []
        panel.ready_requested.connect(lambda: sent.append(True))
        with patch.object(QMessageBox, 'question', return_value=QMessageBox.StandardButton.No):
            panel._flight_ready_btn.click()
        self.assertEqual(sent, [])
        with patch.object(QMessageBox, 'question', return_value=QMessageBox.StandardButton.Yes):
            panel._flight_ready_btn.click()
        self.assertEqual(sent, [True])
        # Sending READY must not optimistically change the displayed flight state.
        self.assertEqual(panel._state_name, 'ARMED')
        panel.close()

    def test_maintenance_locked_from_ready_until_reset(self):
        panel = CommandPanel()
        for state in [0, 1, 7, 2, 3, 4, 5, 6, 0]:
            panel.update_data(telemetry(state))
            for button in panel._bench_buttons + [panel._sd_btn, panel._cam_btn, panel._att_demo_btn]:
                self.assertEqual(button.isEnabled(), state in (0, 1), (state, button.text()))
            self.assertEqual(panel._reset_btn.isEnabled(), state in (0, 1, 6))
            for button in panel._fire_btns:
                self.assertEqual(button.isEnabled(), state in (1, 2, 3, 4, 5, 7))
        panel.close()

    def test_ready_does_not_mark_parachute_fired(self):
        panel = PyroPanel()
        for state in [0, 1, 7, 2, 3, 4]:
            panel.update_data(telemetry(state))
            self.assertNotIn(1, panel._fired)
        panel.update_data(telemetry(5))
        self.assertIn(1, panel._fired)
        panel.update_data(telemetry(0))
        self.assertEqual(panel._fired, set())
        panel.close()


if __name__ == '__main__':
    unittest.main()
