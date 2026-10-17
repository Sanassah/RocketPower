import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt6.QtWidgets import QApplication, QComboBox
from core.serial_ports import describe_port, PortChoice
from ui.widgets.port_selector import refresh_port_combo


class PortTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_interface_identity_not_com_number(self):
        port = SimpleNamespace(device='COM99', vid=0x16C0, pid=0x048B,
                               hwid='', product=None, description='USB Serial Device')
        self.assertEqual(describe_port(port, {'COM99': 0}).role, 'telemetry')
        self.assertEqual(describe_port(port, {'COM99': 2}).role, 'secondary')
        self.assertEqual(describe_port(port, {}).role, '')
        self.assertIn('unknown', describe_port(port, {}).label)
        port.vid = 0x1234
        self.assertEqual(describe_port(port, {'COM99': 0}).role, '')

    def test_combo_preserves_device_and_prefers_role(self):
        choices = [PortChoice('COM16', 'Debug / HITL (COM16)', 'secondary'),
                   PortChoice('COM17', 'Telemetry (COM17)', 'telemetry')]
        combo = QComboBox()
        with patch('ui.widgets.port_selector.list_port_choices', return_value=choices):
            refresh_port_combo(combo)
            self.assertEqual(combo.currentData(), 'COM17')
            self.assertNotEqual(combo.currentText(), combo.currentData())
            combo.setCurrentIndex(0)
            refresh_port_combo(combo)
            self.assertEqual(combo.currentData(), 'COM16')
            combo.clear()
            refresh_port_combo(combo, 'secondary')
            self.assertEqual(combo.currentData(), 'COM16')
        with patch('ui.widgets.port_selector.list_port_choices', return_value=[]):
            refresh_port_combo(combo)
            self.assertIsNone(combo.currentData())


if __name__ == '__main__':
    unittest.main()
