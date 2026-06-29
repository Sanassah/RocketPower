"""Main GCS window: wires together all panels, manages serial thread lifecycle."""

import os
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QMessageBox, QStatusBar
)
from PyQt6.QtCore import Qt, QThread, pyqtSlot
from PyQt6.QtGui  import QFont

from core.serial_worker   import SerialWorker
from core.packet_decoder  import TelemetryData
from core.packet_encoder  import encode_arm, encode_disarm, encode_fire_pyro, encode_ping
from core.data_logger     import DataLogger

from ui.widgets.connection_panel import ConnectionPanel
from ui.widgets.state_panel      import StatePanel
from ui.widgets.sensor_panel     import SensorPanel
from ui.widgets.altitude_plot    import AltitudePlot
from ui.widgets.accel_plot       import AccelPlot
from ui.widgets.gps_map          import GPSMap
from ui.widgets.pyro_panel       import PyroPanel
from ui.widgets.command_panel    import CommandPanel
from ui.widgets.event_log        import EventLog
from ui.widgets.rocket_visual    import RocketVisual

_LOG_DIR = 'logs'


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Rocket GCS')
        self.resize(1600, 900)
        self.setMinimumSize(1100, 700)

        self._worker: SerialWorker | None = None
        self._thread: QThread | None      = None
        self._logger = DataLogger(log_dir=_LOG_DIR)

        self._build_ui()
        self._connect_signals()

    # ── Layout ──────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 0, 12, 12)
        root.setSpacing(12)

        # ── Top bar ─────────────────────────────────────────────────────
        self._conn_panel = ConnectionPanel()
        root.addWidget(self._conn_panel)

        # ── Main content splitter ────────────────────────────────────────
        h_split = QSplitter(Qt.Orientation.Horizontal)
        h_split.setChildrenCollapsible(False)
        root.addWidget(h_split, stretch=3)

        # Left column
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(12)

        self._state_panel   = StatePanel()
        self._pyro_panel    = PyroPanel()
        self._command_panel = CommandPanel()

        left_layout.addWidget(self._state_panel)
        left_layout.addWidget(self._pyro_panel)
        left_layout.addWidget(self._command_panel)
        left_layout.addStretch()
        left.setMinimumWidth(240)
        left.setMaximumWidth(310)

        # Centre column
        centre = QWidget()
        centre_layout = QVBoxLayout(centre)
        centre_layout.setContentsMargins(0, 0, 0, 0)
        centre_layout.setSpacing(12)

        self._alt_plot    = AltitudePlot()
        self._accel_plot  = AccelPlot()
        self._rocket_vis  = RocketVisual()
        self._rocket_vis.setFixedHeight(160)

        centre_layout.addWidget(self._alt_plot,   stretch=5)
        centre_layout.addWidget(self._accel_plot, stretch=5)
        centre_layout.addWidget(self._rocket_vis, stretch=2)

        # Right column
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(12)

        self._sensor_panel = SensorPanel()
        right_layout.addWidget(self._sensor_panel)
        right.setMinimumWidth(230)
        right.setMaximumWidth(320)

        h_split.addWidget(left)
        h_split.addWidget(centre)
        h_split.addWidget(right)
        h_split.setSizes([270, 1100, 270])

        # ── Bottom splitter ──────────────────────────────────────────────
        b_split = QSplitter(Qt.Orientation.Horizontal)
        b_split.setChildrenCollapsible(False)
        root.addWidget(b_split, stretch=2)

        self._event_log = EventLog()
        self._gps_map   = GPSMap()

        b_split.addWidget(self._event_log)
        b_split.addWidget(self._gps_map)
        b_split.setSizes([500, 900])

        # ── Status bar ───────────────────────────────────────────────────
        self._status_bar = QStatusBar()
        self._status_bar.setFont(QFont('JetBrains Mono', 9))
        self._status_bar.setStyleSheet('color: #8E8E93;')
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage('Ready — connect to a COM port to begin.')

    def _connect_signals(self) -> None:
        # Connection panel
        self._conn_panel.connect_requested.connect(self._start_serial)
        self._conn_panel.disconnect_requested.connect(self._stop_serial)

        # Command panel
        self._command_panel.arm_requested.connect(self._send_arm)
        self._command_panel.disarm_requested.connect(self._send_disarm)
        self._command_panel.fire_pyro_requested.connect(self._send_fire)
        self._command_panel.ping_requested.connect(self._send_ping)

    # ── Serial lifecycle ─────────────────────────────────────────────────

    @pyqtSlot(str, int)
    def _start_serial(self, port: str, baud: int) -> None:
        if self._thread and self._thread.isRunning():
            self._stop_serial()

        self._worker = SerialWorker(port, baud)
        self._thread = QThread(self)

        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)

        self._worker.packet_received.connect(self._on_packet)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._worker.stats_updated.connect(self._conn_panel.update_stats)

        self._thread.start()

    @pyqtSlot()
    def _stop_serial(self) -> None:
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit()
            self._thread.wait(3000)
        self._worker = None
        self._thread = None
        self._logger.close()
        self._conn_panel.set_connected(False, 'Disconnected')

    # ── Packet handling ──────────────────────────────────────────────────

    @pyqtSlot(TelemetryData)
    def _on_packet(self, data: TelemetryData) -> None:
        self._logger.log(data)

        self._state_panel.update_data(data)
        self._sensor_panel.update_data(data)
        self._alt_plot.update_data(data)
        self._accel_plot.update_data(data)
        self._pyro_panel.update_data(data)
        self._command_panel.update_data(data)
        self._rocket_vis.update_data(data)
        self._gps_map.update_data(data)
        self._event_log.update_data(data)

    @pyqtSlot(bool, str)
    def _on_connection_changed(self, connected: bool, message: str) -> None:
        self._conn_panel.set_connected(connected, message)
        if connected:
            log_path = self._logger.open()
            self._status_bar.showMessage(f'Logging → {os.path.abspath(log_path)}')
            self._event_log.log(message, level='ok')
            self._alt_plot.reset()
            self._accel_plot.reset()
            self._gps_map.reset()
        else:
            self._logger.close()
            self._status_bar.showMessage('Disconnected')
            self._event_log.log(message, level='error')

    # ── Command senders ──────────────────────────────────────────────────

    def _send_arm(self) -> None:
        if self._worker:
            self._worker.send_bytes(encode_arm())
            self._event_log.log('↑ ARM command sent', level='command')

    def _send_disarm(self) -> None:
        if self._worker:
            self._worker.send_bytes(encode_disarm())
            self._event_log.log('↑ DISARM command sent', level='command')

    def _send_fire(self, channel: int) -> None:
        if self._worker:
            self._worker.send_bytes(encode_fire_pyro(channel))
            self._event_log.log(f'↑ FIRE PYRO CH{channel} sent', level='warn')
            self._pyro_panel.mark_fired(channel)

    def _send_ping(self) -> None:
        if self._worker:
            self._worker.send_bytes(encode_ping())
            self._event_log.log('↑ PING sent', level='info')

    # ── Window close ─────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        self._stop_serial()
        super().closeEvent(event)
