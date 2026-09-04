"""Main GCS window -- dark dashboard: sidebar nav + two pages."""

import os
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QStatusBar,
    QStackedWidget, QFrame, QSplitter
)
from PyQt6.QtCore import Qt, QThread, pyqtSlot
from PyQt6.QtGui  import QFont

from core.serial_worker   import SerialWorker, LinkStats, list_serial_ports
from core.packet_decoder  import TelemetryData
from core.packet_encoder  import (
    encode_arm, encode_disarm, encode_fire_pyro,
    encode_calibrate,
    encode_servo_test, encode_cam_toggle,
    encode_servo_nudge, encode_servo_save_cal, encode_servo_center_all,
    encode_servo_preflight, encode_sd_start, encode_sd_stop,
    encode_attitude_control_enable, encode_attitude_control_disable,
    encode_attitude_demo_enable, encode_attitude_demo_disable,
    encode_reset,
)
from core.data_logger import DataLogger

from ui.widgets.top_bar          import TopBar
from ui.widgets.gps_map          import GPSMap
from ui.widgets.trajectory_plot  import TrajectoryPlot
from ui.widgets.rocket_visual    import RocketVisual
from ui.widgets.pyro_panel       import PyroPanel
from ui.widgets.continuity_panel import ContinuityPanel
from ui.widgets.arm_panel        import ArmPanel
from ui.widgets.sensor_panel     import SensorPanel
from ui.widgets.command_panel    import CommandPanel
from ui.widgets.event_log        import EventLog
from ui.widgets.log_converter_panel import LogConverterPanel

_LOG_DIR = 'logs'

_BG      = '#0F0F10'
_SIDEBAR = '#0B0B0C'
_CARD    = '#1A1A1B'
_BORDER  = '#2E2E30'
_TEXT    = '#F1F5F9'
_MUTED   = '#94A3B8'
_GREEN   = '#22C55E'
_RED     = '#EF4444'
_BLUE    = '#3B82F6'

_BAUD_RATES   = ['115200', '9600', '57600', '38400', '19200']
_DEFAULT_BAUD = '115200'

# Commands are now ack-confirmed: every send carries a sequence number, the
# flight computer echoes it back in an AckPacket the instant it validates the
# command (before executing anything slow), and it deduplicates by that same
# seq so a resend -- because an earlier ack got lost, not because the command
# itself was lost -- is re-acked but never re-executed. That dedup is what
# makes it safe to retry every command, including FIRE_PYRO: a duplicate
# resend can never fire the pyro channel twice. Retries stop the moment the
# matching ack arrives; if none arrives after _CMD_MAX_RETRIES, we give up
# and tell the user explicitly instead of retrying forever or staying silent.
_CMD_MAX_RETRIES = 5   # retries piggybacked on telemetry receptions before giving up


# ── Nav button ────────────────────────────────────────────────────────────────
class _NavBtn(QPushButton):
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setFixedHeight(46)
        self.setText(label)
        self.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._refresh(False)

    def _refresh(self, active: bool) -> None:
        if active:
            self.setStyleSheet(f"""
                QPushButton {{
                    background-color: {_CARD};
                    color: {_TEXT};
                    border: none;
                    border-left: 2px solid {_TEXT};
                    border-radius: 0;
                    padding: 0 16px;
                    text-align: left;
                    font-size: 10px;
                    font-weight: 700;
                    letter-spacing: 1px;
                }}
            """)
        else:
            self.setStyleSheet(f"""
                QPushButton {{
                    background-color: transparent;
                    color: {_MUTED};
                    border: none;
                    border-left: 2px solid transparent;
                    border-radius: 0;
                    padding: 0 16px;
                    text-align: left;
                    font-size: 10px;
                    font-weight: 700;
                    letter-spacing: 1px;
                }}
                QPushButton:hover {{
                    background-color: {_CARD};
                    color: {_TEXT};
                }}
            """)

    def setChecked(self, checked: bool) -> None:
        super().setChecked(checked)
        self._refresh(checked)


# ── Main window ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('RocketPower GCS')
        self.resize(1600, 960)
        self.setMinimumSize(1200, 720)

        self._worker: SerialWorker | None = None
        self._thread: QThread | None      = None
        self._logger = DataLogger(log_dir=_LOG_DIR)
        self._connected = False

        # Ack-tracked command state -- see _CMD_MAX_RETRIES.
        self._cmd_seq_counter = 0
        self._pending_acks: dict[int, dict] = {}   # seq -> {payload, retries_left, label}

        self._build_ui()
        self._connect_signals()

    # ── Layout ────────────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        central = QWidget()
        central.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        central.setStyleSheet(f'background-color:{_BG};')
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self._top_bar = TopBar()
        root.addWidget(self._top_bar)

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        root.addLayout(body, stretch=1)

        body.addWidget(self._build_sidebar())

        self._pages = QStackedWidget()
        self._pages.setStyleSheet(f'background-color:{_BG};')
        self._pages.addWidget(self._build_page_overview())
        self._pages.addWidget(self._build_page_telemetry())
        self._pages.addWidget(self._build_page_data_tools())
        body.addWidget(self._pages, stretch=1)

        self._status_bar = QStatusBar()
        self._status_bar.setFont(QFont('JetBrains Mono', 11))
        self._status_bar.setStyleSheet(
            f'background-color:{_SIDEBAR};color:{_MUTED};'
            f'border-top:1px solid {_BORDER};'
        )
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage('Ready -- connect to a COM port to begin.')

    # ── Sidebar ───────────────────────────────────────────────────────────────
    def _build_sidebar(self) -> QWidget:
        sb = QWidget()
        sb.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        sb.setStyleSheet(
            f'background-color:{_SIDEBAR};border-right:1px solid {_BORDER};'
        )
        sb.setFixedWidth(132)

        lay = QVBoxLayout(sb)
        lay.setContentsMargins(0, 16, 0, 12)
        lay.setSpacing(0)

        # Wordmark
        wm = QLabel('RP')
        wm.setFont(QFont('Segoe UI', 18, QFont.Weight.ExtraBold))
        wm.setAlignment(Qt.AlignmentFlag.AlignCenter)
        wm.setStyleSheet(
            f'color:{_TEXT};letter-spacing:3px;background:transparent;border:none;padding:0 0 4px 0;'
        )
        lay.addWidget(wm)

        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFixedHeight(1); sep.setStyleSheet(f'background:{_BORDER};margin:8px 0;')
        lay.addWidget(sep)

        # Nav buttons (icon + label)
        self._nav_btns: list[_NavBtn] = []
        for i, label in enumerate(['⊞  OVERVIEW', '↗  TELEMETRY', '⇄  DATA TOOLS']):
            btn = _NavBtn(label)
            btn.clicked.connect(lambda _, idx=i: self._switch_page(idx))
            self._nav_btns.append(btn)
            lay.addWidget(btn)

        self._nav_btns[0].setChecked(True)
        lay.addStretch()

        # System status
        sep2 = QFrame(); sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setFixedHeight(1); sep2.setStyleSheet(f'background:{_BORDER};margin:4px 0;')
        lay.addWidget(sep2)

        status_row = QHBoxLayout()
        status_row.setContentsMargins(10, 4, 10, 4)
        self._sys_dot = QLabel('o')
        self._sys_dot.setFont(QFont('Segoe UI', 12))
        self._sys_dot.setStyleSheet(f'color:{_RED};background:transparent;border:none;')
        self._sys_lbl = QLabel('OFFLINE')
        self._sys_lbl.setStyleSheet(
            f'color:{_MUTED};font-size:12px;font-weight:700;letter-spacing:0.5px;'
            f'background:transparent;border:none;'
        )
        status_row.addWidget(self._sys_dot)
        status_row.addWidget(self._sys_lbl)
        status_row.addStretch()
        lay.addLayout(status_row)

        sep3 = QFrame(); sep3.setFrameShape(QFrame.Shape.HLine)
        sep3.setFixedHeight(1); sep3.setStyleSheet(f'background:{_BORDER};margin:4px 0;')
        lay.addWidget(sep3)

        # Connection controls
        def _small_label(text):
            l = QLabel(text)
            l.setStyleSheet(
                f'color:{_MUTED};font-size:12px;font-weight:700;letter-spacing:0.5px;'
                f'background:transparent;border:none;padding:4px 10px 1px 10px;'
            )
            return l

        _combo_css = f"""
            QComboBox {{
                background-color:#141520;color:{_TEXT};
                border:1px solid {_BORDER};border-radius:4px;
                padding:3px 6px;font-size:12px;margin:0 8px;
            }}
            QComboBox::drop-down {{ border:none; }}
            QComboBox QAbstractItemView {{
                background-color:{_CARD};color:{_TEXT};border:1px solid {_BORDER};
            }}
        """

        lay.addWidget(_small_label('PORT'))
        self._port_combo = QComboBox()
        self._port_combo.setFixedHeight(27)
        self._port_combo.setStyleSheet(_combo_css)
        lay.addWidget(self._port_combo)

        lay.addWidget(_small_label('BAUD'))
        self._baud_combo = QComboBox()
        self._baud_combo.setFixedHeight(27)
        self._baud_combo.addItems(_BAUD_RATES)
        self._baud_combo.setCurrentText(_DEFAULT_BAUD)
        self._baud_combo.setStyleSheet(_combo_css)
        lay.addWidget(self._baud_combo)

        lay.addSpacing(6)
        self._conn_btn = QPushButton('CONNECT')
        self._conn_btn.setFixedHeight(30)
        self._conn_btn.setFont(QFont('Segoe UI', 11, QFont.Weight.Bold))
        self._conn_btn.clicked.connect(self._toggle_connection)
        self._conn_btn.setStyleSheet(self._conn_btn_style(False))
        lay.addWidget(self._conn_btn)

        ref = QPushButton('refresh')
        ref.setFixedHeight(22)
        ref.setFont(QFont('Segoe UI', 10))
        ref.clicked.connect(self._refresh_ports)
        ref.setStyleSheet(f"""
            QPushButton {{
                background:transparent;color:{_MUTED};border:none;
                font-size:10px;margin:0 8px;
            }}
            QPushButton:hover {{ color:{_TEXT}; }}
        """)
        lay.addWidget(ref)

        lay.addSpacing(4)
        ver = QLabel('v1.0.0')
        ver.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ver.setStyleSheet(f'color:{_BORDER};font-size:10px;background:transparent;border:none;')
        lay.addWidget(ver)

        self._refresh_ports()
        return sb

    def _conn_btn_style(self, connected: bool) -> str:
        if connected:
            return f"""
                QPushButton {{
                    background:transparent;color:{_RED};
                    border:1px solid {_RED};border-radius:4px;
                    font-weight:700;letter-spacing:0.5px;margin:0 8px;
                }}
                QPushButton:hover {{ background:#200A0A; }}
            """
        return f"""
            QPushButton {{
                background-color:{_GREEN};color:#0A0F0A;border:none;
                border-radius:4px;font-weight:700;letter-spacing:0.5px;margin:0 8px;
            }}
            QPushButton:hover {{ background-color:#4ADE80; }}
        """

    # ── Page 0: Overview ─────────────────────────────────────────────────────
    def _build_page_overview(self) -> QWidget:
        page = QWidget()
        page.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        page.setStyleSheet(f'background-color:{_BG};')

        root = QVBoxLayout(page)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(18)

        # Top: GPS | Trajectory | 3D
        top = QSplitter(Qt.Orientation.Horizontal)
        top.setChildrenCollapsible(False)
        top.setHandleWidth(18)
        top.setStyleSheet(f'QSplitter::handle{{background:{_BG};}}')

        self._gps_map    = GPSMap()
        self._traj_plot  = TrajectoryPlot()
        self._rocket_vis = RocketVisual()

        top.addWidget(self._wrap(self._gps_map, 'o  GPS MAP'))
        top.addWidget(self._traj_plot)           # already has its own header
        top.addWidget(self._wrap_rocket())
        # Matches the bottom row's sizes exactly (see bot.setSizes below) so
        # all 3 columns line up top-to-bottom: GPS/Pyro, Trajectory/
        # Continuity, 3D/Arm.
        top.setSizes([390, 390, 400])

        # Bottom: Pyro | Continuity | Arm
        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.setChildrenCollapsible(False)
        bot.setHandleWidth(18)
        bot.setStyleSheet(f'QSplitter::handle{{background:{_BG};}}')

        self._pyro_panel = PyroPanel()
        self._cont_panel = ContinuityPanel()
        self._arm_panel  = ArmPanel()

        bot.addWidget(self._pyro_panel)
        bot.addWidget(self._cont_panel)
        bot.addWidget(self._arm_panel)
        bot.setSizes([390, 390, 400])   # top.setSizes above matches this exactly

        vsplit = QSplitter(Qt.Orientation.Vertical)
        vsplit.setChildrenCollapsible(False)
        vsplit.setHandleWidth(18)
        vsplit.setStyleSheet(f'QSplitter::handle{{background:{_BG};}}')
        vsplit.addWidget(top)
        vsplit.addWidget(bot)
        vsplit.setSizes([570, 285])

        root.addWidget(vsplit)
        return page

    def _wrap(self, widget: QWidget, header_text: str) -> QWidget:
        """Wrap a widget in a flat card (no outline -- background-shade
        contrast against the page does the separating) with a header label."""
        card = QWidget()
        card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        card.setStyleSheet(
            f'background-color:{_CARD};border:none;border-radius:10px;'
        )
        lay = QVBoxLayout(card)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        hdr = QWidget()
        hdr.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        hdr.setStyleSheet('background:transparent;')
        hdr.setFixedHeight(38)
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(18, 0, 18, 0)

        lbl = QLabel(header_text)
        lbl.setStyleSheet(
            f'color:{_MUTED};font-size:12px;font-weight:700;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hl.addWidget(lbl)
        hl.addStretch()

        lay.addWidget(hdr)
        lay.addWidget(widget)
        return card

    def _wrap_rocket(self) -> QWidget:
        card = QWidget()
        card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        card.setStyleSheet(
            f'background-color:{_CARD};border:none;border-radius:10px;'
        )
        lay = QVBoxLayout(card)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        hdr = QWidget()
        hdr.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        hdr.setStyleSheet('background:transparent;')
        hdr.setFixedHeight(38)
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(18, 0, 18, 0)

        lbl = QLabel('[]  3D ORIENTATION')
        lbl.setStyleSheet(
            f'color:{_MUTED};font-size:12px;font-weight:700;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hl.addWidget(lbl); hl.addStretch()

        lay.addWidget(hdr)
        lay.addWidget(self._rocket_vis)
        return card

    # ── Page 1: Telemetry / Commands / Events ─────────────────────────────────
    def _build_page_telemetry(self) -> QWidget:
        page = QWidget()
        page.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        page.setStyleSheet(f'background-color:{_BG};')

        root = QVBoxLayout(page)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(18)

        # Top: sensor/telemetry data, full page width (it has its own 2
        # internal columns now -- see SensorPanel -- so it needs the room a
        # single side column never gave it; a single-column layout squeezed
        # against a full-height event log made every row's label and value
        # overlap at the current font size).
        self._sensor_panel = SensorPanel()

        # Bottom: commands/utilities (left) + event log (right, now just a
        # bottom strip instead of full page height).
        self._command_panel = CommandPanel()
        self._event_log     = EventLog()

        self._rec_btn = QPushButton('⏺  REC')
        self._rec_btn.setFixedHeight(27)
        self._rec_btn.setFont(QFont('Segoe UI', 11, QFont.Weight.Bold))
        self._rec_btn.setCheckable(True)
        self._rec_btn.setEnabled(False)
        self._rec_btn.clicked.connect(self._toggle_recording)
        self._rec_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:#222224;color:#64748B;
                border:1px solid #2E2E30;border-radius:5px;
                padding:0 10px;font-weight:700;letter-spacing:0.5px;
            }}
            QPushButton:checked {{
                background-color:#200A0A;color:{_RED};border:1px solid {_RED};
            }}
            QPushButton:enabled:hover {{ background-color:#2A2A2C; }}
            QPushButton:disabled {{
                background-color:#1A1A1B;color:#3A3A3E;border:1px solid #2E2E30;
            }}
        """)
        self._event_log.hdr_layout.addWidget(self._rec_btn)

        bottom = QSplitter(Qt.Orientation.Horizontal)
        bottom.setChildrenCollapsible(False)
        bottom.setHandleWidth(18)
        bottom.setStyleSheet(f'QSplitter::handle{{background:{_BG};}}')
        bottom.addWidget(self._command_panel)
        bottom.addWidget(self._event_log)
        bottom.setSizes([750, 650])

        vsplit = QSplitter(Qt.Orientation.Vertical)
        vsplit.setChildrenCollapsible(False)
        vsplit.setHandleWidth(18)
        vsplit.setStyleSheet(f'QSplitter::handle{{background:{_BG};}}')
        vsplit.addWidget(self._sensor_panel)
        vsplit.addWidget(bottom)
        # Event log confined to the bottom strip's height now instead of the
        # full page -- roughly half of what it had before.
        vsplit.setSizes([480, 440])

        root.addWidget(vsplit)
        return page

    def _toggle_recording(self) -> None:
        if self._rec_btn.isChecked():
            path = self._logger.open()
            self._event_log.log(f'Recording -> {path}', level='ok')
            self._status_bar.showMessage(f'Logging -> {os.path.abspath(path)}')
        else:
            self._logger.close()
            self._event_log.log('Recording stopped.', level='warn')
            self._status_bar.showMessage('Recording stopped.')

    # ── Page 2: Data Tools ───────────────────────────────────────────────────
    def _build_page_data_tools(self) -> QWidget:
        page = QWidget()
        page.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        page.setStyleSheet(f'background-color:{_BG};')

        root = QVBoxLayout(page)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(0)

        self._log_converter = LogConverterPanel()
        root.addWidget(self._log_converter)
        return page

    # ── Signals ───────────────────────────────────────────────────────────────
    def _connect_signals(self) -> None:
        self._arm_panel.arm_requested.connect(self._send_arm)
        self._arm_panel.disarm_requested.connect(self._send_disarm)
        self._arm_panel.attitude_control_enable_requested.connect(self._send_attitude_control_enable)
        self._arm_panel.attitude_control_disable_requested.connect(self._send_attitude_control_disable)
        self._command_panel.fire_pyro_requested.connect(self._send_fire)
        self._command_panel.calibrate_requested.connect(self._send_calibrate)
        self._command_panel.servo_test_requested.connect(self._send_servo_test)
        self._command_panel.cam_toggle_requested.connect(self._send_cam_toggle)
        self._command_panel.servo_nudge_requested.connect(self._send_servo_nudge)
        self._command_panel.servo_save_cal_requested.connect(self._send_servo_save_cal)
        self._command_panel.servo_center_requested.connect(self._send_servo_center)
        self._command_panel.servo_preflight_requested.connect(self._send_servo_preflight)
        self._command_panel.sd_start_requested.connect(self._send_sd_start)
        self._command_panel.sd_stop_requested.connect(self._send_sd_stop)
        self._command_panel.attitude_demo_enable_requested.connect(self._send_attitude_demo_enable)
        self._command_panel.attitude_demo_disable_requested.connect(self._send_attitude_demo_disable)
        self._command_panel.reset_requested.connect(self._send_reset)

    def _switch_page(self, idx: int) -> None:
        self._pages.setCurrentIndex(idx)
        for i, btn in enumerate(self._nav_btns):
            btn.setChecked(i == idx)

    # ── Connection ────────────────────────────────────────────────────────────
    def _toggle_connection(self) -> None:
        if self._connected:
            self._stop_serial()
        else:
            port = self._port_combo.currentText()
            baud = int(self._baud_combo.currentText())
            if port:
                self._start_serial(port, baud)

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentText()
        self._port_combo.clear()
        ports = list_serial_ports()
        self._port_combo.addItems(ports)
        if current in ports:
            self._port_combo.setCurrentText(current)

    def _start_serial(self, port: str, baud: int) -> None:
        if self._thread and self._thread.isRunning():
            self._stop_serial()
        self._worker = SerialWorker(port, baud)
        self._thread = QThread(self)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.packet_received.connect(self._on_packet)
        self._worker.ack_received.connect(self._on_ack)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._worker.stats_updated.connect(self._on_stats)
        self._thread.start()

    def _stop_serial(self) -> None:
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit()
            self._thread.wait(3000)
        self._worker = None
        self._thread = None
        self._logger.close()
        self._on_connection_changed(False, 'Disconnected')

    # ── Packets ───────────────────────────────────────────────────────────────
    @pyqtSlot(TelemetryData)
    def _on_packet(self, data: TelemetryData) -> None:
        self._logger.log(data)
        self._top_bar.update_data(data)
        self._gps_map.update_data(data)
        self._traj_plot.update_data(data)
        self._rocket_vis.update_data(data)
        self._pyro_panel.update_data(data)
        self._cont_panel.update_data(data)
        self._arm_panel.update_data(data)
        self._sensor_panel.update_data(data)
        self._command_panel.update_data(data)
        self._event_log.update_data(data)

        # Retry any still-unacked commands, piggybacked on this packet --
        # receiving telemetry is concrete proof the rocket's radio just
        # finished transmitting and should now be listening, the best
        # moment to retry. Safe to resend blindly: the firmware dedupes by
        # seq, so a resend of an already-executed command is re-acked but
        # never re-run.
        for seq in list(self._pending_acks.keys()):
            pending = self._pending_acks.get(seq)
            if pending is None:
                continue
            if pending['retries_left'] <= 0:
                self._event_log.log(f'✗ {pending["label"]} -- no confirmation, giving up', level='error')
                del self._pending_acks[seq]
                continue
            if self._worker:
                self._worker.send_bytes(pending['payload'])
            pending['retries_left'] -= 1

    @pyqtSlot(int, int)
    def _on_ack(self, cmd_seq: int, cmd_type: int) -> None:
        pending = self._pending_acks.pop(cmd_seq, None)
        if pending:
            self._event_log.log(f'✓ {pending["label"]} confirmed', level='ok')

    @pyqtSlot(object)
    def _on_stats(self, stats: LinkStats) -> None:
        self._top_bar.update_stats(stats)

    @pyqtSlot(bool, str)
    def _on_connection_changed(self, connected: bool, message: str) -> None:
        self._connected = connected
        if connected:
            log_path = self._logger.open()
            self._status_bar.showMessage(f'Logging -> {os.path.abspath(log_path)}')
            self._event_log.log(message, level='ok')
            self._gps_map.reset()
            self._traj_plot.reset()
            self._top_bar.set_connected(True)
            self._sys_dot.setStyleSheet(f'color:{_GREEN};background:transparent;border:none;')
            self._sys_lbl.setText('ONLINE')
            self._conn_btn.setText('DISCONNECT')
            self._conn_btn.setStyleSheet(self._conn_btn_style(True))
            self._port_combo.setEnabled(False)
            self._baud_combo.setEnabled(False)
            self._rec_btn.setEnabled(True)
            self._rec_btn.setChecked(True)
        else:
            self._logger.close()
            self._status_bar.showMessage('Disconnected')
            self._event_log.log(message, level='error')
            self._top_bar.set_connected(False)
            self._pending_acks.clear()
            self._sys_dot.setStyleSheet(f'color:{_RED};background:transparent;border:none;')
            self._sys_lbl.setText('OFFLINE')
            self._conn_btn.setText('CONNECT')
            self._conn_btn.setStyleSheet(self._conn_btn_style(False))
            self._port_combo.setEnabled(True)
            self._baud_combo.setEnabled(True)
            self._rec_btn.setEnabled(False)
            self._rec_btn.setChecked(False)

    # ── Commands ──────────────────────────────────────────────────────────────
    def _next_seq(self) -> int:
        self._cmd_seq_counter = (self._cmd_seq_counter + 1) % 256
        return self._cmd_seq_counter

    def _send_tracked(self, encode_fn, *args, label: str, level: str = 'command') -> None:
        """Send a command tagged with a fresh sequence number, log it, and
        track it for ack-confirmed retry (see _CMD_MAX_RETRIES and _on_ack).
        Safe for every command type, including FIRE_PYRO: the firmware
        dedupes by seq, so a resend can only ever re-confirm, never re-run."""
        if not self._worker:
            return
        seq = self._next_seq()
        payload = encode_fn(*args, seq=seq)
        self._worker.send_bytes(payload)
        self._pending_acks[seq] = {
            'payload': payload,
            'retries_left': _CMD_MAX_RETRIES,
            'label': label,
        }
        self._event_log.log(f'^ {label} sent', level=level)

    def _send_arm(self) -> None:
        self._send_tracked(encode_arm, label='ARM')

    def _send_disarm(self) -> None:
        self._send_tracked(encode_disarm, label='DISARM')

    def _send_fire(self, channel: int) -> None:
        names = {1: 'Ignition', 2: 'Parachute', 3: 'Backup'}
        self._send_tracked(
            encode_fire_pyro, channel,
            label=f'FIRE CH{channel} ({names.get(channel, "")})', level='warn',
        )
        self._pyro_panel.mark_fired(channel)

    def _send_calibrate(self) -> None:
        self._send_tracked(encode_calibrate, label='CALIBRATE')
        # Bench-test convenience, ground-station-only: also re-zero the YAW
        # readout to whatever heading it's currently facing. Deliberately
        # yaw-only, not roll/pitch -- those have a real physical zero from
        # gravity and must keep showing true tilt, not whatever pose the
        # rocket happened to be in when this was pressed. Doesn't touch the
        # firmware, telemetry, or AttitudeController's own (separately
        # latched) reference -- see RocketVisual.zero_yaw().
        self._rocket_vis.zero_yaw()

    def _send_servo_test(self, channel: int) -> None:
        self._send_tracked(encode_servo_test, channel, label=f'SERVO TEST CH{channel}')

    def _send_cam_toggle(self) -> None:
        self._send_tracked(encode_cam_toggle, label='CAM TOGGLE')

    def _send_servo_nudge(self, channel: int, positive: bool) -> None:
        sign = '+' if positive else '-'
        self._send_tracked(
            encode_servo_nudge, channel, positive,
            label=f'SERVO CH{channel} nudge {sign}', level='info',
        )

    def _send_servo_save_cal(self) -> None:
        self._send_tracked(encode_servo_save_cal, label='SERVO CALIBRATION', level='ok')

    def _send_servo_center(self) -> None:
        self._send_tracked(encode_servo_center_all, label='SERVO CENTER ALL')

    def _send_servo_preflight(self) -> None:
        self._send_tracked(encode_servo_preflight, label='SERVO PREFLIGHT')

    def _send_sd_start(self) -> None:
        self._send_tracked(encode_sd_start, label='SD START RECORDING', level='ok')

    def _send_sd_stop(self) -> None:
        self._send_tracked(encode_sd_stop, label='SD STOP RECORDING', level='warn')

    def _send_attitude_control_enable(self) -> None:
        self._send_tracked(encode_attitude_control_enable, label='ATTITUDE CONTROL ENABLE (REAL)', level='warn')

    def _send_attitude_control_disable(self) -> None:
        self._send_tracked(encode_attitude_control_disable, label='ATTITUDE CONTROL DISABLE (REAL)', level='ok')

    def _send_attitude_demo_enable(self) -> None:
        self._send_tracked(encode_attitude_demo_enable, label='ATTITUDE DEMO ENABLE', level='ok')

    def _send_attitude_demo_disable(self) -> None:
        self._send_tracked(encode_attitude_demo_disable, label='ATTITUDE DEMO DISABLE')

    def _send_reset(self) -> None:
        # Clear the log FIRST -- _send_tracked's own "^ RESET sent" line
        # (and the ack confirmation right after) become the first fresh
        # entries, giving the next test flight a clean slate instead of
        # scrolling past the previous run's history. Doesn't touch the
        # flight computer's own SD log -- that's closed/reopened firmware-
        # side (see main.cpp's RESET handling).
        self._event_log.clear()
        self._send_tracked(encode_reset, label='RESET', level='warn')

    def closeEvent(self, event) -> None:
        self._stop_serial()
        super().closeEvent(event)
