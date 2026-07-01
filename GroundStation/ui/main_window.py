"""Main GCS window -- dark dashboard: sidebar nav + two pages."""

import os
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QStatusBar,
    QStackedWidget, QFrame, QSplitter
)
from PyQt6.QtCore import Qt, QThread, pyqtSlot
from PyQt6.QtGui  import QFont

from core.serial_worker   import SerialWorker, list_serial_ports
from core.packet_decoder  import TelemetryData
from core.packet_encoder  import (
    encode_arm, encode_disarm, encode_fire_pyro,
    encode_ping, encode_calibrate,
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


# ── Nav button ────────────────────────────────────────────────────────────────
class _NavBtn(QPushButton):
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setFixedHeight(44)
        self.setText(label)
        self.setFont(QFont('Segoe UI', 9, QFont.Weight.Bold))
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
                    font-size: 9px;
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
                    font-size: 9px;
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
        body.addWidget(self._pages, stretch=1)

        self._status_bar = QStatusBar()
        self._status_bar.setFont(QFont('JetBrains Mono', 9))
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
        sb.setFixedWidth(110)

        lay = QVBoxLayout(sb)
        lay.setContentsMargins(0, 16, 0, 12)
        lay.setSpacing(0)

        # Wordmark
        wm = QLabel('RP')
        wm.setFont(QFont('Segoe UI', 16, QFont.Weight.ExtraBold))
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
        for i, label in enumerate(['⊞  OVERVIEW', '↗  TELEMETRY']):
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
        self._sys_dot.setFont(QFont('Segoe UI', 10))
        self._sys_dot.setStyleSheet(f'color:{_RED};background:transparent;border:none;')
        self._sys_lbl = QLabel('OFFLINE')
        self._sys_lbl.setStyleSheet(
            f'color:{_MUTED};font-size:8px;font-weight:700;letter-spacing:0.5px;'
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
                f'color:{_MUTED};font-size:8px;font-weight:700;letter-spacing:0.5px;'
                f'background:transparent;border:none;padding:4px 10px 1px 10px;'
            )
            return l

        _combo_css = f"""
            QComboBox {{
                background-color:#141520;color:{_TEXT};
                border:1px solid {_BORDER};border-radius:4px;
                padding:2px 6px;font-size:10px;margin:0 8px;
            }}
            QComboBox::drop-down {{ border:none; }}
            QComboBox QAbstractItemView {{
                background-color:{_CARD};color:{_TEXT};border:1px solid {_BORDER};
            }}
        """

        lay.addWidget(_small_label('PORT'))
        self._port_combo = QComboBox()
        self._port_combo.setFixedHeight(24)
        self._port_combo.setStyleSheet(_combo_css)
        lay.addWidget(self._port_combo)

        lay.addWidget(_small_label('BAUD'))
        self._baud_combo = QComboBox()
        self._baud_combo.setFixedHeight(24)
        self._baud_combo.addItems(_BAUD_RATES)
        self._baud_combo.setCurrentText(_DEFAULT_BAUD)
        self._baud_combo.setStyleSheet(_combo_css)
        lay.addWidget(self._baud_combo)

        lay.addSpacing(6)
        self._conn_btn = QPushButton('CONNECT')
        self._conn_btn.setFixedHeight(28)
        self._conn_btn.setFont(QFont('Segoe UI', 9, QFont.Weight.Bold))
        self._conn_btn.clicked.connect(self._toggle_connection)
        self._conn_btn.setStyleSheet(self._conn_btn_style(False))
        lay.addWidget(self._conn_btn)

        ref = QPushButton('refresh')
        ref.setFixedHeight(20)
        ref.setFont(QFont('Segoe UI', 8))
        ref.clicked.connect(self._refresh_ports)
        ref.setStyleSheet(f"""
            QPushButton {{
                background:transparent;color:{_MUTED};border:none;
                font-size:8px;margin:0 8px;
            }}
            QPushButton:hover {{ color:{_TEXT}; }}
        """)
        lay.addWidget(ref)

        lay.addSpacing(4)
        ver = QLabel('v1.0.0')
        ver.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ver.setStyleSheet(f'color:{_BORDER};font-size:8px;background:transparent;border:none;')
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
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # Top: GPS | Trajectory | 3D
        top = QSplitter(Qt.Orientation.Horizontal)
        top.setChildrenCollapsible(False)
        top.setStyleSheet(f'QSplitter::handle{{background:{_BORDER};width:1px;}}')

        self._gps_map    = GPSMap()
        self._traj_plot  = TrajectoryPlot()
        self._rocket_vis = RocketVisual()

        top.addWidget(self._wrap(self._gps_map, 'o  GPS MAP'))
        top.addWidget(self._traj_plot)           # already has its own header
        top.addWidget(self._wrap_rocket())
        top.setSizes([400, 440, 360])

        # Bottom: Pyro | Continuity | Arm
        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.setChildrenCollapsible(False)
        bot.setStyleSheet(f'QSplitter::handle{{background:{_BORDER};width:1px;}}')

        self._pyro_panel = PyroPanel()
        self._cont_panel = ContinuityPanel()
        self._arm_panel  = ArmPanel()

        bot.addWidget(self._pyro_panel)
        bot.addWidget(self._cont_panel)
        bot.addWidget(self._arm_panel)
        bot.setSizes([390, 390, 400])

        vsplit = QSplitter(Qt.Orientation.Vertical)
        vsplit.setChildrenCollapsible(False)
        vsplit.setStyleSheet(f'QSplitter::handle{{background:{_BORDER};height:1px;}}')
        vsplit.addWidget(top)
        vsplit.addWidget(bot)
        vsplit.setSizes([570, 285])

        root.addWidget(vsplit)
        return page

    def _wrap(self, widget: QWidget, header_text: str) -> QWidget:
        """Wrap a widget in a dark card with a subtle header label."""
        card = QWidget()
        card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        card.setStyleSheet(
            f'background-color:{_CARD};border:1px solid {_BORDER};border-radius:8px;'
        )
        lay = QVBoxLayout(card)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        hdr = QWidget()
        hdr.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        hdr.setStyleSheet('background:transparent;')
        hdr.setFixedHeight(32)
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(14, 0, 14, 0)

        lbl = QLabel(header_text)
        lbl.setStyleSheet(
            f'color:{_MUTED};font-size:10px;font-weight:700;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hl.addWidget(lbl)
        hl.addStretch()

        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFixedHeight(1); sep.setStyleSheet(f'background:{_BORDER};')

        lay.addWidget(hdr)
        lay.addWidget(sep)
        lay.addWidget(widget)
        return card

    def _wrap_rocket(self) -> QWidget:
        card = QWidget()
        card.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        card.setStyleSheet(
            f'background-color:{_CARD};border:1px solid {_BORDER};border-radius:8px;'
        )
        lay = QVBoxLayout(card)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        hdr = QWidget()
        hdr.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        hdr.setStyleSheet('background:transparent;')
        hdr.setFixedHeight(32)
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(14, 0, 14, 0)

        lbl = QLabel('[]  3D ORIENTATION  (MATLAB STYLE)')
        lbl.setStyleSheet(
            f'color:{_MUTED};font-size:10px;font-weight:700;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hl.addWidget(lbl); hl.addStretch()

        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFixedHeight(1); sep.setStyleSheet(f'background:{_BORDER};')

        lay.addWidget(hdr)
        lay.addWidget(sep)
        lay.addWidget(self._rocket_vis)
        return card

    # ── Page 1: Telemetry / Commands / Events ─────────────────────────────────
    def _build_page_telemetry(self) -> QWidget:
        page = QWidget()
        page.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        page.setStyleSheet(f'background-color:{_BG};')

        root = QHBoxLayout(page)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        self._sensor_panel  = SensorPanel()
        self._command_panel = CommandPanel()
        self._event_log     = EventLog()

        self._sensor_panel.setMinimumWidth(240)
        self._sensor_panel.setMaximumWidth(320)
        self._command_panel.setMinimumWidth(260)
        self._command_panel.setMaximumWidth(360)

        root.addWidget(self._sensor_panel)
        root.addWidget(self._command_panel)
        root.addWidget(self._event_log, stretch=1)
        return page

    # ── Signals ───────────────────────────────────────────────────────────────
    def _connect_signals(self) -> None:
        self._arm_panel.arm_requested.connect(self._send_arm)
        self._arm_panel.disarm_requested.connect(self._send_disarm)
        self._command_panel.arm_requested.connect(self._send_arm)
        self._command_panel.disarm_requested.connect(self._send_disarm)
        self._command_panel.fire_pyro_requested.connect(self._send_fire)
        self._command_panel.ping_requested.connect(self._send_ping)
        self._command_panel.calibrate_requested.connect(self._send_calibrate)

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
        self._worker.connection_changed.connect(self._on_connection_changed)
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
        else:
            self._logger.close()
            self._status_bar.showMessage('Disconnected')
            self._event_log.log(message, level='error')
            self._top_bar.set_connected(False)
            self._sys_dot.setStyleSheet(f'color:{_RED};background:transparent;border:none;')
            self._sys_lbl.setText('OFFLINE')
            self._conn_btn.setText('CONNECT')
            self._conn_btn.setStyleSheet(self._conn_btn_style(False))
            self._port_combo.setEnabled(True)
            self._baud_combo.setEnabled(True)

    # ── Commands ──────────────────────────────────────────────────────────────
    def _send_arm(self) -> None:
        if self._worker:
            self._worker.send_bytes(encode_arm())
            self._event_log.log('^ ARM sent', level='command')

    def _send_disarm(self) -> None:
        if self._worker:
            self._worker.send_bytes(encode_disarm())
            self._event_log.log('^ DISARM sent', level='command')

    def _send_fire(self, channel: int) -> None:
        if self._worker:
            names = {1: 'Ignition', 2: 'Parachute', 3: 'Backup'}
            self._worker.send_bytes(encode_fire_pyro(channel))
            self._event_log.log(
                f'^ FIRE CH{channel} ({names.get(channel,"")}) sent', level='warn'
            )
            self._pyro_panel.mark_fired(channel)

    def _send_ping(self) -> None:
        if self._worker:
            self._worker.send_bytes(encode_ping())
            self._event_log.log('^ PING sent', level='info')

    def _send_calibrate(self) -> None:
        if self._worker:
            self._worker.send_bytes(encode_calibrate())
            self._event_log.log('^ CALIBRATE BARO sent', level='command')

    def closeEvent(self, event) -> None:
        self._stop_serial()
        super().closeEvent(event)
