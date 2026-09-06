"""Live view of the flight computer's DEBUG_SERIAL port (Teensy's second USB
serial port, SerialUSB1) -- plain-text bench diagnostics (loop/sensor/GPS/SD
timing breakdowns, init messages) that firmware keeps off the main telemetry
port entirely, specifically so it stops interleaving with the binary packet
stream there (see FlightComputer's config.h DEBUG_SERIAL macro). Exists so a
bench session doesn't need a separate serial-monitor window fighting with
GroundStation for the same COM port.

Independent of the main telemetry connection on purpose -- this is a second
physical COM port (a separate USB CDC interface on the same Teensy), so it
gets its own port/connect controls rather than piggybacking on the telemetry
Connect button.

Two view modes, toggled by the LOG/LIVE buttons, both fed by the same
underlying stream:
  - LOG: the raw scrolling text (uncapped), mirrored to a timestamped file
    (see _open_file) so nothing is lost even across a GroundStation restart.
    The SHOW list is a live tag filter -- one checkbox per distinct "[TAG]"
    prefix seen so far (auto-discovered, not hardcoded), each checked by
    default. Unchecking one only hides it from the view -- the file and the
    in-memory history both still keep every line regardless.
  - LIVE: every "key=value" pair firmware has ever printed, one static row
    per key (namespaced by its [TAG] so e.g. two different prints both using
    "v=" don't collide), where only the value updates in place -- nothing
    scrolls. Rows appear on their own the first time a key is seen.

REC starts/stops writing the current LIVE values to a CSV file (one row per
tick, fixed columns snapshotted from whatever keys are known when REC is
pressed -- a key that first appears *after* that point isn't retroactively
added to that file; let the stream run a bit first, or press REC again to
start a fresh file with an updated column set) -- built for pulling straight
into a spreadsheet/plotting tool afterward.
"""

import csv
import os
import re
import time
from datetime import datetime
from typing import Optional

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPlainTextEdit,
    QPushButton, QComboBox, QListWidget, QListWidgetItem, QScrollArea,
    QStackedWidget, QFrame
)
from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSlot
from PyQt6.QtGui import QFont

from core.raw_serial_worker import RawSerialWorker
from core.serial_worker import list_serial_ports

# Matches a leading "[SOME TAG]" at the start of a line, e.g. "[BARO RAW]",
# "[SENSOR BREAKDOWN]", "[GPS BREAKDOWN]" -- everything firmware prints on
# DEBUG_SERIAL follows this convention. A line that doesn't match this (rare)
# falls into the _OTHER_TAG bucket below rather than being dropped.
_TAG_RE = re.compile(r'^(\[[^\]]+\])')
_OTHER_TAG = '(other)'

# Generic "key=value" extractor, run against whatever's left of a line after
# its tag. Value is either a parenthetical group -- "q=(0.1,0.2,0.3,0.4)"
# (IMU RAW's quaternion) comes back as one opaque string, not exploded into
# 4 fields -- or a plain token up to the next whitespace/closing-paren (so
# "(singleCallMax=7.679ms calls=5)" still yields two clean separate pairs,
# the surrounding parens are just noise; and "calls=5)" doesn't drag that
# trailing ")" into the value).
_KV_RE = re.compile(r'([A-Za-z_][A-Za-z0-9_.]*)\s*=\s*(\([^)]*\)|[^\s)]+)')
# [LOOP]'s own rate is printed as a bare "68.6 Hz" with no "key=", not caught
# by the generic pattern above -- special-cased since it's too useful a
# number to leave out of the LIVE view/CSV.
_HZ_RE = re.compile(r'^\s*([\d.]+)\s*Hz')
# Strips a trailing unit suffix ("ms", "V", "Hz", "g", ...) so e.g. "46.752ms"
# still lands in the CSV as a plottable number, 46.752 -- leaves the raw
# string (with the unit) alone for on-screen display either way.
_LEADING_NUM_RE = re.compile(r'^[-+]?[0-9]*\.?[0-9]+')

_BG      = '#1A1A1B'
_BORDER  = '#2E2E30'
_TEXT    = '#F1F5F9'
_MUTED   = '#94A3B8'
_GREEN   = '#22C55E'
_RED     = '#EF4444'
_BLUE    = '#3B82F6'

_LOG_DIR = 'logs'
_RECORD_INTERVAL_MS = 200   # CSV row cadence -- 5 rows/s, plenty for a bench plot


def _to_float(raw: str) -> Optional[float]:
    m = _LEADING_NUM_RE.match(raw)
    if not m:
        return None
    try:
        return float(m.group(0))
    except ValueError:
        return None


class _LiveRow:
    """One static label + a value that updates in place -- same visual
    pattern as SensorPanel/TestPanel's row helpers elsewhere in this app."""
    def __init__(self, layout: QGridLayout, row: int, key: str):
        lbl = QLabel(key)
        lbl.setStyleSheet(f'color:{_MUTED};font-size:12px;border:none;background:transparent;')
        self._val = QLabel('--')
        self._val.setStyleSheet(
            f'color:{_TEXT};font-family:"JetBrains Mono",monospace;'
            f'font-size:13px;font-weight:600;border:none;background:transparent;'
        )
        self._val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(lbl,       row, 0)
        layout.addWidget(self._val, row, 1)

    def set(self, text: str) -> None:
        self._val.setText(text)


class DebugLogPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(f'background-color:{_BG};border:none;border-radius:10px;')

        self._worker: Optional[RawSerialWorker] = None
        self._thread: Optional[QThread] = None
        self._connected = False
        self._file = None
        self._file_path: Optional[str] = None

        # Every line ever received this session (unfiltered) -- the file on
        # disk mirrors this same complete set regardless of the tag filter
        # below, which only controls what's actually shown in self._text.
        # Kept in memory too (not just the file) so toggling a checkbox can
        # instantly re-render the filtered view from history.
        self._all_lines: list[str] = []
        self._tag_items: dict[str, QListWidgetItem] = {}

        # LIVE view state: namespaced "[TAG] key" -> latest raw string value,
        # and the row widget showing it. live_sections holds each tag's grid
        # layout so a newly-seen key gets appended under the right header
        # instead of creating a duplicate section.
        self._live_values: dict[str, str] = {}
        self._live_rows: dict[str, _LiveRow] = {}
        self._live_sections: dict[str, QGridLayout] = {}
        self._live_container: Optional[QVBoxLayout] = None

        # CSV recording
        self._record_file = None
        self._record_writer = None
        self._record_columns: list[str] = []
        self._record_timer: Optional[QTimer] = None
        self._record_start_time = 0.0
        self._recording = False

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 16, 20, 16)
        root.setSpacing(8)

        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('#')
        sym.setStyleSheet(f'color:{_TEXT};font-size:14px;background:transparent;border:none;')
        ttl = QLabel('DEBUG SERIAL  (bench diagnostics)')
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:13px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()

        combo_css = f"""
            QComboBox {{
                background-color:#141520;color:{_TEXT};
                border:1px solid {_BORDER};border-radius:4px;
                padding:2px 6px;font-size:12px;
            }}
            QComboBox::drop-down {{ border:none; }}
            QComboBox QAbstractItemView {{
                background-color:{_BG};color:{_TEXT};border:1px solid {_BORDER};
            }}
        """
        self._port_combo = QComboBox()
        self._port_combo.setFixedHeight(24)
        self._port_combo.setFixedWidth(110)
        self._port_combo.setStyleSheet(combo_css)
        hdr.addWidget(self._port_combo)

        refresh_btn = QPushButton('refresh')
        refresh_btn.setFixedHeight(24)
        refresh_btn.setStyleSheet(
            f'QPushButton{{background:transparent;color:{_MUTED};border:none;font-size:11px;}}'
            f'QPushButton:hover{{color:{_TEXT};}}'
        )
        refresh_btn.clicked.connect(self._refresh_ports)
        hdr.addWidget(refresh_btn)

        self._conn_btn = QPushButton('CONNECT')
        self._conn_btn.setFixedHeight(24)
        self._conn_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._conn_btn.clicked.connect(self._toggle_connection)
        self._set_conn_style(False)
        hdr.addWidget(self._conn_btn)

        clear_btn = QPushButton('clear')
        clear_btn.setFixedHeight(24)
        clear_btn.setStyleSheet(
            f'QPushButton{{background:transparent;color:{_MUTED};border:none;font-size:11px;}}'
            f'QPushButton:hover{{color:{_TEXT};}}'
        )
        clear_btn.clicked.connect(self._clear)
        hdr.addWidget(clear_btn)

        root.addLayout(hdr)

        # ---- View toggle + record -----------------------------------
        mode_row = QHBoxLayout(); mode_row.setSpacing(6)
        self._log_view_btn  = QPushButton('LOG')
        self._live_view_btn = QPushButton('LIVE')
        for b in (self._log_view_btn, self._live_view_btn):
            b.setCheckable(True)
            b.setFixedHeight(24)
            b.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._log_view_btn.setChecked(True)
        self._log_view_btn.clicked.connect(lambda: self._set_view(0))
        self._live_view_btn.clicked.connect(lambda: self._set_view(1))
        mode_row.addWidget(self._log_view_btn)
        mode_row.addWidget(self._live_view_btn)
        mode_row.addStretch()

        self._record_btn = QPushButton('● REC')
        self._record_btn.setFixedHeight(24)
        self._record_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._record_btn.clicked.connect(self._toggle_recording)
        self._set_record_style(False)
        mode_row.addWidget(self._record_btn)
        root.addLayout(mode_row)
        self._update_view_btn_styles()

        self._status = QLabel('Not connected -- pick the Teensy\'s SECOND COM port (not the telemetry one) and hit Connect.')
        self._status.setWordWrap(True)
        self._status.setStyleSheet(f'color:{_MUTED};font-size:11px;background:transparent;border:none;')
        root.addWidget(self._status)

        body = QHBoxLayout()
        body.setSpacing(10)

        # ---- Tag filter -- which [BRACKETED] categories actually show up
        # in self._text below. Populated dynamically as new tags are seen
        # (nothing hardcoded -- if firmware adds a new [WHATEVER] print
        # tomorrow, it just shows up here on its own next reflash), each
        # defaulting to checked so nothing is hidden by surprise. Unchecking
        # one only affects the on-screen view -- self._file and
        # self._all_lines both still get every line regardless.
        filter_col = QVBoxLayout()
        filter_col.setSpacing(4)
        filter_hdr = QLabel('SHOW')
        filter_hdr.setStyleSheet(
            f'color:{_MUTED};font-size:10px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        filter_col.addWidget(filter_hdr)

        filter_btns = QHBoxLayout(); filter_btns.setSpacing(6)
        all_btn = QPushButton('all')
        none_btn = QPushButton('none')
        for b in (all_btn, none_btn):
            b.setFixedHeight(18)
            b.setStyleSheet(
                f'QPushButton{{background:transparent;color:{_MUTED};border:none;font-size:10px;}}'
                f'QPushButton:hover{{color:{_TEXT};}}'
            )
        all_btn.clicked.connect(lambda: self._set_all_tags(True))
        none_btn.clicked.connect(lambda: self._set_all_tags(False))
        filter_btns.addWidget(all_btn); filter_btns.addWidget(none_btn); filter_btns.addStretch()
        filter_col.addLayout(filter_btns)

        self._tag_list = QListWidget()
        self._tag_list.setFixedWidth(150)
        self._tag_list.setStyleSheet(f"""
            QListWidget {{
                background-color: #0F0F10;
                color: {_TEXT};
                border: 1px solid {_BORDER};
                border-radius: 6px;
                font-size: 11px;
            }}
            QListWidget::item {{ padding: 2px 4px; }}
        """)
        self._tag_list.itemChanged.connect(self._on_tag_item_changed)
        filter_col.addWidget(self._tag_list, stretch=1)
        body.addLayout(filter_col)

        self._stack = QStackedWidget()

        self._text = QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QFont('JetBrains Mono', 12))
        # Uncapped on purpose -- this exists specifically so bench data isn't
        # lost; the mirrored file (see _open_file below) is the real safety
        # net if a session runs long enough for this to matter memory-wise.
        self._text.setMaximumBlockCount(0)
        self._text.setStyleSheet(f"""
            QPlainTextEdit {{
                background-color: #0F0F10;
                color: #F1F5F9;
                border: 1px solid {_BORDER};
                border-radius: 6px;
                padding: 6px 10px;
                font-size: 12px;
            }}
        """)
        self._stack.addWidget(self._text)                 # index 0: LOG
        self._stack.addWidget(self._build_live_view())     # index 1: LIVE

        body.addWidget(self._stack, stretch=1)
        root.addLayout(body, stretch=1)

        self._refresh_ports()

    # ------------------------------------------------------------------
    # LIVE view

    def _build_live_view(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(f"""
            QScrollArea {{ background-color:#0F0F10; border:1px solid {_BORDER}; border-radius:6px; }}
        """)
        inner = QWidget()
        inner.setStyleSheet('background-color:#0F0F10;')
        layout = QVBoxLayout(inner)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(10)
        layout.addStretch()   # keeps sections top-aligned as they're added
        self._live_container = layout
        scroll.setWidget(inner)
        return scroll

    def _update_live_value(self, namespaced_key: str, tag: str, short_key: str, raw_value: str) -> None:
        self._live_values[namespaced_key] = raw_value
        row = self._live_rows.get(namespaced_key)
        if row is None:
            row = self._add_live_row(tag, short_key, namespaced_key)
        row.set(raw_value)

    def _add_live_row(self, tag: str, short_key: str, namespaced_key: str) -> _LiveRow:
        grid = self._live_sections.get(tag)
        if grid is None:
            section = QFrame()
            section.setStyleSheet('background:transparent;border:none;')
            section_layout = QVBoxLayout(section)
            section_layout.setContentsMargins(0, 0, 0, 0)
            section_layout.setSpacing(4)
            hdr = QLabel(tag)
            hdr.setStyleSheet(
                f'color:{_BLUE};font-size:11px;font-weight:800;letter-spacing:0.5px;'
                f'background:transparent;border:none;'
            )
            section_layout.addWidget(hdr)
            grid = QGridLayout()
            grid.setColumnStretch(1, 1)
            grid.setVerticalSpacing(2)
            section_layout.addLayout(grid)
            self._live_sections[tag] = grid
            # Insert before the trailing stretch (always the last item).
            self._live_container.insertWidget(self._live_container.count() - 1, section)
        row_idx = grid.rowCount()
        row = _LiveRow(grid, row_idx, short_key)
        self._live_rows[namespaced_key] = row
        return row

    def _set_view(self, index: int) -> None:
        self._stack.setCurrentIndex(index)
        self._log_view_btn.setChecked(index == 0)
        self._live_view_btn.setChecked(index == 1)
        self._update_view_btn_styles()

    def _update_view_btn_styles(self) -> None:
        for btn in (self._log_view_btn, self._live_view_btn):
            if btn.isChecked():
                btn.setStyleSheet(f"""
                    QPushButton {{ background-color:{_BLUE};color:#fff;border:none;border-radius:4px;font-weight:700; }}
                """)
            else:
                btn.setStyleSheet(f"""
                    QPushButton {{ background:transparent;color:{_MUTED};border:1px solid {_BORDER};border-radius:4px;font-weight:700; }}
                    QPushButton:hover {{ color:{_TEXT}; }}
                """)

    # ------------------------------------------------------------------
    # CSV recording

    def _set_record_style(self, recording: bool) -> None:
        if recording:
            self._record_btn.setStyleSheet(f"""
                QPushButton {{ background-color:{_RED};color:#fff;border:none;border-radius:4px;font-weight:700; }}
            """)
        else:
            self._record_btn.setStyleSheet(f"""
                QPushButton {{ background:transparent;color:{_MUTED};border:1px solid {_BORDER};border-radius:4px;font-weight:700; }}
                QPushButton:hover {{ color:{_TEXT}; }}
            """)

    def _toggle_recording(self) -> None:
        if self._recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self) -> None:
        # Fixed column set, snapshotted now -- a key first seen after this
        # point isn't retroactively added (CSV headers can't grow mid-file).
        # Let the stream run a bit first, or press REC again for a fresh
        # file with an updated column set, if something's missing.
        self._record_columns = sorted(self._live_values.keys())
        if not self._record_columns:
            self._status.setText('Nothing to record yet -- no values seen. Connect and wait for some data first.')
            return
        try:
            os.makedirs(_LOG_DIR, exist_ok=True)
            ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            path = os.path.join(_LOG_DIR, f'values_{ts}.csv')
            self._record_file = open(path, 'w', newline='', encoding='utf-8')
            self._record_writer = csv.writer(self._record_file)
            self._record_writer.writerow(['time_s'] + self._record_columns)
        except Exception:
            self._record_file = None
            self._record_writer = None
            return

        self._record_start_time = time.time()
        self._record_timer = QTimer(self)
        self._record_timer.timeout.connect(self._record_tick)
        self._record_timer.start(_RECORD_INTERVAL_MS)
        self._recording = True
        self._set_record_style(True)
        self._status.setText(f'Recording to {path}')

    def _record_tick(self) -> None:
        if not self._record_writer:
            return
        t = time.time() - self._record_start_time
        row = [f'{t:.3f}']
        for col in self._record_columns:
            raw = self._live_values.get(col, '')
            num = _to_float(raw)
            row.append(num if num is not None else raw)
        try:
            self._record_writer.writerow(row)
            self._record_file.flush()
        except Exception:
            pass

    def _stop_recording(self) -> None:
        if self._record_timer:
            self._record_timer.stop()
            self._record_timer = None
        if self._record_file:
            try:
                self._record_file.flush()
                self._record_file.close()
            except Exception:
                pass
        self._record_file = None
        self._record_writer = None
        self._recording = False
        self._set_record_style(False)

    # ------------------------------------------------------------------
    def _set_conn_style(self, connected: bool) -> None:
        if connected:
            self._conn_btn.setText('DISCONNECT')
            self._conn_btn.setStyleSheet(f"""
                QPushButton {{
                    background:transparent;color:{_RED};
                    border:1px solid {_RED};border-radius:4px;font-weight:700;
                }}
                QPushButton:hover {{ background:#200A0A; }}
            """)
        else:
            self._conn_btn.setText('CONNECT')
            self._conn_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color:{_GREEN};color:#0A0F0A;border:none;
                    border-radius:4px;font-weight:700;
                }}
                QPushButton:hover {{ background-color:#4ADE80; }}
            """)

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentText()
        self._port_combo.clear()
        ports = list_serial_ports()
        self._port_combo.addItems(ports)
        if current in ports:
            self._port_combo.setCurrentText(current)

    def _clear(self) -> None:
        self._text.clear()
        self._all_lines.clear()

    def shutdown(self) -> None:
        """Call from the main window's closeEvent so this panel's thread
        doesn't outlive the app."""
        if self._recording:
            self._stop_recording()
        if self._connected:
            self._stop()

    def _toggle_connection(self) -> None:
        if self._connected:
            self._stop()
        else:
            port = self._port_combo.currentText()
            if port:
                self._start(port)

    def _start(self, port: str) -> None:
        if self._thread and self._thread.isRunning():
            self._stop()

        self._open_file()

        self._worker = RawSerialWorker(port, 115200)
        self._thread = QThread(self)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.line_received.connect(self._on_line)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._thread.start()

    def _stop(self) -> None:
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit()
            self._thread.wait(3000)
        self._worker = None
        self._thread = None
        self._close_file()
        self._on_connection_changed(False, 'Disconnected')

    @pyqtSlot(bool, str)
    def _on_connection_changed(self, connected: bool, message: str) -> None:
        self._connected = connected
        self._set_conn_style(connected)
        suffix = f' -- log saved to {self._file_path}' if connected and self._file_path else ''
        self._status.setText(message + suffix)

    @pyqtSlot(str)
    def _on_line(self, line: str) -> None:
        # Unfiltered: memory buffer (for instant re-render on a filter
        # change) and the backup file both always get everything.
        self._all_lines.append(line)
        if self._file:
            try:
                self._file.write(line + '\n')
                self._file.flush()
            except Exception:
                pass

        tag = self._extract_tag(line)
        item = self._ensure_tag_known(tag)
        if item.checkState() == Qt.CheckState.Checked:
            self._text.appendPlainText(line)

        self._parse_live_values(tag, line)

    # ------------------------------------------------------------------
    # key=value parsing -> LIVE view + recording

    def _parse_live_values(self, tag: str, line: str) -> None:
        rest = line[len(tag):] if line.startswith(tag) else line
        for m in _KV_RE.finditer(rest):
            key, value = m.group(1), m.group(2)
            self._update_live_value(f'{tag} {key}', tag, key, value)
        if tag == '[LOOP]':
            hz = _HZ_RE.match(rest)
            if hz:
                self._update_live_value(f'{tag} Hz', tag, 'Hz', hz.group(1))

    # ------------------------------------------------------------------
    # Tag filter

    @staticmethod
    def _extract_tag(line: str) -> str:
        m = _TAG_RE.match(line)
        return m.group(1) if m else _OTHER_TAG

    def _ensure_tag_known(self, tag: str) -> QListWidgetItem:
        """Return this tag's checkbox item, creating it (checked by
        default) the first time this tag is ever seen."""
        item = self._tag_items.get(tag)
        if item is not None:
            return item
        item = QListWidgetItem(tag)
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
        item.setCheckState(Qt.CheckState.Checked)
        self._tag_list.addItem(item)
        self._tag_items[tag] = item
        return item

    def _set_all_tags(self, checked: bool) -> None:
        state = Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked
        self._tag_list.blockSignals(True)
        for item in self._tag_items.values():
            item.setCheckState(state)
        self._tag_list.blockSignals(False)
        self._render_filtered()

    @pyqtSlot('QListWidgetItem*')
    def _on_tag_item_changed(self, _item: QListWidgetItem) -> None:
        self._render_filtered()

    def _render_filtered(self) -> None:
        """Full rebuild of the visible log from self._all_lines, applying
        the current tag checkboxes. Only runs on an actual filter change
        (rare, user-triggered) -- the common case (a new line arriving) is
        a cheap single-line append in _on_line above, not this."""
        enabled = {tag for tag, item in self._tag_items.items()
                   if item.checkState() == Qt.CheckState.Checked}
        visible = [ln for ln in self._all_lines if self._extract_tag(ln) in enabled]
        self._text.setPlainText('\n'.join(visible))
        cursor = self._text.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        self._text.setTextCursor(cursor)

    # ------------------------------------------------------------------
    def _open_file(self) -> None:
        try:
            os.makedirs(_LOG_DIR, exist_ok=True)
            ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            self._file_path = os.path.join(_LOG_DIR, f'debug_{ts}.log')
            self._file = open(self._file_path, 'w', encoding='utf-8')
        except Exception:
            self._file = None
            self._file_path = None

    def _close_file(self) -> None:
        if self._file:
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                pass
        self._file = None
