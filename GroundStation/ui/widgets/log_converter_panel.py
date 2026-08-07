"""
Flight log converter -- turns a .BIN pulled off the flight computer's SD card
into .csv and/or MATLAB .mat. Pure local file utility: no telemetry, no
serial connection involved. Wraps the exact same parser as
tools/log_convert.py (the command-line version of this same tool) in a
file-picker UI instead of a terminal.
"""

import os
import importlib.util

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFrame, QFileDialog, QCheckBox
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui  import QFont

from tools.log_convert import parse_log, write_csv, write_mat
from ui.widgets.event_log import EventLog

_SCIPY_AVAILABLE = importlib.util.find_spec('scipy') is not None

_BG     = '#1A1A1B'
_CARD2  = '#222224'
_BORDER = '#2E2E30'
_TEXT   = '#F1F5F9'
_MUTED  = '#94A3B8'
_BLUE   = '#3B82F6'


def _divider() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFixedHeight(1)
    f.setStyleSheet(f'background-color:{_BORDER};margin:6px 0;')
    return f


def _section(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:0.8px;'
        f'border:none;background:transparent;'
    )
    return lbl


class LogConverterPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(
            f'background-color:{_BG};border:1px solid {_BORDER};border-radius:8px;'
        )

        self._input_path: str | None = None
        self._records: list[dict] | None = None

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 16)
        root.setSpacing(10)

        hdr = QHBoxLayout(); hdr.setSpacing(8)
        sym = QLabel('<>')
        sym.setStyleSheet(f'color:{_TEXT};font-size:12px;background:transparent;border:none;')
        ttl = QLabel('FLIGHT LOG CONVERTER')
        ttl.setStyleSheet(
            f'color:{_TEXT};font-size:11px;font-weight:800;letter-spacing:1px;'
            f'background:transparent;border:none;'
        )
        hdr.addWidget(sym); hdr.addWidget(ttl); hdr.addStretch()
        root.addLayout(hdr)
        root.addWidget(_divider())

        desc = QLabel(
            "Converts a raw .BIN flight log pulled off the flight computer's SD "
            "card into a .csv and/or MATLAB .mat file for analysis. The flight "
            "computer logs in this compact binary format on purpose -- this tool "
            "does the analysis-friendly conversion afterward, on your PC."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet(f'color:{_MUTED};font-size:11px;border:none;background:transparent;')
        root.addWidget(desc)

        # ---- Step 1: pick a file ----
        root.addWidget(_section('1. SELECT LOG FILE'))
        browse_row = QHBoxLayout(); browse_row.setSpacing(8)
        self._browse_btn = QPushButton('Browse...')
        self._browse_btn.setFixedHeight(28)
        self._browse_btn.setFont(QFont('Segoe UI', 10, QFont.Weight.Bold))
        self._browse_btn.clicked.connect(self._on_browse)
        self._browse_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_CARD2};color:{_TEXT};
                border:1px solid {_BORDER};border-radius:5px;
            }}
            QPushButton:hover {{ background-color:#222436; }}
        """)
        self._path_lbl = QLabel('No file selected')
        self._path_lbl.setStyleSheet(f'color:{_MUTED};font-size:11px;border:none;background:transparent;')
        browse_row.addWidget(self._browse_btn)
        browse_row.addWidget(self._path_lbl, stretch=1)
        root.addLayout(browse_row)

        self._summary_lbl = QLabel('')
        self._summary_lbl.setStyleSheet(
            f'color:{_TEXT};font-size:11px;font-weight:600;border:none;background:transparent;'
        )
        root.addWidget(self._summary_lbl)
        root.addWidget(_divider())

        # ---- Step 2: choose output formats ----
        root.addWidget(_section('2. OUTPUT FORMAT'))
        fmt_row = QHBoxLayout(); fmt_row.setSpacing(16)
        self._csv_chk = QCheckBox('CSV')
        self._csv_chk.setChecked(True)
        self._csv_chk.setStyleSheet(f'color:{_TEXT};font-size:11px;border:none;background:transparent;')
        self._mat_chk = QCheckBox('MATLAB (.mat)')
        self._mat_chk.setChecked(_SCIPY_AVAILABLE)
        self._mat_chk.setEnabled(_SCIPY_AVAILABLE)
        self._mat_chk.setStyleSheet(f'color:{_TEXT};font-size:11px;border:none;background:transparent;')
        if not _SCIPY_AVAILABLE:
            self._mat_chk.setToolTip('Requires scipy -- pip install scipy')
        fmt_row.addWidget(self._csv_chk)
        fmt_row.addWidget(self._mat_chk)
        fmt_row.addStretch()
        root.addLayout(fmt_row)

        if not _SCIPY_AVAILABLE:
            hint = QLabel('scipy not installed -- .mat export unavailable (pip install scipy)')
            hint.setStyleSheet(f'color:{_MUTED};font-size:10px;border:none;background:transparent;')
            root.addWidget(hint)

        root.addWidget(_divider())

        # ---- Step 3: convert ----
        root.addWidget(_section('3. CONVERT'))
        self._convert_btn = QPushButton('Convert & Save As...')
        self._convert_btn.setFixedHeight(32)
        self._convert_btn.setFont(QFont('Segoe UI', 11, QFont.Weight.Bold))
        self._convert_btn.setEnabled(False)
        self._convert_btn.clicked.connect(self._on_convert)
        self._convert_btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{_BLUE};color:#FFF;border:none;
                border-radius:6px;font-weight:700;letter-spacing:0.5px;
            }}
            QPushButton:hover {{ background-color:#60A5FA; }}
            QPushButton:pressed {{ background-color:#2563EB; }}
            QPushButton:disabled {{ background-color:{_CARD2};color:{_MUTED}; }}
        """)
        root.addWidget(self._convert_btn)
        root.addWidget(_divider())

        self._log = EventLog(title='CONVERSION LOG')
        root.addWidget(self._log, stretch=1)

    # ------------------------------------------------------------------
    def _on_browse(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, 'Select flight log', '', 'Flight logs (*.bin *.BIN);;All files (*.*)'
        )
        if not path:
            return
        self._input_path = path
        self._path_lbl.setText(os.path.basename(path))
        self._parse_selected()

    def _parse_selected(self) -> None:
        self._records = None
        self._convert_btn.setEnabled(False)
        self._summary_lbl.setText('')

        try:
            records = parse_log(self._input_path)
        except Exception as e:
            self._log.log(f'Failed to read {os.path.basename(self._input_path)}: {e}', level='error')
            return

        if not records:
            self._summary_lbl.setText('No complete records found in this file.')
            self._log.log(f'{os.path.basename(self._input_path)}: no complete records found.', level='warn')
            return

        self._records = records
        t0, t1 = records[0]['timestamp_ms'], records[-1]['timestamp_ms']
        self._summary_lbl.setText(
            f'{len(records)} records · {(t1 - t0) / 1000.0:.1f}s of flight time'
        )
        self._log.log(f'Parsed {os.path.basename(self._input_path)}: {len(records)} records.', level='ok')
        self._convert_btn.setEnabled(True)

    def _on_convert(self) -> None:
        if not self._records:
            return
        if not self._csv_chk.isChecked() and not self._mat_chk.isChecked():
            self._log.log('Select at least one output format.', level='warn')
            return

        base = os.path.splitext(os.path.basename(self._input_path))[0]

        if self._csv_chk.isChecked():
            path, _ = QFileDialog.getSaveFileName(self, 'Save CSV as', base + '.csv', 'CSV files (*.csv)')
            if path:
                try:
                    write_csv(self._records, path)
                    self._log.log(f'Wrote {path}', level='ok')
                except Exception as e:
                    self._log.log(f'CSV export failed: {e}', level='error')

        if self._mat_chk.isChecked():
            path, _ = QFileDialog.getSaveFileName(self, 'Save MATLAB file as', base + '.mat', 'MATLAB files (*.mat)')
            if path:
                try:
                    write_mat(self._records, path)
                    self._log.log(f'Wrote {path}', level='ok')
                except Exception as e:
                    self._log.log(f'MAT export failed: {e}', level='error')
