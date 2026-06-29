"""
Rocket GCS — Ground Control Station
Entry point. Run from the ground_station/ directory:
    python main.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui     import QFont

# pyqtgraph global config — must be set before any PlotWidget is created
import pyqtgraph as pg
pg.setConfigOption('background', '#FFFFFF')
pg.setConfigOption('foreground', '#6E6E73')
pg.setConfigOptions(antialias=True)

# ── Design System ──────────────────────────────────────────────────────────────
#
#   BG_APP   = #F5F5F7   off-white app background
#   BG_CARD  = #FFFFFF   card / panel
#   BLUE     = #0071E3   primary action (Apple blue)
#   GREEN    = #34C759   success / landed
#   ORANGE   = #FF9F0A   warning / powered ascent
#   RED      = #FF3B30   danger / fire
#   TEXT_PRI = #1D1D1F   near-black body text
#   TEXT_SEC = #6E6E73   grey secondary label
#   TEXT_TER = #8E8E93   tertiary / placeholder
#   BORDER   = #D2D2D7   card border
#   DIVIDER  = #E5E5EA   subtle divider
#
# ──────────────────────────────────────────────────────────────────────────────

STYLESHEET = """
/* ── Base ─────────────────────────────────────────────────── */
QMainWindow, QWidget {
    background-color: #F5F5F7;
    color: #1D1D1F;
    font-family: 'Segoe UI', 'SF Pro Display', Arial, sans-serif;
    font-size: 11px;
}

/* ── Cards (Group Boxes) ───────────────────────────────────── */
QGroupBox {
    background-color: #FFFFFF;
    border: 1px solid #D2D2D7;
    border-radius: 12px;
    margin-top: 22px;
    padding: 10px 12px 12px 12px;
    color: #8E8E93;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.6px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 14px;
    padding: 2px 6px;
    background-color: #F5F5F7;
    color: #8E8E93;
    border-radius: 3px;
}

/* ── Default Button ────────────────────────────────────────── */
QPushButton {
    background-color: #FFFFFF;
    color: #0071E3;
    border: 1px solid #D2D2D7;
    border-radius: 8px;
    padding: 8px 16px;
    font-size: 12px;
    font-weight: 600;
}
QPushButton:hover  { background-color: #F2F2F7; border-color: #0071E3; }
QPushButton:pressed { background-color: #E8E8EC; }
QPushButton:disabled {
    background-color: #F2F2F7;
    color: #C7C7CC;
    border-color: #E5E5EA;
}

/* ARM — solid blue fill */
QPushButton#armButton {
    background-color: #0071E3;
    color: #FFFFFF;
    border: none;
    border-radius: 8px;
    font-weight: 700;
    font-size: 12px;
    padding: 8px 16px;
}
QPushButton#armButton:hover   { background-color: #0077ED; }
QPushButton#armButton:pressed { background-color: #005BBB; }

/* DISARM — outlined blue */
QPushButton#disarmButton {
    background-color: transparent;
    color: #0071E3;
    border: 1.5px solid #0071E3;
    border-radius: 8px;
    font-weight: 700;
    font-size: 12px;
    padding: 8px 16px;
}
QPushButton#disarmButton:hover   { background-color: #E1F0FD; }
QPushButton#disarmButton:pressed { background-color: #C8E0F9; }

/* FIRE — solid red fill */
QPushButton#fireButton {
    background-color: #FF3B30;
    color: #FFFFFF;
    border: none;
    border-radius: 8px;
    font-weight: 700;
    font-size: 11px;
    padding: 7px 14px;
}
QPushButton#fireButton:hover   { background-color: #FF453A; }
QPushButton#fireButton:pressed { background-color: #D70015; }
QPushButton#fireButton:disabled {
    background-color: #F2F2F7;
    color: #C7C7CC;
    border: 1px solid #E5E5EA;
}

/* ── Combo Boxes ───────────────────────────────────────────── */
QComboBox {
    background-color: #FFFFFF;
    color: #1D1D1F;
    border: 1px solid #D2D2D7;
    border-radius: 8px;
    padding: 5px 10px;
    font-size: 12px;
}
QComboBox:hover   { border-color: #0071E3; }
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView {
    background-color: #FFFFFF;
    color: #1D1D1F;
    border: 1px solid #D2D2D7;
    border-radius: 8px;
    selection-background-color: #E1F0FD;
    selection-color: #0071E3;
    padding: 4px;
    outline: none;
}

/* ── Text Edit ─────────────────────────────────────────────── */
QTextEdit {
    background-color: #FFFFFF;
    color: #1D1D1F;
    border: none;
    font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
    font-size: 10px;
    padding: 4px;
    selection-background-color: #E1F0FD;
    selection-color: #0071E3;
}

/* ── Scroll Bars ───────────────────────────────────────────── */
QScrollBar:vertical {
    background: transparent;
    width: 6px;
    border: none;
    margin: 2px 0;
}
QScrollBar::handle:vertical {
    background: #C7C7CC;
    border-radius: 3px;
    min-height: 28px;
}
QScrollBar::handle:vertical:hover { background: #8E8E93; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }

/* ── Splitters ─────────────────────────────────────────────── */
QSplitter::handle { background-color: #E5E5EA; }
QSplitter::handle:horizontal { width: 1px; }
QSplitter::handle:vertical   { height: 1px; }
QSplitter::handle:hover { background-color: #0071E3; }

/* ── Status Bar ────────────────────────────────────────────── */
QStatusBar {
    background-color: #FFFFFF;
    color: #8E8E93;
    border-top: 1px solid #E5E5EA;
    font-size: 10px;
}

/* ── Labels ────────────────────────────────────────────────── */
QLabel { color: #1D1D1F; }

/* ── Message Boxes ─────────────────────────────────────────── */
QMessageBox { background-color: #FFFFFF; color: #1D1D1F; }
QMessageBox QPushButton { min-width: 88px; padding: 8px 22px; border-radius: 8px; }
"""


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName('Rocket GCS')
    app.setStyle('Fusion')
    app.setStyleSheet(STYLESHEET)
    app.setFont(QFont('Segoe UI', 11))

    from ui.main_window import MainWindow
    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
