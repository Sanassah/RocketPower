"""
RocketPower GCS -- Ground Control Station
Entry point. Run from the ground_station/ directory:
    python main.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui     import QFont

import pyqtgraph as pg
pg.setConfigOption('background', '#0F0F10')
pg.setConfigOption('foreground', '#64748B')
pg.setConfigOptions(antialias=True)

STYLESHEET = """
QMainWindow, QWidget {
    background-color: #0F0F10;
    color: #F1F5F9;
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 11px;
}

QScrollBar:vertical {
    background: transparent;
    width: 5px;
    border: none;
}
QScrollBar::handle:vertical {
    background: #2A2A2C;
    border-radius: 2px;
    min-height: 28px;
}
QScrollBar::handle:vertical:hover { background: #64748B; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }

QSplitter::handle { background-color: #1E1E20; }
QSplitter::handle:horizontal { width: 1px; }
QSplitter::handle:vertical   { height: 1px; }

QStatusBar {
    background-color: #1A1A1B;
    color: #94A3B8;
    border-top: 1px solid #2E2E30;
    font-size: 11px;
}

QComboBox {
    background-color: #222224;
    color: #F1F5F9;
    border: 1px solid #2A2A2C;
    border-radius: 5px;
    padding: 3px 7px;
    font-size: 11px;
}
QComboBox:hover { border-color: #3B82F6; }
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView {
    background-color: #1A1A1B;
    color: #F1F5F9;
    border: 1px solid #2A2A2C;
    selection-background-color: #222224;
    selection-color: #3B82F6;
    outline: none;
}

QPushButton {
    background-color: #222224;
    color: #F1F5F9;
    border: 1px solid #2A2A2C;
    border-radius: 5px;
    padding: 6px 14px;
    font-size: 11px;
}
QPushButton:hover   { background-color: #2A2A2C; border-color: #64748B; }
QPushButton:pressed { background-color: #1A1A1B; }
QPushButton:disabled { color: #2A2A2C; border-color: #1E1E20; }

QTextEdit {
    background-color: #0F0F10;
    color: #F1F5F9;
    border: none;
    font-family: 'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace;
    font-size: 10px;
    padding: 4px;
    selection-background-color: #222224;
    selection-color: #3B82F6;
}

QLabel { color: #F1F5F9; }

QMessageBox {
    background-color: #1A1A1B;
    color: #F1F5F9;
}
QMessageBox QPushButton {
    min-width: 80px;
    padding: 7px 18px;
    border-radius: 5px;
}
"""


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName('RocketPower GCS')
    app.setStyle('Fusion')
    app.setStyleSheet(STYLESHEET)
    app.setFont(QFont('Segoe UI', 11))

    from ui.main_window import MainWindow
    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
