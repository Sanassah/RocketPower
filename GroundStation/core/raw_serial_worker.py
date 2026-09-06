"""
QThread-based worker for the flight computer's second USB serial port
(SerialUSB1 / DEBUG_SERIAL in config.h), which now carries ONLY plain
human-readable bench diagnostic text -- loop/sensor/GPS/SD timing
breakdowns, init messages, etc. -- kept off the main telemetry port
specifically so it can't interleave with (and corrupt the readability of)
the binary packet stream SerialWorker parses. See FlightComputer's
config.h DEBUG_SERIAL macro for the firmware side of this split.

Unlike SerialWorker, there's no packet framing here -- just line-buffered
text -- so this is deliberately much simpler: read bytes, split on
newlines, emit each complete line.
"""

import serial
import time

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot


class RawSerialWorker(QObject):
    """
    Move this object to a QThread via moveToThread().
    Connect thread.started -> run().
    """

    line_received       = pyqtSignal(str)
    connection_changed  = pyqtSignal(bool, str)   # (connected, message)

    def __init__(self, port: str, baud: int, parent: QObject = None):
        super().__init__(parent)
        self._port    = port
        self._baud    = baud
        self._running = False
        self._serial: serial.Serial | None = None
        self._buf = bytearray()

    @pyqtSlot()
    def run(self) -> None:
        self._running = True
        while self._running:
            if self._serial is None or not self._serial.is_open:
                self._try_connect()
                if self._serial is None:
                    time.sleep(2.0)
                    continue
            self._read_step()
        self._close()

    def stop(self) -> None:
        self._running = False

    # ------------------------------------------------------------------
    # Internal helpers

    def _try_connect(self) -> None:
        try:
            self._serial = serial.Serial(self._port, self._baud, timeout=0.1)
            self._buf.clear()
            self.connection_changed.emit(True, f'Connected to {self._port} @ {self._baud} baud')
        except serial.SerialException as exc:
            self._serial = None
            self.connection_changed.emit(False, f'Failed to open {self._port}: {exc}')

    def _close(self) -> None:
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self.connection_changed.emit(False, 'Disconnected')

    def _read_step(self) -> None:
        try:
            chunk = self._serial.read(256)
        except serial.SerialException as exc:
            self._serial = None
            self.connection_changed.emit(False, f'Serial error: {exc}')
            return

        if not chunk:
            return
        self._buf.extend(chunk)

        # Emit one signal per complete line; leave any trailing partial
        # line in the buffer for the next read to complete.
        while True:
            nl = self._buf.find(b'\n')
            if nl == -1:
                break
            raw_line = bytes(self._buf[:nl])
            self._buf = self._buf[nl + 1:]
            line = raw_line.decode('utf-8', errors='replace').rstrip('\r')
            self.line_received.emit(line)
