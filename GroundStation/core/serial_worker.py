"""
QThread-based serial worker. Runs on its own thread, reads the COM port,
finds packet boundaries (telemetry and ack packets share the same wire),
validates checksums, and emits a signal for each valid packet.
Auto-reconnects on disconnection.
"""

import serial
import serial.tools.list_ports
import time
from typing import Optional

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from core.packet_decoder import (
    TelemetryData, decode_packet, find_packet_start,
    TELEM_MAGIC_0, TELEM_MAGIC_1, TELEM_SIZE,
    decode_ack, ACK_MAGIC_0, ACK_MAGIC_1, ACK_SIZE,
)


class SerialWorker(QObject):
    """
    Move this object to a QThread via moveToThread().
    Connect thread.started → run().
    """

    packet_received    = pyqtSignal(TelemetryData)
    ack_received        = pyqtSignal(int, int)   # (cmd_seq, cmd_type)
    connection_changed = pyqtSignal(bool, str)   # (connected, message)
    stats_updated      = pyqtSignal(int, float)  # (total_packets, packets_per_sec)

    def __init__(self, port: str, baud: int, parent: QObject = None):
        super().__init__(parent)
        self._port    = port
        self._baud    = baud
        self._running = False
        self._serial: Optional[serial.Serial] = None

        self._buf          = bytearray()
        self._total_pkts   = 0
        self._window_pkts  = 0
        self._window_start = time.time()

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

    def send_bytes(self, data: bytes) -> None:
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(data)
            except serial.SerialException:
                pass

    # ------------------------------------------------------------------
    # Internal helpers

    def _try_connect(self) -> None:
        try:
            self._serial = serial.Serial(
                self._port, self._baud,
                timeout=0.05,
                write_timeout=1.0,
            )
            self._buf.clear()
            self._total_pkts = 0
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

        if chunk:
            self._buf.extend(chunk)

        # Parse all complete packets from the buffer -- telemetry and ack
        # packets are interleaved on the same wire, distinguished by magic.
        while True:
            buf_bytes = bytes(self._buf)
            telem_idx = find_packet_start(buf_bytes, TELEM_MAGIC_0, TELEM_MAGIC_1)
            ack_idx   = find_packet_start(buf_bytes, ACK_MAGIC_0, ACK_MAGIC_1)

            candidates = [(i, 'telem', TELEM_SIZE) for i in (telem_idx,) if i != -1]
            candidates += [(i, 'ack', ACK_SIZE) for i in (ack_idx,) if i != -1]

            if not candidates:
                # No magic found; keep last byte in case next read completes it
                if len(self._buf) > 1:
                    self._buf = self._buf[-1:]
                return

            # Whichever magic pair appears earliest in the buffer goes first
            idx, kind, size = min(candidates, key=lambda c: c[0])

            # Discard bytes before magic
            if idx > 0:
                self._buf = self._buf[idx:]

            if len(self._buf) < size:
                return  # Wait for more bytes

            raw = bytes(self._buf[:size])

            if kind == 'telem':
                data = decode_packet(raw)
                if data is not None:
                    self._buf = self._buf[size:]
                    self._total_pkts  += 1
                    self._window_pkts += 1
                    self.packet_received.emit(data)

                    now = time.time()
                    elapsed = now - self._window_start
                    if elapsed >= 1.0:
                        rate = self._window_pkts / elapsed
                        self.stats_updated.emit(self._total_pkts, rate)
                        self._window_pkts  = 0
                        self._window_start = now
                else:
                    # Bad packet at this offset; skip one byte and retry sync
                    self._buf = self._buf[1:]
            else:
                ack = decode_ack(raw)
                if ack is not None:
                    self._buf = self._buf[size:]
                    self.ack_received.emit(ack.cmd_seq, ack.cmd_type)
                else:
                    self._buf = self._buf[1:]


def list_serial_ports() -> list[str]:
    """Return sorted list of available COM port names."""
    return sorted(p.device for p in serial.tools.list_ports.comports())
