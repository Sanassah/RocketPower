"""iOS-style pill toggle switch -- a small rounded track with an animated
sliding knob, used where a single binary mode needs to sit compactly beside
a card title (see ArmPanel's Static/Active Stabilization switch)."""

from PyQt6.QtWidgets import QWidget, QSizePolicy
from PyQt6.QtCore import Qt, QPropertyAnimation, QEasingCurve, pyqtSignal, pyqtProperty
from PyQt6.QtGui import QPainter, QColor, QPen

_TRACK_OFF = QColor('#3A3A3D')
_TRACK_ON  = QColor('#EF4444')   # red -- matches this app's existing convention for
                                  # the "real" (non-demo) attitude control action
_BORDER    = QColor('#2E2E30')
_KNOB      = QColor('#F1F5F9')


class PillSwitch(QWidget):
    """Binary toggle. Mirrors QAbstractButton's clicked()/setChecked() split
    on purpose: `clicked` fires ONLY on genuine user interaction, while
    `setChecked()` is silent and safe to call from a telemetry-driven sync
    without re-triggering a caller's confirmation-dialog handler."""

    clicked = pyqtSignal(bool)   # new state, user-initiated only

    def __init__(self, checked: bool = False, parent=None):
        super().__init__(parent)
        self._checked  = checked
        self._knob_pos = 1.0 if checked else 0.0
        self.setFixedSize(46, 24)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        self._anim = QPropertyAnimation(self, b'knobPos', self)
        self._anim.setDuration(150)
        self._anim.setEasingCurve(QEasingCurve.Type.InOutCubic)

    def isChecked(self) -> bool:
        return self._checked

    def setChecked(self, checked: bool) -> None:
        """Programmatic only -- never emits `clicked`."""
        if checked == self._checked:
            return
        self._checked = checked
        self._animate_to(1.0 if checked else 0.0)

    def _animate_to(self, target: float) -> None:
        self._anim.stop()
        self._anim.setStartValue(self._knob_pos)
        self._anim.setEndValue(target)
        self._anim.start()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._checked = not self._checked
            self._animate_to(1.0 if self._checked else 0.0)
            self.clicked.emit(self._checked)

    def _get_knob_pos(self) -> float:
        return self._knob_pos

    def _set_knob_pos(self, value: float) -> None:
        self._knob_pos = value
        self.update()

    knobPos = pyqtProperty(float, fget=_get_knob_pos, fset=_set_knob_pos)

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        track_color = QColor(
            int(_TRACK_OFF.red()   + (_TRACK_ON.red()   - _TRACK_OFF.red())   * self._knob_pos),
            int(_TRACK_OFF.green() + (_TRACK_ON.green() - _TRACK_OFF.green()) * self._knob_pos),
            int(_TRACK_OFF.blue()  + (_TRACK_ON.blue()  - _TRACK_OFF.blue())  * self._knob_pos),
        )
        p.setPen(QPen(_BORDER, 1))
        p.setBrush(track_color)
        p.drawRoundedRect(0, 0, w - 1, h - 1, h / 2, h / 2)

        knob_d = h - 6
        x = 3 + (w - knob_d - 6) * self._knob_pos
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(_KNOB)
        p.drawEllipse(int(x), 3, knob_d, knob_d)
