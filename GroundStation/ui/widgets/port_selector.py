from PyQt6.QtCore import Qt
from core.serial_ports import list_port_choices


def refresh_port_combo(combo, preferred_role='telemetry'):
    current = combo.currentData()
    choices = list_port_choices()
    combo.clear()
    for choice in choices:
        combo.addItem(choice.label, choice.device)
        combo.setItemData(combo.count() - 1, choice.label, Qt.ItemDataRole.ToolTipRole)
    index = combo.findData(current) if current else -1
    if index < 0:
        matches = [i for i, choice in enumerate(choices) if choice.role == preferred_role]
        if len(matches) == 1:
            index = matches[0]
    if index >= 0:
        combo.setCurrentIndex(index)
    combo.view().setMinimumWidth(440)
    combo.setToolTip('USB roles are inferred from device interfaces. Debug / HITL depends on the flashed firmware. Use Refresh after plugging in.')
