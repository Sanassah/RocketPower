"""Read-only USB metadata discovery; never opens ports or sends commands."""
from dataclasses import dataclass
import re
import serial.tools.list_ports


@dataclass(frozen=True)
class PortChoice:
    device: str
    label: str
    role: str = ''


def _windows_interfaces():
    """pyserial sometimes omits the composite interface number on Windows."""
    result = {}
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                            r'SYSTEM\CurrentControlSet\Enum\USB') as root:
            for i in range(winreg.QueryInfoKey(root)[0]):
                name = winreg.EnumKey(root, i)
                match = re.fullmatch(r'VID_16C0&PID_048B&MI_([0-9A-F]{2})', name.upper())
                if not match:
                    continue
                with winreg.OpenKey(root, name) as device:
                    for j in range(winreg.QueryInfoKey(device)[0]):
                        instance = winreg.EnumKey(device, j)
                        try:
                            with winreg.OpenKey(device, instance + r'\Device Parameters') as params:
                                port = winreg.QueryValueEx(params, 'PortName')[0]
                                result[port] = int(match[1], 16)
                        except OSError:
                            continue
    except (ImportError, OSError):
        pass
    return result


def describe_port(port, interfaces):
    role = ''
    if (port.vid, port.pid) == (0x16C0, 0x048B):
        interface = interfaces.get(port.device)
        if interface is None:
            match = re.search(r'MI_([0-9A-F]{2})', port.hwid or '', re.I)
            if match:
                interface = int(match[1], 16)
        role = {0: 'telemetry', 2: 'secondary'}.get(interface, '')
        name = {'telemetry': 'Telemetry / commands',
                'secondary': 'Debug / HITL', '': 'USB role unknown'}[role]
        # The USB identity is shared by Teensy-based boards, not unique to this rocket.
        label = f'{name} — Teensy / RocketPower ({port.device})'
    else:
        name = port.product or port.description or 'Serial port'
        name = re.sub(r'\s*\(' + re.escape(port.device) + r'\)$', '', name)
        label = f'{name} ({port.device})'
    return PortChoice(port.device, label, role)


def list_port_choices():
    interfaces = _windows_interfaces()
    return [describe_port(p, interfaces) for p in sorted(serial.tools.list_ports.comports())]
