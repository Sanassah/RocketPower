# Rocket Ground Control Station

Python/PyQt6 GCS for the RocketPower flight computer (custom PCB, NXP IMXRT1062, programmed via the Teensy 4.1 toolchain/bootloader).

For the full operating procedure (arm, launch, recover, review data), see [`../FLIGHT_PROCEDURE.md`](../FLIGHT_PROCEDURE.md). This README covers the app itself.

## Features

Three pages, left sidebar:

- **OVERVIEW** — GPS map (custom-drawn, tile-fetching + QPainter, no WebEngine dependency), scrolling trajectory plot, live 3D rocket orientation (renders your actual STL model, driven by the BNO085 quaternion), pyro channel status, per-channel continuity, and an arm/disarm panel.
- **TELEMETRY** — sensor panel (live health for IMU/baro/accelerometer/GPS/power/SD card/attitude-control mode — re-checked every packet, not just at boot), command panel (ARM/DISARM, per-channel pyro fire, fin servo bench-test controls, camera toggle, SD start/stop, attitude control real/demo mode toggles), and a timestamped event log with its own local CSV recorder ("⏺ REC" — separate from the flight computer's own SD card).
- **DATA TOOLS** — pulls a `.BIN` log off the flight computer's SD card and converts it to CSV and/or MATLAB `.mat` for analysis.

Other things worth knowing:
- **Commands are ack-confirmed**: every command carries a sequence number, gets acked by the firmware before it's even executed, and auto-retries (piggybacked on telemetry reception) until acked or it gives up — safe to retry blindly, including pyro fire, since the firmware dedupes by sequence number.
- **Telemetry rate depends on the link**: 1 Hz over the LoRa radio (airtime-limited, see `TELEMETRY_INTERVAL_MS` in `FlightComputer/src/config.h`) but up to 20 Hz over a direct USB connection, which has no such constraint.
- **Sensor/SD health is live, not a boot-time snapshot** — a mid-flight sensor dropout or a card pulled on the bench shows up in the STATUS section within about half a second.

## Packet format

Binary, little-endian, magic-prefixed. See `core/packet_decoder.py` and `core/packet_encoder.py` for the authoritative field-by-field layout (kept in sync with `FlightComputer/src/telemetry/Packet.h`):

- `TelemetryPacket` (rocket → ground): 52 bytes, magic `0xAA 0x55`.
- `CommandPacket` (ground → rocket): 7 bytes, magic `0xBB 0x44`.
- `AckPacket` (rocket → ground): 6 bytes, magic `0xAC 0x4B`.

## Installation

```bash
pip install PyQt6 pyqtgraph pyserial numpy
```

Optional:
```bash
pip install scipy   # only needed for .mat export in tools/log_convert.py and DATA TOOLS; CSV export works without it
```

## Running

```bash
cd GroundStation
python main.py
```

## Project structure

```
GroundStation/
├── main.py                          Entry point, dark stylesheet, QApplication setup
├── core/
│   ├── packet_decoder.py            TelemetryData dataclass + binary decoder
│   ├── packet_encoder.py            CommandPacket encoder, CommandType enum
│   ├── serial_worker.py             QThread serial reader, magic-byte resync, link stats
│   └── data_logger.py               Ground station's own local CSV session logger
├── tools/
│   └── log_convert.py               CLI: flight computer .BIN -> CSV/.mat
├── ui/
│   ├── main_window.py               QMainWindow, page/sidebar layout, all signal wiring
│   └── widgets/
│       ├── top_bar.py                Connection status, link stats
│       ├── gps_map.py                Custom-drawn map (no WebEngine)
│       ├── trajectory_plot.py        Scrolling altitude/velocity/accel plots (pyqtgraph)
│       ├── rocket_visual.py          3D rocket orientation (renders assets/rocket.stl)
│       ├── pyro_panel.py             Per-channel armed/fired status
│       ├── continuity_panel.py       Per-channel continuity display
│       ├── arm_panel.py              Dedicated arm/disarm control
│       ├── sensor_panel.py           Live sensor + SD card health, GPS/baro/power readout
│       ├── command_panel.py          ARM/DISARM/FIRE, fin servo bench test, camera, SD log
│       ├── event_log.py              Timestamped scrolling event log (reused for the
│       │                             DATA TOOLS conversion log too)
│       └── log_converter_panel.py    DATA TOOLS page: file picker + CSV/.mat export
└── assets/
    ├── rocket.stl                   Real CAD model rendered in the 3D orientation view
    └── rocket.svg
```

> `ui/widgets/accel_plot.py`, `altitude_plot.py`, `connection_panel.py`, and `state_panel.py` still exist in the tree but aren't imported anywhere anymore — superseded by `trajectory_plot.py`, the sidebar's built-in connection controls, and `sensor_panel.py`/`top_bar.py` respectively. Left in place rather than deleted without being asked to; safe to remove if you don't need them for reference.

## Connecting

Two ways to connect, both from the sidebar port/baud selectors:

- **Direct USB** to the flight computer — for bench testing. Baud mostly doesn't matter (native USB serial), 115200 is the default.
- **LoRa ground receiver dongle** — the actual flight link. Set baud to **9600** to match the airborne radio's UART speed (`LORA_BAUD` in `config.h`), and pick the COM port your dongle enumerates as.
