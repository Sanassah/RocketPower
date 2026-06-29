# Rocket Ground Control Station

Python/PyQt6 GCS for the model rocket flight computer (Teensy 4.1 / NXP IMXRT1062).

## Features

- **Live telemetry** decoded from binary packets over LoRa serial link (9600 baud)
- **Flight state display** with colour coding and ARMED-state flash
- **Scrolling plots** — barometric altitude (60 s window, max-altitude marker) and acceleration magnitude (1 g reference)
- **2D rocket orientation** visual derived from BNO085 IMU quaternion
- **GPS map** (Leaflet.js via QWebEngineView) with launch site, rocket position and flight path polyline
- **Pyro channel status** (SAFE / ARMED / FIRED) for 3 channels
- **Command panel** — ARM, DISARM, FIRE PYRO (per-channel), PING — all with confirmation dialogs
- **CSV logging** to `logs/log_YYYY-MM-DD_HH-MM-SS.csv` on each connection
- **Event log** with timestamped state-change and command entries

## Packet format

The GCS decodes the flight computer's binary `TelemetryPacket` (78 bytes, magic `0xAA 0x55`) and encodes `CommandPacket` (6 bytes, magic `0xBB 0x44`). See `core/packet_decoder.py` for the full struct layout.

## Installation

```bash
pip install PyQt6 PyQt6-WebEngine pyserial pyqtgraph numpy folium
```

> **Note:** `PyQt6-WebEngine` is required for the GPS map widget. If it is not installed, the map panel shows an install hint.

## Running

```bash
cd ground_station
python main.py
```

## Project structure

```
ground_station/
├── main.py                       Entry point, dark stylesheet
├── core/
│   ├── packet_decoder.py         TelemetryData dataclass + binary decoder
│   ├── packet_encoder.py         CommandPacket encoder
│   ├── serial_worker.py          QThread serial reader with binary sync
│   └── data_logger.py            CSV session logger
├── ui/
│   ├── main_window.py            QMainWindow, layout, signal wiring
│   └── widgets/
│       ├── connection_panel.py   COM port / baud / connect controls
│       ├── state_panel.py        Flight state with ARMED flash
│       ├── sensor_panel.py       GPS info + sensor values
│       ├── altitude_plot.py      Scrolling baro altitude (pyqtgraph)
│       ├── accel_plot.py         Scrolling acceleration (pyqtgraph)
│       ├── gps_map.py            Leaflet map in QWebEngineView
│       ├── pyro_panel.py         Per-channel fire status
│       ├── command_panel.py      ARM / DISARM / FIRE PYRO buttons
│       ├── event_log.py          Timestamped scrolling event log
│       └── rocket_visual.py      2D QPainter rocket silhouette
└── assets/
    └── rocket.svg
```

## LoRa link

The flight computer sends packets at 200 ms intervals (5 Hz) over Serial1 → EBYTE E22 in transparent UART mode at 9600 baud. Select the COM port that corresponds to your LoRa USB-to-serial adapter and set baud to **9600**.
