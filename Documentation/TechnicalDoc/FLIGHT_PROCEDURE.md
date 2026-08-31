# Flight Procedure

Step-by-step operating instructions for the RocketPower flight computer and
ground station — bench checkout, pad procedure, launch, recovery, and data
review. For code/architecture docs see `FlightComputer/` and
`GroundStation/README.md`; this file is about *running* the system, not how
it's built.

---

## 0. What you're working with

- **Flight computer**: custom PCB (`FlightComputer/`), programmed via the
  Teensy 4.1 toolchain. Runs autonomously once powered — it does not need
  the ground station connected to fly correctly. The ground station is for
  visibility and manual commands (arm, fire, calibrate), not for the flight
  logic itself.
- **Ground station** (`GroundStation/`): the PyQt6 app (`python main.py`).
  Three pages, left sidebar: **OVERVIEW** (GPS map, trajectory, 3D
  orientation, pyro/continuity/arm panels), **TELEMETRY** (sensor status,
  commands, event log), **DATA TOOLS** (post-flight `.BIN` → CSV/MATLAB
  converter).
- **Two ways to connect the ground station**: directly via USB to the
  flight computer (bench testing — fast, but only works with a cable
  attached), or via a ground-side LoRa receiver dongle at **9600 baud**
  (the actual flight link, works untethered). Pick the right COM port/baud
  in the sidebar before connecting.

---

## 1. Bench checkout (do this before you ever go to the pad)

1. Flash the real flight firmware if you haven't recently:
   ```
   pio run -e rocketpower -t upload
   ```
2. Power the flight computer and let it boot. It self-tests every sensor
   and the SD card, then halts (flashing the status LEDs) if the IMU or
   barometer failed — those two are flight-critical and it won't proceed
   without them. Everything else (GPS, accelerometer, power monitor, SD)
   is allowed to fail at boot and just gets flagged, not fatal.
3. Connect the ground station over USB and open the **TELEMETRY** page.
   Check the **STATUS** section: IMU / Barometer / Accelerometer / GPS
   Module / Power Monitor should all read **OK**, and **SD Card** should
   read **IDLE** (present, not yet recording — recording starts
   automatically once you arm, see §3) or **RECORDING** if a card was
   present at boot (it auto-opens a log immediately, see §4).
   - If SD reads **NO CARD**: insert one and press **Start Log** in the
     SD CARD section of the command panel to retry — it doesn't
     re-poll on its own (see `DataLogger::cardPresent()`'s doc comment
     for why).
4. Check pyro continuity on the OVERVIEW page's continuity panel — each
   channel should show continuity if an ematch is connected (or "open" if
   deliberately left disconnected for a bench test).
5. Optional: run a fin servo preflight sequence (TELEMETRY page → FIN
   SERVOS section → **PREFLIGHT**) to visually confirm all 4 channels move
   correctly before you close up the airframe.
6. Optional: toggle the camera on/off once (CAMERA section) to confirm the
   UART link is alive — it also self-probes at boot and reports to Serial.

---

## 2. Pad procedure

1. Mount the rocket vertically on the pad, **stationary** — the next step
   calibrates the barometer against whatever altitude it's sitting at
   right now, so don't move it after this point.
2. Connect the ground station via the LoRa link (not USB, unless you're
   doing a tethered test) and confirm telemetry is flowing (state should
   read **IDLE**, sensor values should look sane, GPS should have a fix if
   you want position data during descent/recovery).
3. Press **ARM** (ARM CONTROL section, or the dedicated arm panel on
   OVERVIEW). This asks for confirmation, then:
   - Arms the pyro channels (fire commands only work while armed).
   - Re-calibrates the barometer at the pad — this is why the rocket needs
     to be stationary and already in its final position.
   - Starts the camera recording, if not already running.
   - SD logging is already running by this point (started automatically at
     boot) — arming doesn't gate it, it's just confirmed by the SD status
     row.
4. Double check the SD status row now reads **RECORDING** and pyro
   continuity still looks correct before proceeding — this is your last
   easy checkpoint before the log starts mattering.

---

## 3. Launch

- If your motor uses **electronic ignition**: press **Fire CH 1 —
  Ignition** in the IGNITION CONTROL section. This is a real, irreversible
  action (confirmation dialog says so) — it's the actual launch trigger.
- If you're hand-lighting the motor: ARM is still required first (it's
  what enables liftoff detection and parachute deployment logic), but
  ignition itself happens outside the electrical system.

From here the flight computer runs itself — you're a spectator with an
abort/backup option (§5), not a pilot.

---

## 4. What happens automatically in flight

The state machine (`StateMachine.cpp`) transitions on its own based on
sensor data, no ground input needed:

| Transition | Trigger |
|---|---|
| ARMED → POWERED_ASCENT | High-g accel > 2.5g, held for 100ms (liftoff) |
| POWERED_ASCENT → COAST | High-g accel drops below 1.0g (burnout) |
| COAST → APOGEE | Vertical velocity crosses from + to -, held for 200ms |
| APOGEE → DESCENT | Immediate — parachute (CH2) fires automatically here |
| DESCENT → LANDED | Altitude stable within 2m for 10 continuous seconds |

Also automatic: the camera stops recording on LANDED, the SD log flushes
on every state change (bounding data loss) and closes fully on LANDED,
and telemetry keeps transmitting at 1Hz over LoRa the whole time
regardless of what the ground station is doing.

---

## 5. If something looks wrong mid-flight

- **Parachute didn't seem to deploy** (altitude/velocity on the ground
  station not showing a slow-down after apparent apogee): press
  **Fire CH 3 — Backup** manually. This works any time after ARM, not
  just during DESCENT — it's a manual override independent of the state
  machine.
- **Telemetry drops out**: the flight computer doesn't care — it keeps
  flying/logging/recording regardless of whether the ground station is
  receiving. Nothing to do but wait for the link to come back (or recover
  by GPS coordinates / last known position).

---

## 6. Recovery and data review

1. Retrieve the rocket. The SD log is already closed and safe (closed
   automatically on LANDED) — no need to power it off first, but do so
   before opening the airframe to swap the card.
2. Pull the SD card, find the newest `FLTxxxxx.BIN` file.
3. Convert it for analysis — either:
   - **Ground station GUI**: DATA TOOLS page → Browse → select the file →
     check CSV and/or MATLAB → **Convert & Save As...**.
   - **Command line**: `python GroundStation/tools/log_convert.py FLTxxxxx.BIN`
     (writes CSV and, if `scipy` is installed, `.mat` next to the input
     file by default).
4. The ground station's own local CSV log (separate from the flight
   computer's SD card — see the "⏺ REC" button in the TELEMETRY page's
   event log header) captured whatever telemetry it received live, useful
   as a cross-check but lower rate/resolution than the onboard SD log.

---

## Quick troubleshooting

| Symptom | Likely cause |
|---|---|
| FC halts at boot, LEDs flashing | IMU or barometer failed — check wiring/power before anything else |
| SD status stuck on "NO CARD" after inserting one | It doesn't auto-repoll while idle (by design — see §1.3); press **Start Log** |
| Telemetry rate unstable / bounces to 0 | Check `USB_SERIAL_BINARY_MIRROR`/reflash status if using USB |
| USB shows no telemetry at all | `USB_SERIAL_BINARY_MIRROR` must be `1` in `config.h` for a direct-USB ground station connection |
| Ground station won't connect | Right COM port? Right baud (9600 for the LoRa dongle, doesn't matter much for direct USB)? |
