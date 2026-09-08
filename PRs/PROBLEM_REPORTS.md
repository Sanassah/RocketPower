# RocketPower — Problem Reports Log

Tracks known, unresolved hardware/firmware problems so investigation already
done isn't repeated or lost before the V&V campaign. Each entry: symptom,
what's been ruled out, current root-cause conclusion (if any), current
mitigation (if any), and what's left to actually close it.

When a PR is fixed: move its entry to the **Resolved** section at the bottom
with the resolution, don't delete it — the ruled-out list is often the most
valuable part for the next person (or future you) hitting something similar.

Last updated: 2026-09-08

---

## OPEN

### PR-001 — ADXL375 (200g accelerometer) reads non-physical noise at rest
**Status:** OPEN — root cause narrowed to hardware, not yet fixed
**Severity:** High (currently worked around; see mitigation)

**Symptom:** Magnitude swings between ~0.3g and 8g+ while the board sits
stationary on the bench. Should read a steady ~1.0g.

**Ruled out (each via a real bench test, not assumption):**
- LoRa/RF coupling — telemetry radio physically disconnected, noise unchanged
- I2C bus clock speed — reverted Wire2 400kHz → 100kHz, no change
- Torn multi-register reads — real bug found (`getEvent()`'s 3 separate I2C
  reads vs `getXYZ()`'s atomic burst read), fixed, did **not** resolve the
  noise (a real bug, just not the cause of this symptom)
- CS pin floating — user confirmed tied HIGH via 10kΩ to 3.3V on the schematic
- Table/mechanical vibration — user confirmed table motionless, PCB-mounted
- SELF_TEST register stuck set — read back directly: `DATA_FORMAT=0b00001011`,
  bit 7 clear
- SD card electrical proximity — card physically removed, no change
- ODR/bandwidth — swept the full range (1.56Hz, 100Hz, 3200Hz). Noise scaled
  in the expected direction (worse at 3200Hz, ~8g swings) but **did not
  disappear at 1.56Hz**, where datasheet-predicted noise is <6mg (should have
  been silent if this were ordinary sensor noise) — this is the test that
  conclusively rules out "just noise," at any bandwidth

**Conclusion:** hardware-layer fault specific to this unit/board — most
likely a marginal solder joint (intermittent contact reads exactly like this)
or a genuinely defective chip. Not fixable in software.

**Current mitigation (temporary, see `config.h`'s `TEMPORARY` comments for
exact revert instructions):**
- `StateMachine.cpp`/`BackupDeploy.cpp`'s liftoff/burnout gating switched
  from `highg_mag_g` (ADXL375) to `accel_mag_ms2` (BNO085
  `SH2_LINEAR_ACCELERATION`, gravity-removed) — safe because both thresholds
  (`IMU_LIFTOFF_ACCEL_THRESHOLD_MS2`=15.0, `IMU_BURNOUT_ACCEL_THRESHOLD_MS2`=2.0)
  sit far below the BNO085's clip ceiling
- Telemetry `accel_x/y/z_cg` also switched to the same BNO085 source
- ADXL375 is still read/logged/telemetried every loop (`highg_x/y/z_g`,
  `highg_mag_g`) for continued visibility, just no longer trusted for flight
  logic

**Known limitation of the mitigation:** the BNO085's onboard accelerometer is
a **fixed ±8g range** (confirmed in the CEVA/Bosch BNO08X datasheet, not
configurable). The current up-to-date OpenRocket simulation (Simulation 1/3
in `RocketPower_V1.ork`) predicts a peak boost acceleration of ~11.7g — above
that ceiling. This does **not** affect liftoff/burnout gating (thresholds are
far below 8g either way), but it means telemetry accel values and the
velocity-fusion IMU-integration step will clip/saturate near 8g during the
~1-2s of peak thrust until the ADXL375 is fixed. Data-quality gap only, not a
safety gap.

**Remaining actions:**
- [ ] Physically inspect/reflow the ADXL375's solder joints
- [ ] Swap-test with a spare ADXL375 unit if one becomes available
- [ ] Once fixed: revert `StateMachine.cpp`, `BackupDeploy.cpp`,
      `TelemetryManager.cpp` back to the ADXL375 source (search `TEMPORARY`
      in those files + `config.h`)

---

### PR-002 — Barometer readings corrupted by RF coupling during LoRa TX
**Status:** OPEN — root cause confirmed, only software-mitigated (not fixed
at the hardware level)
**Severity:** Medium (mitigated for flight-critical use, raw data still bad)

**Symptom:** Barometer-derived vertical velocity showed multi-m/s noise
spikes at rest, correlated with LoRa transmit activity ("super reactive to
the barometer").

**Investigation:**
- Bench capacitor swaps (100µF, 220µF+100µF parallel, 220µF alone, 470µF)
  tried on the barometer's supply rail — inconclusive/didn't fix it
- Disabling LoRa telemetry entirely (software) made the noise disappear —
  confirmed
- Physically disconnecting the telemetry radio also cleared it — confirmed
- **Root cause: RF/antenna-proximity coupling into the barometer**, not a
  power-rail issue as originally suspected (the capacitor changes were never
  the actual fix)

**Current mitigation (software only, protects the *fused* velocity, not the
raw sensor):**
- Complementary filter retuned (`VERT_VEL_ALPHA` 0.98 → 0.999) — math-derived
  in `config.h`'s comment (steady-state noise ≈ `sqrt(β/2)×σ` for a filter
  fed fresh noisy corrections every cycle)
- `MAX_PLAUSIBLE_VERT_SPEED_MS` (350 m/s) sanity gate added — rejects a
  single extreme RF-glitch baro sample outright rather than blending it in

**Remaining/open concern:** raw `baro_alt_m`/`pressure_hpa` are **not**
protected by the above — they're still noisy at the source whenever LoRa is
transmitting. `StateMachine::DESCENT`'s landed-detection reads raw
`baro_alt_m` directly (`fabsf(d.baro_alt_m - _landedRefAlt) <
LANDED_ALT_TOLERANCE_M`), not the fused/filtered value. **Needs explicit V&V
testing:** confirm LANDED detection isn't falsely delayed or re-triggered by
RF noise while telemetry is actively transmitting during descent.

**Remaining actions:**
- [ ] V&V test: verify DESCENT/LANDED behavior with telemetry actively
      transmitting, not just on the bench with it disabled
- [ ] Consider a hardware fix (ferrite bead / series resistor on the
      barometer's supply or signal lines, or antenna relocation/shielding) —
      discussed, never implemented
- [ ] If hardware fix lands, confirm whether `VERT_VEL_ALPHA`/
      `MAX_PLAUSIBLE_VERT_SPEED_MS` can be tightened back up

---

### PR-003 — SERVO_PREFLIGHT / SERVO_TEST commands lack flight-state gating
**Status:** OPEN — identified in full-project audit, not yet fixed
**Severity:** High (could freeze the flight loop mid-flight)

**Symptom (risk, not yet observed in the field):** `SERVO_PREFLIGHT` is a
blocking ~6s all-4-channel choreography; `SERVO_TEST` is a blocking sweep.
Neither checks the current `FlightState` before running. If either command
reaches the flight computer while armed or mid-flight (accidental ground
send, a duplicated/misrouted command), the main loop freezes for the
duration — no sensor reads, no telemetry, no state-machine updates — during
active flight.

**Remaining actions:**
- [ ] Add a flight-state guard in `TelemetryManager::_handleCommand()` (e.g.
      only allow `SERVO_PREFLIGHT`/`SERVO_TEST` when `state == IDLE` or
      `ARMED`-but-not-yet-launched; reject and log otherwise)
- [ ] Decide whether `SERVO_CENTER_ALL`/nudge commands need the same
      treatment (lower risk — non-blocking — but worth a deliberate decision,
      not an oversight)

---

### PR-004 — E22-900T22D can't be reconfigured in-circuit (M0/M1 hardwired to GND)
**Status:** OPEN — requires a PCB revision, not a firmware-only fix
**Severity:** Medium (telemetry works fine as-is; this blocks improving it)

**Symptom:** Air data rate is stuck at the factory default 2.4kbps, capping
reliable telemetry at ~1Hz (see `TELEMETRY_INTERVAL_MS`'s comment in
`config.h`). Investigated raising it (module supports up to 62.5kbps, see
the official E22-900T22D user manual) — the rate/UART-baud/parity live in
one register (REG0, address 0x03), changeable with a single `C0 03 01 <val>`
write command. But reaching **Configuration Mode** (Mode 2: M1=1, M0=0) to
send that command requires driving the module's M0 and M1 pins, and **this
board hardwires both directly to GND in copper** (`LORA_M0_PIN`/`M1_PIN` =
-1 in `config.h` — not just unrouted to the MCU, permanently tied on the
PCB itself), locking it in Mode 0 (Normal) forever.

**What's confirmed NOT needed:** the existing 4-pin JST-GH connection
(TXD/RXD/GND/VCC) is already everything required for the data link itself
at any air rate/baud — that doesn't change. Only two new control signals
(M0, M1) are needed to unlock in-circuit reconfiguration. AUX (busy/ready
handshake) is optional — a fixed settle-time delay can substitute for it if
pin budget is tight, so the real minimum is 2 pins, not 3.

**Blocker:** no spare/free GPIO currently available on this board's
existing routing to wire M0/M1 to. Needs checking against the actual
schematic for a genuinely unused MIMXRT1062 pad (the chip has far more
GPIOs than this board's headers/silkscreen currently expose) rather than
assuming none exist — or freeing up two pins by reassigning a
lower-priority function.

**Current workaround (works, but is a bench-only, every-time-you-want-to-
change-it procedure):** unplug the module's JST-GH cable, wire it to an
external 3.3V-logic USB-TTL adapter with jumpers forcing M1=3.3V/M0=GND,
send the config command at 9600 8N1 (fixed rate the module always uses in
Configuration Mode regardless of its stored operating baud), then restore
M0/M1 to GND and reconnect. No in-field or in-software reconfigurability
with this workaround — every future rate change means physically pulling
the module again.

**Remaining actions:**
- [ ] Audit the schematic for a genuinely free MIMXRT1062 GPIO pad (not
      just "no free header pin") — check `main.kicad_sch` and
      `HighCurrentComponents.kicad_sch` for what's actually unused
- [ ] If none exist, decide what lower-priority function to give up (or
      multiplex) to free 2 pins
- [ ] Next PCB rev: route M0/M1 from the E22 module's pads to those 2 GPIOs
      instead of tying them to GND; add pull-downs so the module still
      defaults to Mode 0 (Normal) if those pins are ever left floating
      (e.g. before the MCU's GPIOs are initialized at boot)
- [ ] Firmware: add `LORA_M0_PIN`/`LORA_M1_PIN` real pin numbers to
      `config.h`, add a one-time (or ground-commanded) config-mode routine
      to `LoRaRadio` that drives Mode 2, sends the REG0 write, confirms via
      read-back, then returns to Mode 0
- [ ] Once wired: revisit the air-rate/interval choice (9.6kbps / REG0=0xE4
      was the conservative first pick given a ~1km max range requirement,
      see chat history — field-test range before committing further)

---

## Resolved this session (reference — not exhaustive project history)

Kept short and only for items that came up as candidate PRs during this same
investigation pass, so they aren't second-guessed or re-investigated later.

- **Serial vs DEBUG_SERIAL bug** — was present in `LoRa.cpp`,
  `TelemetryManager.cpp`, `StateMachine.cpp`, `PyroController.cpp`,
  `FinController.cpp`, `CameraController.cpp`, `BackupDeploy.cpp`. All fixed;
  every plain debug print now goes to `DEBUG_SERIAL`.
- **BackupDeploy's `WAITING_FOR_ALTITUDE` had no timeout** — could get
  permanently stuck on one bad baro sample, silently disabling the entire
  backup-deploy safety net for the rest of the flight. Fixed with
  `BACKUP_ALTITUDE_TIMEOUT_MS` (45s) — re-arms `WAITING_FOR_BOOST` on
  timeout instead of forcing the apogee watch open.
- **README's Flight Procedure link broken** — both `README.md` files
  deleted outright (proper ones to be written at the end of the V&V
  campaign, per the user's call).
- **Unused config.h defines** — `GPS_DEBUG_RAW_NMEA` was dead (never wired
  up); now actually echoes raw NMEA to `DEBUG_SERIAL` when enabled.
  `SERVO_NUM_CHANNELS` was defined but ignored (`FinController` hardcoded
  `4` everywhere); now used consistently. `LORA_M0/M1/AUX_PIN`,
  `LORA_CHANNEL` left as-is — legitimate hardware documentation, not bugs.
  `PYRO_IGNITION` was already gone from the earlier pyro-channel remap.
- **Accelerometer.cpp hardcoded `Wire2`** instead of `ADXL375_I2C_BUS` —
  fixed; changing that macro now actually takes effect.
- **config.h's I2C Bus Assignments section was split in two** by later
  insertions — reordered back to contiguous.
- **FlightStates.h called the apogee event "drogue"** while the only
  channel is `PYRO_PARACHUTE` — this is a single-deploy design, "drogue"
  incorrectly implied an unbuilt dual-deploy stage. Renamed to "parachute".
- **FinController.h's file comment** was old investigation-journal style —
  condensed to current terse convention.

---

## Deferred / proposed (not problems — don't confuse with the PR list above)

- **Barometer/IMU-based liftoff cross-check**: offered twice during the
  ADXL375 investigation as a second independent liftoff signal. User hasn't
  said yes/no yet — not started.
