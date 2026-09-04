# HITL (hardware-in-the-loop) testing

Runs the **real, compiled** flight firmware — `StateMachine`, `BackupDeploy`,
`AttitudeController`, `FinController`, `PyroController`, all byte-identical
to the flight build — on the actual Teensy 4.1, fed synthetic sensor data
from `RocketPowerSim.slx`'s 6DOF/aero model in real time, with the real fin
commands it computes closing the loop back into the simulated aero/actuator
response. This is what `config.h`'s pre-flight checklist means by "validated
in simulation (closed-loop)" before ever sending `ATTITUDE_CONTROL_ENABLE`
for a real flight — open-loop telemetry replay can't test this because
there's nothing for a replayed fin command to act on.

**One model, one switch.** `RocketPowerSim.slx` handles both plain
simulation and HITL now — double-click the **"HITL Mode Select"** Manual
Switch on the canvas to flip between them. It defaults to Normal Sim (no
hardware needed, same behavior as before HITL existed); flip it to HITL
only when you actually want a Teensy in the loop.

## What's real vs. simulated (in HITL mode)

| | Real | Simulated |
|---|---|---|
| Control code (state machine, attitude PID, fin allocation/clamping, pyro logic) | ✅ actual Teensy, actual compiled firmware | |
| Sensors (IMU, baro, high-g accel) | | ✅ injected over serial, from the 6DOF plant |
| Rigid-body dynamics, aero, servo lag/rate-limit | | ✅ `RocketPowerSim.slx`, unmodified downstream of the switch |
| Fin servos, pyro MOSFETs, SD card, LoRa | ✅ real hardware (harmless on a bench with no ematches/props attached) | |

## One-time setup

1. Flash the HITL build (not the flight firmware):
   ```
   cd FlightComputer
   pio run -e rocketpower_hitl -t upload
   ```
   This is a separate PlatformIO environment (`platformio.ini`) — the
   default `rocketpower` env (real flight firmware) is untouched by any of
   this.
2. Find the Teensy's **second** USB serial port (`SerialUSB1` on the device
   side) in Device Manager — it shows up as a second "USB Serial Device"
   alongside the one you already use for the ground station. The first port
   still carries normal command RX and the existing telemetry mirror
   completely unmodified — you can have the real GroundStation app
   connected over it and watch a HITL run live at the same time.
3. Open `RocketPowerSim.slx`, go inside the **"HITL Bridge"** subsystem,
   double-click `TeensyBridge`, and set `ComPort` to that second port.

## Running

- **Plain simulation**: leave "HITL Mode Select" at its default (Normal
  Sim) and run the model exactly as before — nothing about this changed,
  no hardware involved, same speed.
- **HITL**: double-click "HITL Mode Select" to flip it to HITL, then run.
  It runs in **real time** — `TeensyBridge` paces itself to wall clock
  every step, because the firmware's own safety timing (liftoff-confirm,
  apogee-detect window, backup-fire delay) is driven by the Teensy's
  hardware `millis()`, which nothing here can override (see the header
  comment on `SensorInjectPacket` in
  `FlightComputer/src/telemetry/Packet.h`). A 30s simulated flight takes
  ~30s.

`tout`/`pos_log`/`q_log` land in the base workspace exactly like before, so
`animate_flight.m` works unchanged either way.

To exercise ARM → liftoff → apogee → landed in HITL, send the same
`ARM`/`DISARM`/`FIRE_PYRO` commands you'd use from the ground station, over
the Teensy's **first** serial port, at whatever point in the run you want
them to take effect — the state machine reacts to them exactly as it would
in flight. Active fin control is a separate opt-in from ARM — send
`ATTITUDE_CONTROL_ENABLE` (works any time before liftoff, survives ARM) or
`ATTITUDE_DEMO_ENABLE` (takes effect in `IDLE` or `ARMED`, no need to arm
first) if you want the fins to actually respond.

### Debug scopes (inside "HITL Bridge")

- **"HITL Firmware State"** — the real firmware's own `FlightState` this
  step (`0`=IDLE, `1`=ARMED, `2`=POWERED_ASCENT, `3`=COAST, `4`=APOGEE,
  `5`=DESCENT, `6`=LANDED, `-1`=no response yet). Scope it alongside
  `rollCmd`/`pitchCmd`/`yawCmd` (`HITL Bridge`'s own outputs, or tap them
  where they feed the `Mode Switch N` blocks at the top level) if a command
  looks like it's still moving when you think it shouldn't be — this
  model's scopes plot against *simulation* time, while GroundStation's
  display is timestamped off the Teensy's own `millis()`, so
  cross-referencing "what I saw in GroundStation" against a Simulink
  scope's x-axis isn't reliable. The state output settles that directly: a
  command can only still be actively changing while state reads `2`
  (`POWERED_ASCENT`) or `3` (`COAST`).
- **"HITL Sensors (highg, tiltDeg, altitude, vertVel, gyro)"** — the raw
  plant-side signals `TeensyBridge` derives from the sensor stream:
  `highg` (1x3, g's, the exact specific-force vector sent to the firmware
  — same thing `StateMachine`/`BackupDeploy` threshold on via its
  magnitude), `tiltDeg` (angle from identity quaternion, not necessarily
  "from vertical" — see `TeensyBridge.m`'s class header), `altitude` (m,
  up-positive), `vertVel` (m/s, up-positive), `gyro` (1x3 rad/s). Valid
  every step regardless of whether the firmware actually responded, since
  they're computed straight from the injected sensor state, not the reply
  — useful for isolating a plant/physics issue from a firmware/control-loop
  one (e.g. watching whether `highg`/`tiltDeg` misbehave the same way even
  with `ATTITUDE_CONTROL_ENABLE` never sent, which points at
  `RocketPowerSim.slx`'s own aero model rather than anything in
  `FlightComputer/`).

## How the bridge works (`TeensyBridge.m`)

Lives inside the **"HITL Bridge"** Enabled Subsystem (quat/gyro/Xe in,
roll/pitch/yaw out), which only actually runs while "HITL Mode Select" is
set to HITL — see "Why an Enabled Subsystem" below for why that matters.
Each discrete step (default `Ts = 0.01s`, matching the firmware's real
100Hz loop):
1. Paces to wall clock.
2. Packs quaternion + body rate + position (derived from the `6DOF
   (Quaternion)` block's outputs — see the port mapping below) into a
   `SensorInjectPacket` and writes it to the Teensy.
3. Blocks for one `HITLResponsePacket` back — the real firmware's actual
   decision, fin clamping included.
4. Recovers pre-allocation roll/pitch/yaw commands from the 4 real per-fin
   degrees (`CH1=S, CH2=E, CH3=N, CH4=W` — see `AttitudeController.h`).

At the top level, each axis has a `Switch` block picking between "HITL
Bridge"'s recovered command and the model's own original PID output
(`Sum`/`Sum2`/`Sum3`), controlled by the same "HITL Mode Select" signal —
either way the result feeds the same actuator chain (`15deg sat` →
`1000deg/s` → `Servo lag (τ=15ms)` → moment/aero chain), unchanged.

### Why an Enabled Subsystem, not just a switch downstream

`TeensyBridge` opens its serial port **lazily**, on its first real
`stepImpl()` call, not in `setupImpl()`. This was a deliberate fix, and
matters if you ever restructure this: `matlab.System`'s `setupImpl` runs
once during Simulink's initialization phase for **every** block in the
diagram, *including* ones inside a currently-disabled Enabled
Subsystem — confirmed empirically. An Enabled Subsystem's enable signal
only gates `stepImpl`, not one-time setup. So if the port were opened in
`setupImpl` instead, a plain Normal Sim run would still require the Teensy
plugged in and running `rocketpower_hitl`, defeating the entire point of
the switch. Wrapping "HITL Bridge" in an Enabled Subsystem (rather than
just switching its output downstream, with the subsystem always running)
is what makes `stepImpl` — and therefore the port-open — only ever happen
when actually enabled.

### "roll"/"pitch"/"yaw" axis convention

This model (see `Constants.m`'s own header comment, `"spin about z = your
'yaw'"`) uses a **drone-style** convention: `"roll"` and `"pitch"` are the
two transverse-tilt channels (`Fin Moment Arm Roll`/`Pitch`, gain
`Cn_alpha`), and `"yaw"` is spin about the rocket's own longitudinal axis
(`Rocket Diameter2`, gain `Cl_delta` — confirmed by tracing each `*deg sat`
block forward to its moment-arm gain).

`FlightComputer/src/config.h`/`AttitudeController.cpp` use this SAME
convention (previously they used the opposite, standard-aerospace one,
where "roll" is the spin axis) — see config.h's axis-naming comment above
the `ATTITUDE_ROLL_RATE`/etc. macros. So each `Mode Switch N` block wires
`HITL Bridge` output `i` to actuator row `i` — a plain name-to-name mapping
is correct.

**This matters if you ever touch either side's axis convention again**: a
same-named-but-wrong mapping is silently wrong, not an error — it applies
the firmware's spin correction as a tilt moment and one of its tilt
corrections as a spin moment, so the controller can never actually reduce
the real error on 2 of 3 axes. That looked exactly like "the controller
won't converge" (saturating, growing commands), with no error thrown
anywhere. If you rename an axis on either side, re-derive whether the
mapping needs to go back to an explicit physics-based remap instead of
row-to-row.

### `6DOF (Quaternion)` output port mapping

Not documented anywhere in the model — empirically confirmed by probing a
live run against `Constants.m`'s own `eul_0`/`pm_0` initial conditions
(out3 matched `eul_0` exactly; out7 matched `pm_0` exactly; out4 matched
the quaternion equivalent of `eul_0`'s rotation) and, separately, by
differentiating every 3-wide output against every other over a real
flight:

| Port | Signal | Used by the bridge? |
|---|---|---|
| out1 | `Ve` — world-frame **velocity** (confirmed: `d(out2)/dt ≈ out1` almost exactly over a full flight) | not used directly |
| out2 | `Xe` — position `[x y z]`, m. **`Xe(3)` is UP-positive** (confirmed empirically against a full ascent/descent run — apex altitude and a clean positive→negative vertical-velocity crossing right at apex). This is *not* the NED Z-down convention `animate_flight.m`'s own `alt = -pos(:,3)` comment describes for `pos_log` — that variable evidently goes through additional processing this bridge doesn't touch (it taps out2 directly) | ✅ → altitude (`Xe(3)`, no negation); differentiated twice (world velocity, then world acceleration) for `vertVel` and body-frame kinematic accel |
| out3 | Euler angles, rad | not used |
| out4 | quaternion `[w x y z]` | ✅ → `quat_w/x/y/z`; also used to rotate world-frame quantities (acceleration, gravity) into body frame |
| out5 | DCM (quaternion→DCM) | not used |
| out6 | `Vb` — body-frame velocity | not used |
| out7 | `Wb` — body angular rate, rad/s | ✅ → `gyro_x/y/z` |

**The `6DOF (Quaternion)` block has no acceleration output at all.** Body-
frame kinematic acceleration is derived by the bridge itself: double-
differentiate `Xe` (world position) to get world-frame acceleration, then
rotate into body frame with the quaternion via `quatRotateWorldToBody.m`
(the same helper used for gravity, below). This replaced an earlier,
wrong version of this bridge that fed `out1` in as if it were acceleration
— it isn't, it's velocity, and using it as acceleration meant the firmware
was receiving a velocity value mislabeled as `lin_accel`/used for `highg`
for a while. If you ever add a new signal from this block, verify what it
actually is (e.g. by differentiating it against known IC's or another
candidate) rather than trusting a name or a guess — this model doesn't
label its own ports.

Gravity in this same world frame is `[0;0;-g]` (pulls toward -Z, matching
the "+Z is up" finding above) — used to project gravity into body frame for
`highg`. Getting this wrong doesn't just flip `highg`'s per-axis sign, it
corrupts its **magnitude** too whenever the airframe isn't sitting still or
perfectly vertical (the kinematic-accel and gravity-projection vectors stop
being parallel), and magnitude is exactly what `StateMachine`/
`BackupDeploy` threshold on (`highg_mag_g`) — see
`quatRotateWorldToBody.m`'s header.

**Getting the altitude sign wrong is a silent, nasty failure mode**, not a
crash: it doesn't break liftoff detection (that's magnitude-only), but it
inverts the sense of `COAST`'s zero-crossing check, so a real ascent-to-
descent transition reads as vert_vel going negative→positive instead of the
positive→negative the firmware is watching for. The state machine then gets
stuck in `COAST` for the rest of the flight, however long it runs, with no
error anywhere — it looks exactly like "the state machine isn't working"
rather than "the plant-side sign is backwards." If a flight visibly climbs
and comes back down but `state` never leaves `COAST`, check this first.

## Troubleshooting

- **"nothing on the dashboard" in GroundStation, even though it's
  connected**: make sure you're connected to the Teensy's *first* USB
  serial port, not `SerialUSB1`. Also make sure you reflashed after any
  firmware update — `SensorManager::update()` falls back to a safe
  "resting on the pad" default and keeps the loop (and telemetry) running
  even before `TeensyBridge` connects.
- **`Function 'serialport' not supported for code generation` when running
  the model**: `TeensyBridge` must run in interpreted execution, not the
  MATLAB System block's default code-generation mode. Double-click it
  (inside "HITL Bridge"), open its block parameters, and set "Simulate
  using" to "Interpreted execution".
- **A HITL run seems to hang, or errors immediately about a port**: check
  `TeensyBridge`'s `ComPort` (inside "HITL Bridge") actually matches the
  Teensy's *second* port right now — it can change across replugs/reflashes.
  The error message lists what's actually available if it can't find it.
- **Started a HITL run, want to go back to plain simulation**: just flip
  "HITL Mode Select" back and rerun — no other cleanup needed.
