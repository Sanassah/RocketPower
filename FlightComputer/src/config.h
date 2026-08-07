#pragma once

// =============================================================
// config.h — All pin definitions and flight constants
// Derived from KiCad schematics (main.kicad_sch and sub-sheets)
// Board: Custom RocketPower PCB (NXP MIMXRT1062), programmed via the
// Teensy 4.1 toolchain/bootloader -- not a Teensy dev board.
// =============================================================

// ===== I2C Bus Assignments =====
// Wire  (LPI2C1, pins 18 SDA / 19 SCL): sensors with no bus-number suffix
// Wire1 (LPI2C3, pins 17 SDA / 16 SCL): sensors labelled _SDA1 / _SCL1
// Wire2 (LPI2C4, pins 25 SDA / 24 SCL): sensors labelled _SDA2 / _SCL2

#define BNO085_I2C_BUS   Wire
#define BNO085_I2C_ADDR  0x4A   // SA0 pulled LOW (BNO085_SCL/SDA, no suffix)

#define BMP390_I2C_BUS   Wire1
#define BMP390_I2C_ADDR  0x77   // SDO pulled HIGH (BMP390_SCL1/SDA1)

#define ZOEM8_I2C_BUS    Wire1
#define ZOEM8_I2C_ADDR   0x42   // fixed u-blox DDC address (ZOEM8_SCL1/SDA1)

// Echo raw NMEA sentences to Serial as they're read (verbose — GPS bring-up only)
#define GPS_DEBUG_RAW_NMEA 0

#define INA260_I2C_BUS   Wire1
#define INA260_I2C_ADDR  0x40   // A0=A1=GND (INA260_SCL1/SDA1)

#define ADXL375_I2C_BUS  Wire2
#define ADXL375_I2C_ADDR 0x1D   // CS tied HIGH, ADDR tied HIGH (ADXL375_SCL2/SDA2)

// ===== UART Assignments =====
// From schematic text annotations: "0 (RX1)", "1 (TX1)", "RX 28", "TX 29"
#define LORA_SERIAL      Serial1   // Lora_TX→pin0 (RX1), Lora_RX→pin1 (TX1)
#define LORA_BAUD        9600

#define CAM_SERIAL       Serial7   // Cam_TX→pin28 (RX7), Cam_RX→pin29 (TX7)
#define CAM_BAUD         115200

// ===== LoRa E22-900T22D Control =====
// M0 and M1 are NOT connected to MCU on this board (4-pin JST-GH connector).
// Module is hardwired in Mode 0 (transparent UART) — M0=GND, M1=GND on PCB.
#define LORA_M0_PIN      -1   // not connected
#define LORA_M1_PIN      -1   // not connected
#define LORA_AUX_PIN     -1   // not connected
#define LORA_CHANNEL     0    // 900.125 MHz + channel * 1 MHz

// ===== SD Card =====
// MicroSD via SDIO (Hirose DM3D-SF, SD_B0 bus)
#define SD_CS_PIN        BUILTIN_SDCARD
// There's no hardware card-detect pin on this socket, so presence can only
// be inferred by actually attempting SD.begin()/a write -- see DataLogger.
// Deliberately NOT polled on a timer from the main loop: SD.begin() isn't
// cheap on real hardware (worse yet with no card to answer), and doing that
// every couple seconds from the same real-time loop that also has to hit
// its telemetry timing was a plausible cause of intermittent telemetry
// stalls seen on the bench.

// ===== Pyro Channels =====
// From HighCurrentComponents.kicad_sch hierarchical labels
// PyroCHx_N  → low-side MOSFET gate, MCU pin N (active HIGH to fire)
// Pyrox_Test_N → continuity sense (analog input), MCU pin N
#define PYRO_CH1_FIRE_PIN   2     // PyroCH1_2  — ignition
#define PYRO_CH2_FIRE_PIN   3     // PyroCH2_3  — parachute
#define PYRO_CH3_FIRE_PIN   4     // PyroCH3_4  — backup

#define PYRO_CH1_CONT_PIN   40    // Pyro1_Test_40
#define PYRO_CH2_CONT_PIN   41    // Pyro2_Test_41
#define PYRO_CH3_CONT_PIN   39    // Pyro3_Test_39

#define PYRO_NUM_CHANNELS   3
#define PYRO_FIRE_DURATION_MS  500     // pulse width when firing
#define PYRO_CONT_THRESHOLD    512     // ADC counts — above = continuity OK

// ===== Pyro channel aliases =====
#define PYRO_IGNITION   1   // CH1 — motor igniter, fired by ground command
#define PYRO_PARACHUTE  2   // CH2 — recovery parachute, fired at apogee
#define PYRO_BACKUP     3   // CH3 — backup charge, fired manually if CH2 fails

// ===== Camera =====
// JST-GH 4-pin, Serial7 at 115200 baud (RunCam Split 4-25 compatible)

// ===== Fin actuation servos =====
// From HighCurrentComponents.kicad_sch hierarchical labels Servo1_PWM..Servo4_PWM.
// Closed-loop attitude control exists (see AttitudeController) but is OFF by
// default -- see ATTITUDE_CONTROL_ENABLED below. Bench-test-only functions
// (testSweep, preflightSequence, trim nudge/save) are unaffected either way.
#define SERVO1_PIN   36
#define SERVO2_PIN   8
#define SERVO3_PIN   7
#define SERVO4_PIN   9
#define SERVO_NUM_CHANNELS 4

// Blue Bird BMS-101HV (6.0-8.4V digital HV micro servo) datasheet: neutral
// 1520us, rated travel 1100-1900us (90 deg total, ~8.9us/deg). We deliberately
// command a much smaller window than the full rated range for now.
#define SERVO_CENTER_US     1520   // neutral (datasheet value, not 1500)
#define SERVO_MAX_ANGLE_DEG 25     // max deflection each side of center (rated travel is +-45deg)
#define SERVO_MIN_US        (SERVO_CENTER_US - (800L * SERVO_MAX_ANGLE_DEG) / 90)
#define SERVO_MAX_US        (SERVO_CENTER_US + (800L * SERVO_MAX_ANGLE_DEG) / 90)
#define SERVO_TEST_STEP_MS 500    // dwell time at each position during a test sweep

// Per-channel trim (offset from SERVO_CENTER_US) is set by nudging against a
// printed alignment jig and saved to flash -- see FinController.
#define SERVO_TRIM_STEP_US    1      // pulse-width change per nudge -- 1us is the finest step our
                                      // integer-microsecond tracking can express (~0.11 deg on the
                                      // BMS-101HV's 800us/90deg scale)

// Gap between commanding each channel's very first position at boot, so the 4
// servos don't all snap to center in the same instant -- spreads out inrush
// current on the shared, unregulated servo rail instead of stacking it.
// 60ms wasn't enough to actually separate the events (a full board power
// cycle still reproduced the failure); 400ms is closer to the real-world
// separation a manual one-at-a-time reconnect gives each servo, which does
// recover it.
#define SERVO_INIT_STAGGER_MS 400

#define SERVO_CAL_EEPROM_ADDR 0
#define SERVO_CAL_MAGIC       0xC1   // bump if FinCalibration's layout ever changes

// ===== BENCH DIAGNOSTIC =====
// Set to 1-4 to leave that one channel unattached, isolating the other three
// for one-at-a-time testing. 0 = attach all 4 normally.
#define FIN_TEST_SKIP_CHANNEL 0

// ===== Active attitude control (fin-based PD attitude hold) =====
// See AttitudeController.h for the full design writeup. Short version: a PD
// controller per axis with two independent terms --
//   ANGLE (position): how far the current orientation has drifted from a
//     REFERENCE orientation, latched the instant control/demo starts
//     actually running (see AttitudeController::update()). Drives that
//     drift back toward zero -- this is what makes it hold an attitude
//     instead of just resisting motion.
//   RATE (damping): sensed angular rate, same as before -- keeps the angle
//     term from overshooting/oscillating on the way back.
// A wrong RATE gain only ever under/over-damps. A wrong ANGLE gain is a
// different, higher-stakes failure mode: it can actively steer AWAY from
// the reference instead of toward it. Both need the same bench-verification
// discipline below, and the angle term needs it more.
//
// Whether it's actually ACTIVE is a runtime, ground-commanded thing now
// (ATTITUDE_CONTROL_ENABLE/DISABLE and ATTITUDE_DEMO_ENABLE/DISABLE command
// packets, see Packet.h + TelemetryManager) -- not a flag here. That mirrors
// how ARM/DISARM and pyro fire already work in this codebase: the ground
// station commands the mode, the firmware's state-machine gating (see
// main.cpp step 2b) is what actually keeps it safe, not requiring a reflash
// to change your mind. Both runtime flags default OFF on every boot and get
// forced back OFF on DISARM, so nothing carries over between sessions.
//
// Before ever sending ATTITUDE_CONTROL_ENABLE for a real flight:
//   1. ATTITUDE_*_RATE and ATTITUDE_*_ANGLE_ERR below are verified on the
//      bench (see the comments there -- this is the single most important
//      thing to get right; a flipped sign REINFORCES the error instead of
//      opposing it, which is worse than no control at all).
//   2. The gains below have been validated in simulation (closed-loop --
//      see the HIL discussion; open-loop replay can't test this, there's
//      nothing to close the loop against a fin command with).
//   3. You're comfortable with a bench functional test via
//      ATTITUDE_DEMO_ENABLE: hand-tilt the armed (but not flying) airframe
//      away from whatever orientation it was in when you hit Enable, HOLD
//      it there, and confirm the fins hold a deflection that opposes the
//      tilt (not just react while you're actively moving it).

// Which raw gyro_x/y/z channel corresponds to which physical rotation axis.
// UNVERIFIED -- there is no documented BNO085-mounting-orientation
// convention anywhere in this codebase (checked). The assignment below is
// only a starting guess (roll = gyro_z, matching the ground station's own
// rendering convention, which is itself not verified against the real PCB).
// To find the real mapping: rotate the airframe by hand about its actual
// long axis (roll) and watch which of gyro_x/gyro_y/gyro_z shows the large
// signal in Serial/telemetry -- that's your real roll axis. Repeat about
// the other two physical axes for pitch/yaw. Update these three lines
// (swap which field each reads, and negate if the sign opposes what you
// observed) -- nothing else in AttitudeController needs to change.
#define ATTITUDE_ROLL_RATE(d)   ((d).gyro_z)
#define ATTITUDE_PITCH_RATE(d)  ((d).gyro_x)
#define ATTITUDE_YAW_RATE(d)    ((d).gyro_y)

// Same idea, but for the ANGLE (position) term: which component of the
// quaternion-error vector (ex, ey, ez -- see AttitudeController.cpp for how
// that's computed from the current vs. latched-reference orientation) maps
// to which physical rotation axis. These MIRROR the gyro mapping above on
// purpose -- the quaternion and the raw gyro come from the same physical
// sensor/frame, so whatever channel/sign you find correct for rate here
// applies unchanged to angle, and vice versa. Verify once via the rate
// mapping above (or this one, either order), then copy the same
// channel/sign choice to both.
#define ATTITUDE_ROLL_ANGLE_ERR(ex, ey, ez)   (ez)
#define ATTITUDE_PITCH_ANGLE_ERR(ex, ey, ez)  (ex)
#define ATTITUDE_YAW_ANGLE_ERR(ex, ey, ez)    (ey)

// Angle (position) gain: fin correction (deg) per radian of drift away from
// the latched reference orientation. This is the term that makes the
// controller actually HOLD an attitude instead of just resisting motion --
// see the class comment in AttitudeController.h for why a wrong sign here
// is worse than a wrong rate-gain sign (it can actively steer away from the
// reference, not just fail to help). Starting at a small nonzero value (not
// 0) specifically so demo mode is usable for its actual purpose -- checking
// axis mapping/allocation signs by hand-tilting the airframe and watching
// whether the fins pull it back or push it further. Provisional bench/demo
// value, NOT simulation-validated -- see the checklist above before ever
// using this for a real flight.
#define ATTITUDE_ROLL_ANGLE_KP   10.0f
#define ATTITUDE_PITCH_ANGLE_KP  10.0f
#define ATTITUDE_YAW_ANGLE_KP    10.0f

// Rate (damping) gain: fin correction (deg) per (rad/s) of sensed rate on
// that axis. Keeps the angle term above from overshooting/oscillating on
// the way back to the reference -- the PD controller's "D" term, in effect.
// Independent per axis since fin authority and the rocket's moment of
// inertia aren't the same for roll vs. pitch/yaw. Same "provisional demo
// value" caveat as the angle gains above.
#define ATTITUDE_ROLL_RATE_KP    4.0f
#define ATTITUDE_PITCH_RATE_KP   4.0f
#define ATTITUDE_YAW_RATE_KP     4.0f

// Optional integral gains on the RATE term -- 0 = pure-PD (recommended
// starting point, and the only mode that's been reasoned about above). An
// integral term here can null out a steady rate bias but risks winding up
// over a short, dynamic powered-ascent phase. Only consider enabling these
// after PD-only behavior is validated.
#define ATTITUDE_ROLL_RATE_KI    0.0f
#define ATTITUDE_PITCH_RATE_KI   0.0f
#define ATTITUDE_YAW_RATE_KI     0.0f

// Below this sensed rate (rad/s), treat it as sensor noise and command
// zero correction -- without a deadband a "stable" rocket still dithers
// the servos constantly chasing gyro noise around zero.
#define ATTITUDE_RATE_DEADBAND_RADS  0.02f

// Per-axis correction authority limit (deg), clamped BEFORE allocating
// across the 4 fins -- keeps one noisy or saturated axis from eating the
// whole servo range at the expense of the other two. The per-fin total
// (after allocation) is still hard-clamped again by FinController to
// SERVO_MIN_US/SERVO_MAX_US regardless, as a second, independent backstop.
#define ATTITUDE_MAX_AXIS_DEG   10.0f

// ===== Flight Constants =====
#define LIFTOFF_ACCEL_THRESHOLD    2.5f   // g — triggers POWERED_ASCENT
#define LIFTOFF_CONFIRM_MS         100    // must hold for this long
#define BURNOUT_ACCEL_THRESHOLD    1.0f   // g — drop below = COAST
#define APOGEE_DETECTION_WINDOW_MS 200    // vertical-velocity zero-crossing window
#define LANDED_STABLE_MS           10000  // altitude stable for this long = LANDED
#define LANDED_ALT_TOLERANCE_M     2.0f   // ±m to count as "stable"

// Requesting 200ms (5Hz) outran what the LoRa link could actually clear over
// the air, leaving the radio almost continuously transmitting -- observed
// real throughput was 0.8-2.3Hz, and since the link is half-duplex, a nearly
// always-busy radio also had little time free to receive inbound commands.
// 400ms still wasn't enough idle/listening time for reliable command
// reception. Confirmed empirically with the isolated TelemetryTest rig
// (dual-comms build): 1000ms (1Hz) gave a clean, reliable command hit rate
// alongside continuous telemetry. Slower downlink, but a link that actually
// works both ways beats a faster one that doesn't.
#define TELEMETRY_INTERVAL_MS      1000   // 1 Hz downlink (was 400ms/2.5Hz)

// The raw binary packet is also written to USB Serial on every send/ack (see
// LoRa.cpp) so the ground station can read telemetry over a direct USB link
// with no radio involved. That's unreadable noise -- garbled control
// characters -- in a plain text serial monitor. Set to 0 while watching the
// USB serial monitor for human debugging (e.g. command RX/dedup behavior);
// set back to 1 only when the ground station actually needs to connect over
// USB instead of the LoRa link.
#define USB_SERIAL_BINARY_MIRROR   1

// USB telemetry mirror rate, independent of TELEMETRY_INTERVAL_MS above.
// The 1Hz LoRa rate is a hard airtime limit (see the comment on that
// constant) -- a direct USB link has no such ceiling, so throttling it to
// match the radio would just be leaving USB performance on the table for no
// reason. 50ms/20Hz is comfortably inside what Teensy's native USB serial
// can move (a 51-byte packet is nothing) and well past what's visually
// distinguishable on the ground station's attitude display; SensorManager
// itself only refreshes at the ~100Hz loop rate, so this is not the
// bottleneck. Lower it further if useful, there's plenty of headroom.
#define USB_TELEMETRY_INTERVAL_MS  50     // 20 Hz
#define LOG_INTERVAL_MS            10     // 100 Hz SD logging

// A sensor is reported "down" in telemetry/logging if it hasn't produced a
// successful reading within this window. Loose enough to absorb a sensor's
// normal per-loop miss rate (e.g. BNO085 events interleave across report
// types, so not every 10ms loop tick gets one) without flickering healthy
// sensors, tight enough to flag a real mid-flight dropout within half a
// second. Purely informational -- nothing in flight-critical control logic
// (state transitions, pyro firing) gates on these flags.
#define SENSOR_HEALTH_TIMEOUT_MS   500

// Complementary filter weight for fused vertical velocity (SensorManager).
// 0.98 = IMU integration dominates above ~0.5 Hz, baro corrects drift below that.
#define VERT_VEL_ALPHA  0.98f

// ===== Sea-level pressure for altitude reference =====
#define SEA_LEVEL_HPA 1013.25f

// ===== Status LEDs =====
// From main.kicad_sch: D3 "GPS LED", D4 "Altimeter LED", D5 "200G ACC LED",
// D6 "IMU LED" each have their cathode tied to GND and their anode driven
// through a series resistor by a dedicated MIMXRT1062 GPIO (active HIGH).
// Pin numbers below were cross-checked against the Teensy 4.1 core's
// CORE_PINn_CONFIG -> IOMUXC pad table (core_pins.h), since this board wires
// the bare MIMXRT1062 the same way the real Teensy 4.1 module does.
// D1 "3.3V LED" is a hardwired power-good indicator (not GPIO-driven).
// D2 "BOOT LED" is driven by the separate MKL02 USB/bootloader chip, not the
// flight MCU -- no software control possible from this firmware.
#define IMU_LED_PIN     26   // D6 "IMU LED"       -- GPIO_AD_B1_14
#define BARO_LED_PIN    5    // D4 "Altimeter LED" -- GPIO_EMC_08
#define ACCEL_LED_PIN   13   // D5 "200G ACC LED"  -- GPIO_B0_03 (shares Teensy's LED_BUILTIN pad)
#define GPS_LED_PIN     27   // D3 "GPS LED"        -- GPIO_AD_B1_15

#define STATUS_LED_FLASH_MS  250   // half-period for fault flashing (~2 Hz)
