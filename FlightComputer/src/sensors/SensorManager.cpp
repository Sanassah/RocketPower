#include "SensorManager.h"
#include "../config.h"
#include "../DebugPrint.h"
#include <Wire.h>
#include <math.h>
#include <string.h>

#ifdef HITL_MODE

bool SensorManager::_hitlReadPacket(SensorInjectPacket& out, uint32_t timeoutMs) {
    uint8_t buf[sizeof(SensorInjectPacket)];
    size_t  have = 0;
    uint32_t deadline = millis() + timeoutMs;
    while ((int32_t)(deadline - millis()) > 0) {
        if (SerialUSB1.available() == 0) continue;
        uint8_t b = (uint8_t)SerialUSB1.read();
        if (have == 0) {
            if (b != HITL_SENSOR_MAGIC_0) continue;
            buf[have++] = b;
        } else if (have == 1) {
            if (b != HITL_SENSOR_MAGIC_1) { have = 0; continue; }
            buf[have++] = b;
        } else {
            buf[have++] = b;
            if (have == sizeof(SensorInjectPacket)) {
                uint16_t expected = packetChecksum(buf, sizeof(SensorInjectPacket) - 2);
                uint16_t stored;
                memcpy(&stored, buf + sizeof(SensorInjectPacket) - 2, 2);
                if (expected == stored) {
                    memcpy(&out, buf, sizeof(SensorInjectPacket));
                    return true;
                }
                have = 0;   // bad checksum -- resync from scratch, don't get stuck out of frame
            }
        }
    }
    return false;   // timed out -- caller falls back to _lastHitlPkt
}

bool SensorManager::begin() {
    Serial.println("[SENSOR] HITL_MODE build -- sensors are SIMULATED, injected over SerialUSB1.");
    SerialUSB1.begin(115200);   // baud is ignored by Teensy's native USB CDC; kept for host-side portability

    // No real hardware to probe -- report healthy so boot doesn't halt on
    // the imu_ok/baro_ok gate below. gps/power aren't modeled by the bridge;
    // nothing flight-critical gates on those two (see SENSOR_HEALTH_TIMEOUT_MS
    // in config.h), so leaving them "unhealthy" is honest and harmless.
    _data.imu_ok = _data.baro_ok = _data.accel_ok = true;
    _data.gps_ok = _data.power_ok = false;

    // Safe default until the very first real packet arrives -- "resting on
    // the pad": identity orientation, zero rates, ~1g on highg_z (matches
    // what a real accelerometer reads sitting still). See update()/the note
    // on _lastHitlPkt in the header for why this matters: it's what keeps
    // the loop (and telemetry) running normally before a HITL bridge is
    // actually connected, instead of the whole build silently hanging on
    // the first sensors.update() call.
    _lastHitlPkt = SensorInjectPacket{};
    _lastHitlPkt.quat_w   = 1.0f;
    _lastHitlPkt.highg_z  = 1.0f;
    _hasHitlPacket = false;

    return true;
}

void SensorManager::update() {
    SensorInjectPacket pkt;
    // 20ms: comfortably longer than a real bridge round-trip (sub-ms to a
    // few ms over USB), short enough that the loop still heartbeats at a
    // reasonable rate (~50Hz) while idle/unconnected. See the header note
    // on _hitlReadPacket/_lastHitlPkt for why this can't just block forever.
    if (_hitlReadPacket(pkt, 20)) {
        _lastHitlPkt   = pkt;
        _hasHitlPacket = true;
    }
    const SensorInjectPacket& src = _lastHitlPkt;

    _data.timestamp_ms = millis();

    _data.quat_w = src.quat_w; _data.quat_x = src.quat_x;
    _data.quat_y = src.quat_y; _data.quat_z = src.quat_z;
    _data.gyro_x = src.gyro_x; _data.gyro_y = src.gyro_y; _data.gyro_z = src.gyro_z;
    _data.lin_accel_x = src.lin_accel_x;
    _data.lin_accel_y = src.lin_accel_y;
    _data.lin_accel_z = src.lin_accel_z;
    _data.accel_mag_ms2 = sqrtf(src.lin_accel_x * src.lin_accel_x +
                                 src.lin_accel_y * src.lin_accel_y +
                                 src.lin_accel_z * src.lin_accel_z);

    _data.highg_x_g = src.highg_x; _data.highg_y_g = src.highg_y; _data.highg_z_g = src.highg_z;
    _data.highg_mag_g = sqrtf(src.highg_x * src.highg_x + src.highg_y * src.highg_y + src.highg_z * src.highg_z);

    _data.baro_alt_m  = src.baro_alt_m;
    _data.vert_vel_ms = src.vert_vel_ms;   // sim ground truth -- the real build's complementary filter isn't compiled into this path at all

    // Not modeled by the HITL bridge -- zeroed, harmless (see begin()'s note
    // on gps_ok/power_ok; nothing flight-critical reads these otherwise).
    _data.pressure_hpa = 0.0f; _data.temperature_c = 0.0f;
    _data.lat = 0.0; _data.lon = 0.0; _data.gps_alt_m = 0.0f; _data.gps_sats = 0; _data.gps_fix = false;
    _data.voltage_v = 0.0f; _data.current_ma = 0.0f; _data.power_mw = 0.0f;
}

void SensorManager::calibrateBaro() {
    // No physical baro to zero -- the bridge already sends pad-relative
    // altitude directly (see Simulation/hitl/). No-op.
}

#else

bool SensorManager::begin() {
    Wire.begin();
    Wire1.begin();
    Wire2.begin();

    _data.imu_ok   = _imu.begin();
    _data.baro_ok  = _baro.begin();
    _data.accel_ok = _accel.begin();
    _data.gps_ok   = _gps.begin();
    _data.power_ok = _power.begin();

    // Must run AFTER every begin() above, not before: Adafruit_I2CDevice
    // (BMP3XX/ADXL375/BNO08x's I2C base) calls wire->begin() again inside
    // ITS OWN begin(), which resets a Teensy bus's clock back to 100kHz --
    // setting it earlier meant baro/power silently undid it with nothing to
    // reapply after. BNO085 (Wire) doesn't tolerate 400kHz on this board
    // (bench-confirmed boot failure); Wire1/Wire2 clean at 400kHz --
    // 400kHz was RULED OUT as the cause of the ADXL375 noise investigated
    // 2026-09 (reverting Wire2 to 100kHz didn't fix it either; root cause
    // was Accelerometer.cpp's torn-read bug instead, see there).
    Wire1.setClock(400000);
    Wire2.setClock(400000);

    // Seed each sensor's "last known good" clock so update() has a baseline
    // to measure staleness from. A sensor that failed begin() keeps its
    // timestamp at 0, which reads as "unhealthy" from the very first update().
    uint32_t now = millis();
    if (_data.imu_ok)   _lastImuOkMs   = now;
    if (_data.baro_ok)  _lastBaroOkMs  = now;
    if (_data.accel_ok) _lastAccelOkMs = now;
    if (_data.gps_ok)   _lastGpsOkMs   = now;
    if (_data.power_ok) _lastPowerOkMs = now;

    // IMU and barometer are critical — flight is unsafe without them
    return _data.imu_ok && _data.baro_ok;
}

void SensorManager::update() {
    uint32_t now = millis();

    // BNO085 streams 3 reports at 100Hz each (300 events/s) into a loop that
    // only runs at 100Hz -- one event/call can't keep up, FIFO backs up.
    // Drain everything queued each call, capped so a flood can't stall the
    // loop. Cost lives inside a single _imu.update() call (I2C chunked at
    // 32B/transaction, Adafruit_I2CDevice), not from calling it N times --
    // real fix was detuning linear accel off-tick in IMU.cpp so the hub
    // stops queuing all 3 reports together; this cap just bounds the drain.
    uint32_t t0 = micros();
    bool imuGotData = false;
    uint8_t imuCalls = 0;
    uint32_t imuSingleMaxUs = 0;
    for (uint8_t i = 0; i < 6; i++) {
        uint32_t cs = micros();
        bool got = _imu.update();
        uint32_t ce = micros();
        if (ce - cs > imuSingleMaxUs) imuSingleMaxUs = ce - cs;
        imuCalls++;
        if (!got) break;
        imuGotData = true;
    }
    if (imuGotData) _lastImuOkMs = now;
    uint32_t t1 = micros();
    bool baroGotData = _baro.update();
    if (baroGotData) _lastBaroOkMs = now;
    uint32_t t2 = micros();
    if (_accel.update()) _lastAccelOkMs = now;
    uint32_t t3 = micros();
    // Rate-limited -- see GPS_POLL_INTERVAL_MS (config.h) for why: a single
    // GPS I2C read costs ~7ms regardless of chunk size (u-blox DDC clock-
    // stretching), and the module only produces new data ~1Hz, so most loop
    // iterations skip this entirely rather than pay that cost for nothing.
    if (now - _lastGpsPollMs >= GPS_POLL_INTERVAL_MS) {
        _lastGpsPollMs = now;
        if (_gps.update()) _lastGpsOkMs = now;
    }
    uint32_t t4 = micros();
    if (_power.update()) _lastPowerOkMs = now;
    uint32_t t5 = micros();

    static uint32_t maxImuUs = 0, maxBaroUs = 0, maxAccelUs = 0, maxGpsUs = 0, maxPowerUs = 0;
    static uint32_t maxImuSingleUs = 0;
    static uint8_t  maxImuCalls = 0;
    if (t1-t0 > maxImuUs)   maxImuUs   = t1-t0;
    if (t2-t1 > maxBaroUs)  maxBaroUs  = t2-t1;
    if (t3-t2 > maxAccelUs) maxAccelUs = t3-t2;
    if (t4-t3 > maxGpsUs)   maxGpsUs   = t4-t3;
    if (t5-t4 > maxPowerUs) maxPowerUs = t5-t4;
    if (imuSingleMaxUs > maxImuSingleUs) maxImuSingleUs = imuSingleMaxUs;
    if (imuCalls > maxImuCalls) maxImuCalls = imuCalls;

    static uint32_t lastSensorStatMs = 0;
    if (debugPrintReady(lastSensorStatMs, 5000)) {
        // ms, 3 decimals -- full us precision, just relabeled (see main.cpp).
        DEBUG_SERIAL.print("[SENSOR BREAKDOWN] imu="); DEBUG_SERIAL.print(maxImuUs / 1000.0f, 3);
        DEBUG_SERIAL.print("ms (singleCallMax="); DEBUG_SERIAL.print(maxImuSingleUs / 1000.0f, 3);
        DEBUG_SERIAL.print("ms calls="); DEBUG_SERIAL.print(maxImuCalls);
        DEBUG_SERIAL.print(") baro="); DEBUG_SERIAL.print(maxBaroUs / 1000.0f, 3);
        DEBUG_SERIAL.print("ms accel="); DEBUG_SERIAL.print(maxAccelUs / 1000.0f, 3);
        DEBUG_SERIAL.print("ms gps="); DEBUG_SERIAL.print(maxGpsUs / 1000.0f, 3);
        DEBUG_SERIAL.print("ms power="); DEBUG_SERIAL.print(maxPowerUs / 1000.0f, 3);
        DEBUG_SERIAL.println("ms  (each is a max over the same 5s window)");
        maxImuUs = maxBaroUs = maxAccelUs = maxGpsUs = maxPowerUs = 0;
        maxImuSingleUs = 0;
        maxImuCalls = 0;
    }

    // "OK" means a successful read within the last SENSOR_HEALTH_TIMEOUT_MS,
    // not just that the sensor passed its boot check.
    _data.imu_ok   = (now - _lastImuOkMs)   < SENSOR_HEALTH_TIMEOUT_MS;
    _data.baro_ok  = (now - _lastBaroOkMs)  < SENSOR_HEALTH_TIMEOUT_MS;
    _data.accel_ok = (now - _lastAccelOkMs) < SENSOR_HEALTH_TIMEOUT_MS;
    _data.gps_ok   = (now - _lastGpsOkMs)   < SENSOR_HEALTH_TIMEOUT_MS;
    _data.power_ok = (now - _lastPowerOkMs) < SENSOR_HEALTH_TIMEOUT_MS;

    _data.timestamp_ms = now;

    // IMU
    const auto& imu = _imu.data();
    _data.quat_w       = imu.quat_w;
    _data.quat_x       = imu.quat_x;
    _data.quat_y       = imu.quat_y;
    _data.quat_z       = imu.quat_z;
    _data.lin_accel_x  = imu.lin_accel_x;
    _data.lin_accel_y  = imu.lin_accel_y;
    _data.lin_accel_z  = imu.lin_accel_z;
    _data.gyro_x       = imu.gyro_x;
    _data.gyro_y       = imu.gyro_y;
    _data.gyro_z       = imu.gyro_z;
    _data.accel_mag_ms2 = sqrtf(imu.lin_accel_x * imu.lin_accel_x +
                                 imu.lin_accel_y * imu.lin_accel_y +
                                 imu.lin_accel_z * imu.lin_accel_z);

    // Barometer
    const auto& baro = _baro.data();
    _data.pressure_hpa = baro.pressure_hpa;
    _data.temperature_c = baro.temperature_c;
    _data.baro_alt_m   = baro.rel_altitude_m;
    _data.vert_vel_ms  = baro.vert_vel_ms;

    // High-g accelerometer
    const auto& accel = _accel.data();
    _data.highg_x_g  = accel.x_g;
    _data.highg_y_g  = accel.y_g;
    _data.highg_z_g  = accel.z_g;
    _data.highg_mag_g = accel.magnitude_g;

    // GPS
    const auto& gps = _gps.data();
    _data.lat       = gps.lat;
    _data.lon       = gps.lon;
    _data.gps_alt_m = gps.alt_m;
    _data.gps_sats  = gps.sats;
    _data.gps_fix   = gps.fix;

    // Power
    const auto& pwr = _power.data();
    _data.voltage_v  = pwr.voltage_v;
    _data.current_ma = pwr.current_ma;
    _data.power_mw   = pwr.power_mw;

    // Fused vertical velocity: complementary filter, IMU integration
    // predicts every cycle, baro corrects on its own refresh (~50Hz). Falls
    // back to barometer-only (already in _data.vert_vel_ms) if IMU is down.
    // Uses Barometer::vert_vel_ms (correctly-timed there, see its update())
    // rather than re-differentiating baro_alt_m against this loop's own
    // dt -- doing that used to alias against the barometer's slower ODR and
    // amplify noise into multi-m/s spikes (bench-reported "super reactive
    // to barometer").
    //
    // Resync on IMU recovery: VERT_VEL_ALPHA's ~15s time constant (tuned to
    // reject baro noise) is longer than most of this rocket's flights -- if
    // _fusedVel_ms were left frozen through an IMU dropout, resuming with a
    // slow blend back toward reality could mean it never catches up before
    // the flight ends. Snap straight to the current baro reading instead the
    // moment IMU comes back, rather than crawling toward it.
    if (_data.imu_ok && !_imuWasOk) {
        _fusedVel_ms = baro.vert_vel_ms;
    }
    _imuWasOk = _data.imu_ok;

    if (_data.imu_ok && _prevFuseTime_ms > 0) {
        float dt = (_data.timestamp_ms - _prevFuseTime_ms) * 0.001f;
        if (dt > 0.0f && dt < 0.1f) {
            // Rotate body-frame linear acceleration to world-frame vertical (Z-up)
            // using the BNO085 quaternion. Row 3 of the rotation matrix:
            float qw = _data.quat_w, qx = _data.quat_x;
            float qy = _data.quat_y, qz = _data.quat_z;
            float accel_vert = 2.0f*(qx*qz - qw*qy) * _data.lin_accel_x
                             + 2.0f*(qy*qz + qw*qx) * _data.lin_accel_y
                             + (1.0f - 2.0f*(qx*qx + qy*qy)) * _data.lin_accel_z;

            _fusedVel_ms += accel_vert * dt;   // predict every cycle (100Hz)
            // Correct only when baro actually refreshed (~50Hz) AND the
            // sample is physically plausible -- see MAX_PLAUSIBLE_VERT_SPEED_MS.
            if (baroGotData && fabsf(baro.vert_vel_ms) <= MAX_PLAUSIBLE_VERT_SPEED_MS) {
                _fusedVel_ms = VERT_VEL_ALPHA * _fusedVel_ms
                             + (1.0f - VERT_VEL_ALPHA) * baro.vert_vel_ms;
            }
            _data.vert_vel_ms = _fusedVel_ms;

            // Bench diagnostic -- isolates IMU-integration (accelStep) from
            // the baro correction (baroVel, only meaningful if baroFresh=1)
            // so it's visible which one drives a given swing in fusedVel.
            {
                static uint32_t lastPrintMs = 0;
                if (debugPrintReady(lastPrintMs)) {
                    DEBUG_SERIAL.print("[VELFUSE] dt="); DEBUG_SERIAL.print(dt, 4);
                    DEBUG_SERIAL.print(" accelVert="); DEBUG_SERIAL.print(accel_vert, 3);
                    DEBUG_SERIAL.print(" accelStep="); DEBUG_SERIAL.print(accel_vert * dt, 4);
                    DEBUG_SERIAL.print(" baroFresh="); DEBUG_SERIAL.print(baroGotData ? 1 : 0);
                    DEBUG_SERIAL.print(" baroVel="); DEBUG_SERIAL.print(baro.vert_vel_ms, 3);
                    DEBUG_SERIAL.print(" baroAlt="); DEBUG_SERIAL.print(_data.baro_alt_m, 3);
                    DEBUG_SERIAL.print(" fusedVel="); DEBUG_SERIAL.println(_fusedVel_ms, 3);
                }
            }
        }
    }
    _prevFuseTime_ms = _data.timestamp_ms;
}

void SensorManager::calibrateBaro() {
    _baro.calibrate();
    _fusedVel_ms     = 0.0f;
    _prevFuseTime_ms = 0;     // forces a one-cycle skip so no spurious velocity spike
}

#endif // HITL_MODE
