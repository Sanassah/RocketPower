#include "AttitudeController.h"
#include "../config.h"
#include <math.h>

float AttitudeController::_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void AttitudeController::reset() {
    _controlEnabled = false;
    _demoEnabled    = false;
    _wasEngaged     = false;
    _refQuatW = 1.0f; _refQuatX = 0.0f; _refQuatY = 0.0f; _refQuatZ = 0.0f;
    _rollIntegral   = 0.0f;
    _pitchIntegral  = 0.0f;
    _yawIntegral    = 0.0f;
    _prevUpdateMs   = 0;
}

void AttitudeController::update(const FlightData& d, FinController& fins) {
    if (!_controlEnabled && !_demoEnabled) {
        _wasEngaged = false;   // next real engage re-latches fresh, doesn't reuse a stale reference
        return;
    }

    // Rising edge: this is the first loop where the controller is actually
    // running (main.cpp only calls update() once its flight-state gate is
    // open too -- POWERED_ASCENT/COAST for real control, ARMED for demo).
    // Latch whatever orientation it's in RIGHT NOW as the reference to hold.
    // For real flight that lands exactly on liftoff; for demo it lands on
    // whichever happens later, arming or hitting Enable.
    if (!_wasEngaged) {
        _refQuatW = d.quat_w; _refQuatX = d.quat_x;
        _refQuatY = d.quat_y; _refQuatZ = d.quat_z;
        _rollIntegral = _pitchIntegral = _yawIntegral = 0.0f;
        _prevUpdateMs = 0;
        _wasEngaged = true;
    }

    uint32_t now = millis();
    float dt = (_prevUpdateMs > 0) ? (now - _prevUpdateMs) * 0.001f : 0.0f;
    _prevUpdateMs = now;

    // ---- Rate term (damping) ----
    float rollRate  = ATTITUDE_ROLL_RATE(d);
    float pitchRate = ATTITUDE_PITCH_RATE(d);
    float yawRate   = ATTITUDE_YAW_RATE(d);

    // Deadband -- ignore sensor noise near zero so a genuinely stable
    // rocket doesn't dither the servos chasing it around.
    if (fabsf(rollRate)  < ATTITUDE_RATE_DEADBAND_RADS) rollRate  = 0.0f;
    if (fabsf(pitchRate) < ATTITUDE_RATE_DEADBAND_RADS) pitchRate = 0.0f;
    if (fabsf(yawRate)   < ATTITUDE_RATE_DEADBAND_RADS) yawRate   = 0.0f;

    // Integral accumulation -- inert while the *_RATE_KI gains are 0 (the
    // recommended starting point). Skipped on the first call after a fresh
    // latch (dt==0) and on any implausibly large gap so a stale accumulator
    // can't suddenly dump a huge correction the instant control starts.
    if (dt > 0.0f && dt < 0.1f) {
        _rollIntegral  += rollRate  * dt;
        _pitchIntegral += pitchRate * dt;
        _yawIntegral   += yawRate   * dt;
    }

    // ---- Angle term (position -- pulls back toward the latched reference) ----
    // Standard small-angle quaternion-error feedback: qe = conj(ref) (x) cur.
    // For small-to-moderate angular displacement, 2 * (qe.x, qe.y, qe.z) is
    // the body-frame rotation FROM ref TO cur, in radians -- exactly the
    // "how far off and which way" the angle term needs to null out. Same
    // sign convention as the rate term: correction = -(KP * error).
    float rw = _refQuatW, rx = _refQuatX, ry = _refQuatY, rz = _refQuatZ;
    float qw = d.quat_w,  qx = d.quat_x,  qy = d.quat_y,  qz = d.quat_z;
    float ew =  rw*qw + rx*qx + ry*qy + rz*qz;
    float ex =  rw*qx - rx*qw - ry*qz + rz*qy;
    float ey =  rw*qy + rx*qz - ry*qw - rz*qx;
    float ez =  rw*qz - rx*qy + ry*qx - rz*qw;
    if (ew < 0.0f) { ex = -ex; ey = -ey; ez = -ez; }   // shortest-path representation

    float rollAngleErr  = 2.0f * ATTITUDE_ROLL_ANGLE_ERR(ex, ey, ez);
    float pitchAngleErr = 2.0f * ATTITUDE_PITCH_ANGLE_ERR(ex, ey, ez);
    float yawAngleErr   = 2.0f * ATTITUDE_YAW_ANGLE_ERR(ex, ey, ez);

    // ---- Combine: PD per axis (angle = position, rate = damping) ----
    float rollCmd  = -(ATTITUDE_ROLL_ANGLE_KP  * rollAngleErr  + ATTITUDE_ROLL_RATE_KP  * rollRate  + ATTITUDE_ROLL_RATE_KI  * _rollIntegral);
    float pitchCmd = -(ATTITUDE_PITCH_ANGLE_KP * pitchAngleErr + ATTITUDE_PITCH_RATE_KP * pitchRate + ATTITUDE_PITCH_RATE_KI * _pitchIntegral);
    float yawCmd   = -(ATTITUDE_YAW_ANGLE_KP   * yawAngleErr   + ATTITUDE_YAW_RATE_KP   * yawRate   + ATTITUDE_YAW_RATE_KI   * _yawIntegral);

    rollCmd  = _clampf(rollCmd,  -ATTITUDE_MAX_AXIS_DEG, ATTITUDE_MAX_AXIS_DEG);
    pitchCmd = _clampf(pitchCmd, -ATTITUDE_MAX_AXIS_DEG, ATTITUDE_MAX_AXIS_DEG);
    yawCmd   = _clampf(yawCmd,   -ATTITUDE_MAX_AXIS_DEG, ATTITUDE_MAX_AXIS_DEG);

    // Allocation across the 4 fins -- CH1=S, CH2=E, CH3=N, CH4=W. That
    // channel-to-compass mapping itself is bench-confirmed (testSweep per
    // channel, watched against the physical airframe). The +/- SIGNS below
    // (which direction of roll/pitch differential is "positive") are not --
    // still exactly as unverified as the axis mappings in config.h, and need
    // the same bench/demo-mode check before this is ever trusted.
    //
    // yawCmd (spin, in this file's Simulation-matching convention -- see
    // config.h's axis-naming note) is uniform across all 4 fins; rollCmd/
    // pitchCmd are the two transverse-tilt differentials (N/S, E/W).
    fins.setCorrectionDeg(1, yawCmd - rollCmd);   // S
    fins.setCorrectionDeg(2, yawCmd - pitchCmd);  // E
    fins.setCorrectionDeg(3, yawCmd + rollCmd);   // N
    fins.setCorrectionDeg(4, yawCmd + pitchCmd);  // W
}
