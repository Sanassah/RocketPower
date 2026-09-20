%% ============================================================
%  ROCKET CONSTANTS - fin-actuated stabilization model
%
%  Convention: "drone" body frame -> z = longitudinal axis (+z = nose)
%              x,y = transverse tilt axes | spin about z = your "yaw"
%  Units:      control loop in DEGREES (matches firmware)
%              aero coefficients per RADIAN (D2R gain at each moment
%              chain entry does the conversion)
%  Legend:     [MEASURED] from datasheet/ruler  [COMPUTED] derived here
%              [OPENROCKET] export from your .ork  [BARROWMAN] hand calc
%              [TUNE] adjusted in sim  [PLACEHOLDER] dummy - replace!
%% ============================================================

%% --- UNIT CONVERSIONS ---
D2R         = pi/180;       % deg -> rad, entry of each fin moment chain
R2D         = 180/pi;       % rad -> deg, on wx/wy/wz (6DOF rates -> Kd inputs!)
K_quat2deg  = 360/pi;       % quat vector component -> error angle in deg
                            % (= 2 * 180/pi: undo half-angle, rad to deg)

%% --- GEOMETRY [MEASURED - see CID_RocketPower 13.3] ---
d           = 0.066;        % m  - Body diameter
r           = d/2;          % m  - Body radius
L_rocket    = 0.75;         % m  - Total length
A_ref       = pi*r^2;       % m2 - Reference area (cross section) = 0.003421 m2
                            % All aero coefficients below reference THIS area.

%% --- MASS PROPERTIES [MEASURED - see CID_RocketPower 13.3] ---
m_total     = 0.743;        % kg - Total mass at launch (loaded, live motor installed)
m_prop      = 0.055;        % kg - Propellant mass (G38 datasheet) [UNVERIFIED vs
                            %     this rocket's own motor -- CID 13.3/13.5 still
                            %     lists propellant mass as TODO pending the actual
                            %     spec sheet. 743g loaded - 656g no-motor = 87g
                            %     total motor mass; 87-55=32g implied casing is
                            %     plausible for a G-class motor, but not confirmed
                            %     -- this is this file's own number, not newly
                            %     supplied. Re-check before trusting mdot/t_burn.
m_dry       = m_total - m_prop; % kg - Mass at burnout = 0.688. NOTE: despite the
                            %     name, this is BURNOUT mass (spent casing still
                            %     onboard, only propellant subtracted), NOT the
                            %     656g no-motor-at-all bench reference -- see CID
                            %     13.4. Name kept as-is since Simulink blocks
                            %     reference it; don't rename without updating the
                            %     .slx too.

%% --- INERTIA [Ixx/Iyy MEASURED - bifilar pendulum, loaded; Izz still PLACEHOLDER] ---
% Drone convention: z = longitudinal => Izz is the SMALL spin inertia.
Ixx         = 0.0338;       % kg.m2 - Transverse (tilt about x) [MEASURED - bifilar
                            %     pendulum, loaded config, see CID 13.2.2/13.3]
Iyy         = 0.0338;       % kg.m2 - Transverse (tilt about y) [MEASURED - same
                            %     bifilar pendulum result, symmetric assumption]
Izz         = 0.001;        % kg.m2 - Spin about the long axis [PLACEHOLDER -
                            %     roll-axis inertia not yet measured, CID 13.5.
                            %     Order-of-magnitude sanity check: modeling the
                            %     body alone as a cylinder (m=0.743, r=0.033)
                            %     gives 0.0004 (solid) to 0.00081 (thin shell)
                            %     kg.m2; fins sit outside that radius and add a
                            %     bit more despite their small mass. 0.001 sits
                            %     just above the body-only band -- reasoned
                            %     placeholder, not a substitute for the real
                            %     nose-up bifilar measurement.

%% --- MASS CENTER & AERO CENTERS (all measured from NOSE TIP, along z) ---
CG_z        = 0.390;        % m - [MEASURED - balance test, loaded, CID 13.3]
                            %     Center of gravity: the pivot. Every arm is
                            %     measured FROM here.
CP_rocket_z = 0.478;        % m - [OPENROCKET] WHOLE-ROCKET center of pressure,
                            %     Barrowman method, CID 13.3. CAUTION: CID's own
                            %     CAD flat-plate CP estimate (0.361m) disagrees
                            %     with this by ~11.7cm and implies CP FORWARD of
                            %     CG (unstable) -- unresolved, see CID 13.5. This
                            %     Simulink file trusts the OpenRocket value.
                            %     Where total aero force acts at angle of attack,
                            %     weighted average of all components' CPs. Used
                            %     by the PASSIVE aero subsystem only.
CP_fin_z    = 0.696;         % m - [MEASURED] FIN center of pressure: where the
                            %     DEFLECTION force acts. One of the ingredients
                            %     inside CP_rocket_z, isolated here because fin
                            %     deflection adds force only there. Derived from
                            %     a direct measurement of 0.306 m between CG and
                            %     fin CP (CG_z + 0.306); close to the old
                            %     Barrowman-offset/placeholder guess of 0.70,
                            %     off by only 4mm -- good cross-check.
CG          = [0; 0; CG_z]; % m - vector forms, if blocks want them
CP          = [0; 0; CP_rocket_z];
SM          = (CP_rocket_z - CG_z)/d; % calibers - [COMPUTED] static margin, want >1

%% --- PASSIVE AERODYNAMICS (weathercock subsystem - NOT the fin chains) ---
Cn_alpha    = 8.0;          % /rad - [RECONCILED] whole-rocket normal force
                            %     slope, ref A_ref. Was 2.0 -- flagged in its own
                            %     comment as looking like a nose-alone value; a
                            %     finned rocket is typically 8-12 here. Matched
                            %     to the Dynamic Drag coeff fcn's own CNa, which
                            %     was already a reasonable placeholder in that
                            %     range -- one number now, not two disagreeing
                            %     guesses. Still not a real OpenRocket/wind-
                            %     tunnel value; pull one if you want better.
Cmq         = -8.0;         % /rad - [PLACEHOLDER] pitch damping derivative,
                            %     ref A_ref and L_ref (add when passive
                            %     subsystem is built; keeps sim from ringing)
Cd          = 0.5;          % -    - [OPENROCKET] drag coefficient
L_ref       = d;            % m    - reference length for moment coefficients
L_arm_passive = CP_rocket_z - CG_z; % m - [COMPUTED] rocket CP to CG = 0.088.
                            %     PASSIVE subsystem arm ONLY. Never in fin chains.

%% --- FIN GEOMETRY [OPENROCKET - Trapezoidal Fin Set export] ---
n_fins      = 4;            % -   - Number of fins
fin_span    = 0.043;        % m   - Semispan, root to tip (OpenRocket "Height")
fin_root    = 0.074;        % m   - Root chord
fin_tip     = 0.0286;       % m   - Tip chord
fin_sweep   = 0.0461;       % m   - Sweep length, root LE to tip LE (0 cant)
fin_area    = 0.5*(fin_root+fin_tip)*fin_span; % m2 - [COMPUTED] trapezoid
                            %     panel area = 0.002206 (bookkeeping/hinge check;
                            %     the moment chains use A_ref, not this)

%% --- FIN CONTROL AERODYNAMICS (the three moment chains) ---
Cn_delta    = 15;           % /rad - [BARROWMAN/PLACEHOLDER] deflecting fin PAIR
                            %     effectiveness, ref A_ref (body cross-section
                            %     convention - must stay paired with A_ref!).
                            %     NOT the same thing as Cn_alpha.
Cl_delta    = 0.1;          % /rad - [PLACEHOLDER] differential-deflection spin
                            %     coefficient for the z channel, ref A_ref
% Moment arms - names match the gain blocks in your Simulink diagram:
L_arm_roll  = CP_fin_z - CG_z;    % m - x row: fin CP to CG = 0.306 [MEASURED,
                            %     see CP_fin_z above].
L_arm_pitch = L_arm_roll;         % m - y row: same physics, same variable
L_arm_yaw   = r + 0.45*fin_span;  % m - z row (spin): RADIAL arm = 0.0524
                            %     body radius + spanwise CP of the panel.
                            %     Constant through flight (pure geometry),
                            %     unlike the axial arms which grow as CG
                            %     moves forward during burn.

%% --- CONTROL LIMITS (degrees - the loop convention) ---
delta_max_deg = 15;         % deg - software clamp in controller (fin stall margin)
delta_max   = delta_max_deg*D2R;  % rad - same value for any rad-side use
delta_min   = -delta_max;   % rad
I_MAX       = 5;            % deg - [TUNE] hard clamp on integrator state
                            %     (must equal the firmware's clamp)

%% --- SERVO: BlueBird BMS-101HV @ 7.4V [MEASURED from datasheet] ---
Ts_servo    = 0.003;        % s     - frame period (333 Hz) -> Zero-Order Hold
slew_servo  = 1000;         % deg/s - 0.06 s/60deg -> Rate Limiter +/- value
tau_servo   = 0.015;        % s     - [PLACEHOLDER] small-signal lag, 1/(tau*s+1).
                            %     Estimate; measure with the slow-mo step test.
db_servo    = 0.1125;       % deg   - dead zone half-width (2us * 0.1125 deg/us)
travel_servo= 45;           % deg   - physical travel backstop +/-
pwm_step    = 0.1125;       % deg   - 1us PWM resolution (optional Quantizer)
T_stall     = 1.1*0.0980665;% N.m   - stall torque = 0.108 (hinge-moment check)

%% --- CONTROLLER TIMING ---
Ts_ctrl     = 0.003;        % s - one PID iteration per servo frame,
                            %     matching firmware pacing (Atomic Subsystem)

%% --- ATMOSPHERE (sea level standard) ---
rho         = 1.225;        % kg/m3
g           = 9.81;         % m/s2
v_sound     = 343;          % m/s

%% --- MOTOR (AeroTech G38) ---
% T_max and I_total used to live here too, but nothing ever read them (not
% the model, not even the sanity prints below) and they didn't match the
% G38 Motor Thrust lookup block's own embedded curve (peak 52.9N, impulse
% 86.8 N.s vs the datasheet-nameplate 78.2N/87.7N here) -- removed as stale,
% redundant duplicates of data the lookup table already encodes correctly.
T_avg       = 32.9;         % N   - average thrust [RECOMPUTED from the G38
                            %     Motor Thrust lookup block's own breakpoint
                            %     data via trapezoidal integration -- the
                            %     40.2N datasheet-nameplate value didn't match
                            %     what the lookup table (the thing actually
                            %     driving the sim) integrates to]
t_burn      = 2.64;         % s   - burn time (matches the lookup table's
                            %     last breakpoint exactly)

%% --- PID GAINS [TUNE] - degrees loop: deg of fin per deg of error ---
% Firmware runs PD-only on ALL THREE axes (config.h: RATE_KI=0 for roll/
% pitch/yaw, no angle-integral term exists at all) -- "0 = pure-PD
% (recommended starting point)... risks winding up over a short, dynamic
% powered-ascent phase. Only consider enabling after PD-only behavior is
% validated." Ki_roll/Ki_pitch zeroed to match (were 0.001) -- same
% mismatch class as yaw's Ki, just smaller magnitude.
% x row - transverse tilt ("Roll PID" in your diagram)
Kp_roll     = 0.4363;       % [RECONCILED, 2026-09] was 1.0 -- same units
                            % mismatch class as Kd (see below): this Kp
                            % multiplies a DEGREES angle-error signal (traced
                            % Roll PID's input back through Gain2, 360/pi,
                            % to the quaternion-error computation), while
                            % firmware's ATTITUDE_ROLL_ANGLE_KP=25.0f is
                            % "deg fin per RADIAN of drift" (config.h,
                            % applied to tiltX/tiltY -- asinf/atan2f,
                            % radian-native). Converted: 25.0*(pi/180) =
                            % 0.4363. CAVEAT: the Kd settling-time sweep
                            % below was run against the OLD Kp=1.0, not
                            % this value -- a different Kp changes the whole
                            % system's dynamics (critical damping point
                            % moves too), so that sweep should be re-run
                            % now that Kp is also firmware-representative.
Ki_roll     = 0;
Kd_roll     = 0.5;          % deg fin per deg/s -> feed wx*R2D (deg/s!)
                            % [TUNE, 2026-09, NEEDS RE-VALIDATION] settling-
                            % time sweep (0 diverged, 0.07 rang/overshot,
                            % 0.5 settled clean ~3s, 1 took 3.5s, 2 took
                            % 11.5s, 100 saturated) was run with Kp_roll=1.0,
                            % not the now-corrected 0.4363 above -- re-sweep
                            % before trusting 0.5 as final. Firmware's
                            % ATTITUDE_ROLL_RATE_KP already updated to this
                            % value's unit-converted equivalent (config.h);
                            % re-derive that too if this changes.
% y row - transverse tilt ("Pitch PID")
Kp_pitch    = 0.4363;       % [RECONCILED, 2026-09] same reasoning as
                            % Kp_roll above -- same physics both axes.
Ki_pitch    = 0;
Kd_pitch    = 0.5;          % [TUNE, 2026-09, NEEDS RE-VALIDATION] same
                            % caveat as Kd_roll above.
                            % Left literally as Kd_roll's value, not the
                            % variable itself, so the two can diverge later
                            % if roll/pitch ever need separate tuning.
% z row - spin about long axis ("Yaw PID" - the WEAK channel: Cl_delta
% small, arm 0.0524 vs 0.31 -> needs its own much gentler tuning)
%
% Yaw is now a pure RATE controller on the real firmware (ATTITUDE_YAW_
% ANGLE_KP=0 in config.h) -- angle term deliberately disabled, only rate
% damping survives. Kp_yaw/Ki_yaw zeroed to match. There's no firmware
% angle-integral term at all (any axis) and no derivative-of-error term
% either (firmware's "damping" is direct proportional-on-rate, not a
% classic PID D) -- see the Yaw PID block's own D field and the separate
% Kd_yaw-driven rate-gain block in the .slx, both zeroed/fixed to match
% this architecture, not just this file.
Kp_yaw      = 0;
Ki_yaw      = 0;
Kd_yaw      = 0.005;         % [TUNE] -- sim-tuned rate gain, not required
                            %     to numerically match firmware's RATE_KP
                            %     (validated independently, sim vs bench)
% No N filter coefficients: D comes from the gyro rate directly, so there
% is no error derivative to filter. If you ever switch to derivative-on-
% error, reintroduce N here AND implement the identical filter in firmware.

%% --- SIMULATION SETTINGS ---
t_end       = 30;           % s - simulation end time
dt_max      = 5e-4;         % s - solver MAX step (0.01 was too coarse next to
                            %     Ts=0.003 holds and the 15 ms servo lag)
t_detect    = 3.0;          % s - ground detection delay

%% --- INITIAL CONDITIONS ---
pos0        = [0; 0; 0];    % m
vel0        = [0; 0; 0];    % m/s
euler0      = [0; 0; 0];    % rad
omega0      = [0; 0; 0];    % rad/s

%% --- DERIVED QUANTITIES [COMPUTED] ---
W           = m_total * g;  % N - weight at launch
T_W_ratio   = T_avg / W;    % -  - thrust to weight
mdot        = m_prop/t_burn;% kg/s - propellant mass flow

%% --- SANITY PRINTS - run the file, read the numbers ---
fprintf('Static margin: %.2f cal (want >1, <~4)\n', SM);
fprintf('T/W: %.2f (want >5 off a rail)\n', T_W_ratio);
fprintf('Arms  passive: %.3f | fin l_f: %.3f | spin: %.3f m\n', ...
        L_arm_passive, L_arm_roll, L_arm_yaw);
fprintf('Max fin moment @ 50 m/s, 15 deg: %.3f N.m\n', ...
        0.5*rho*50^2*A_ref*Cn_delta*delta_max*L_arm_pitch);
fprintf('Hinge moment margin @ 50 m/s: fin load*arm vs %.3f N.m stall\n', T_stall);