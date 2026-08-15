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

%% --- GEOMETRY [MEASURED] ---
d           = 0.067;        % m  - Body diameter
r           = d/2;          % m  - Body radius
L_rocket    = 0.73;         % m  - Total length
A_ref       = pi*r^2;       % m2 - Reference area (cross section) = 0.003526 m2
                            % All aero coefficients below reference THIS area.

%% --- MASS PROPERTIES [MEASURED] ---
m_total     = 1.0;          % kg - Total mass at launch
m_prop      = 0.055;        % kg - Propellant mass (G38 datasheet)
m_dry       = m_total - m_prop; % kg - Dry mass after burnout = 0.945

%% --- INERTIA [PLACEHOLDER - replace with Fusion360] ---
% Drone convention: z = longitudinal => Izz is the SMALL spin inertia.
Ixx         = 0.08;         % kg.m2 - Transverse (tilt about x)
Iyy         = 0.08;         % kg.m2 - Transverse (tilt about y)
Izz         = 0.002;        % kg.m2 - Spin about the long axis

%% --- MASS CENTER & AERO CENTERS (all measured from NOSE TIP, along z) ---
CG_z        = 0.382;        % m - [OPENROCKET/balance test] Center of gravity:
                            %     the pivot. Every arm is measured FROM here.
CP_rocket_z = 0.457;        % m - [OPENROCKET] WHOLE-ROCKET center of pressure:
                            %     where total aero force acts at angle of attack.
                            %     Weighted average of all components' CPs.
                            %     Used by the PASSIVE aero subsystem only.
CP_fin_z    = 0.70;         % m - [BARROWMAN/PLACEHOLDER] FIN center of pressure:
                            %     where the DEFLECTION force acts. One of the
                            %     ingredients inside CP_rocket_z, isolated here
                            %     because fin deflection adds force only there.
CG          = [0; 0; CG_z]; % m - vector forms, if blocks want them
CP          = [0; 0; CP_rocket_z];
SM          = (CP_rocket_z - CG_z)/d; % calibers - [COMPUTED] static margin, want >1

%% --- PASSIVE AERODYNAMICS (weathercock subsystem - NOT the fin chains) ---
Cn_alpha    = 2.0;          % /rad - [OPENROCKET/PLACEHOLDER] whole-rocket normal
                            %     force slope, ref A_ref. NOTE: 2.0 looks like a
                            %     nose-alone value; a finned rocket is typically
                            %     8-12 on this reference. Pull the real one.
Cmq         = -8.0;         % /rad - [PLACEHOLDER] pitch damping derivative,
                            %     ref A_ref and L_ref (add when passive
                            %     subsystem is built; keeps sim from ringing)
Cd          = 0.5;          % -    - [OPENROCKET] drag coefficient
L_ref       = d;            % m    - reference length for moment coefficients
L_arm_passive = CP_rocket_z - CG_z; % m - [COMPUTED] rocket CP to CG = 0.075.
                            %     PASSIVE subsystem arm ONLY. Never in fin chains.

%% --- FIN GEOMETRY [MEASURED - go measure, these are guesses] ---
n_fins      = 4;            % -   - Number of fins
fin_span    = 0.05;         % m   - Semispan, root to tip [PLACEHOLDER]
fin_root    = 0.04;         % m   - Root chord [PLACEHOLDER]
fin_tip     = 0.02;         % m   - Tip chord [PLACEHOLDER]
fin_area    = 0.5*(fin_root+fin_tip)*fin_span; % m2 - [COMPUTED] trapezoid
                            %     panel area = 0.0015 (bookkeeping/hinge check;
                            %     the moment chains use A_ref, not this)

%% --- FIN CONTROL AERODYNAMICS (the three moment chains) ---
Cn_delta    = 15;           % /rad - [BARROWMAN/PLACEHOLDER] deflecting fin PAIR
                            %     effectiveness, ref A_ref (body cross-section
                            %     convention - must stay paired with A_ref!).
                            %     NOT the same thing as Cn_alpha.
Cl_delta    = 0.1;          % /rad - [PLACEHOLDER] differential-deflection spin
                            %     coefficient for the z channel, ref A_ref
% Moment arms - names match the gain blocks in your Simulink diagram:
L_arm_roll  = CP_fin_z - CG_z;    % m - x row: fin CP to CG = 0.318 [COMPUTED]
L_arm_pitch = L_arm_roll;         % m - y row: same physics, same variable
L_arm_yaw   = r + 0.45*fin_span;  % m - z row (spin): RADIAL arm = 0.056
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

%% --- MOTOR (AeroTech G38) [MEASURED from datasheet] ---
T_avg       = 40.2;         % N   - average thrust
T_max       = 78.2;         % N   - max thrust
I_total     = 87.7;         % N.s - total impulse
t_burn      = 2.64;         % s   - burn time

%% --- PID GAINS [TUNE] - degrees loop: deg of fin per deg of error ---
% x row - transverse tilt ("Roll PID" in your diagram)
Kp_roll     = 1.0;
Ki_roll     = 0.001;
Kd_roll     = 2.0;          % deg fin per deg/s -> feed wx*R2D (deg/s!)
% y row - transverse tilt ("Pitch PID")
Kp_pitch    = 1.0;
Ki_pitch    = 0.001;
Kd_pitch    = 2.0;
% z row - spin about long axis ("Yaw PID" - the WEAK channel: Cl_delta
% small, arm 0.056 vs 0.318 -> needs its own much gentler tuning)
Kp_yaw      = 0.01;
Ki_yaw      = 0.001;
Kd_yaw      = 0.005;
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