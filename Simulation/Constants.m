%% --- GEOMETRY ---
d           = 0.067;        % m  - Rocket body diameter
r           = d/2;          % m  - Rocket body radius
L_rocket    = 0.73;          % m  - Total rocket length
A_ref       = pi*r^2;       % m² - Reference area (cross section) = 0.000661 m²

%% --- MASS PROPERTIES ---
m_total     = 1.0;          % kg - Total rocket mass at launch
m_prop      = 0.055;        % kg - Propellant mass (G38 datasheet)
m_dry       = m_total - m_prop; % kg - Dry mass after burnout = 0.945 kg
t_burn      = 2.64;         % s  - Motor burn time (G38 datasheet)

%% --- INERTIA (dummy values - replace with Fusion360) ---
Ixx         = 0.08;        % kg·m² - Roll inertia  
Iyy         = 0.08;         % kg·m² - Pitch inertia
Izz         = 0.002;         % kg·m² - Yaw inertia   

%% --- CENTER OF MASS & PRESSURE (from OpenRocket - dummy for now) ---
CG          = [0; 0; 0.382]; % m - CG position from nose tip
CP          = [0; 0; 0.457]; % m - CP position from nose tip
SM          = (CP(3)-CG(3))/d; % calibers - Static stability margin (should be >1)

%% --- AERODYNAMICS ---
Cn_alpha    = 2.0;          % /rad - Normal force coefficient slope
Cd          = 0.5;          % -    - Drag coefficient (get from OpenRocket)
Cl_delta    = 0.1;          % /rad - Roll moment coefficient (smaller than Cn_alpha)
L_ref       = d;            % m    - Reference length = diameter 
L_arm       = CP(3)-CG(3);  % m    - Moment arm (CP to CG distance)

%% --- ATMOSPHERE (sea level standard) ---
rho         = 1.225;        % kg/m³ - Air density at sea level
g           = 9.81;         % m/s²  - Gravitational acceleration
v_sound     = 343;          % m/s   - Speed of sound at sea level

%% --- FIN GEOMETRY ---
n_fins      = 4;                    % -    - Number of fins
delta_max   = 0.2618;               % rad  - Max fin deflection = +15°
delta_min   = -delta_max;           % rad  - Min fin deflection = -15°
fin_span    = 0.05;                 % m    - Fin span (MEASURE YOUR FINS)
fin_chord   = 0.04;                 % m    - Fin root chord (MEASURE YOUR FINS)
fin_area    = 0.5*fin_span*fin_chord; % m² - Approximate fin area = 0.001 m²
b           = fin_span;             % m    - Roll moment arm = fin span = 0.05m

%% --- MOTOR (G38 AeroTech) ---
T_avg       = 40.2;         % N   - Average thrust
T_max       = 78.2;         % N   - Max thrust
I_total     = 87.7;         % N·s - Total impulse
t_burn      = 2.64;         % s   - Burn time

%% --- PID GAINS (tune these!) ---
% Pitch
Kp_pitch    = 1.0;          % Proportional gain
Ki_pitch    = 0.001;        % Integral gain
Kd_pitch    = 2.0;          % Derivative gain
N_pitch     = 10;           % Filter coefficient

% Yaw
Kp_yaw      = 1.0;          % Proportional gain
Ki_yaw      = 0.001;        % Integral gain
Kd_yaw      = 2.0;          % Derivative gain
N_yaw       = 10;           % Filter coefficient

% Roll (direct moment - no aero chain)
Kp_roll     = 0.01;         % Proportional gain
Ki_roll     = 0.001;        % Integral gain
Kd_roll     = 0.005;        % Derivative gain
N_roll      = 10;           % Filter coefficient

%% --- SIMULATION SETTINGS ---
t_end       = 30;           % s    - Simulation end time
dt          = 0.01;         % s    - Solver max step size
t_detect    = 3.0;          % s    - Ground detection delay

%% --- INITIAL CONDITIONS ---
pos0        = [0; 0; 0];    % m    - Initial position [x;y;z]
vel0        = [0; 0; 0];    % m/s  - Initial velocity
euler0      = [0; 0; 0];    % rad  - Initial [roll;pitch;yaw]
omega0      = [0; 0; 0];    % rad/s - Initial angular rates

%% --- DERIVED QUANTITIES ---
W           = m_total * g;  % N    - Rocket weight at launch
T_W_ratio   = T_avg / W;   % -    - Thrust to weight ratio 
mdot        = m_prop/t_burn;% kg/s - Propellant mass flow rate