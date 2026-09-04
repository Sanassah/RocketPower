classdef TeensyBridge < matlab.System
    % TeensyBridge  Real-time HITL bridge between RocketPowerSim and the
    % REAL Teensy 4.1 flight computer, running the rocketpower_hitl
    % PlatformIO build (see FlightComputer/platformio.ini), over a second
    % Teensy USB virtual serial port (SerialUSB1).
    %
    % Every discrete step (period Ts):
    %   1. Paces itself to wall clock, so 1s of block sample time = 1s of
    %      real elapsed time. This is not optional: the real firmware's
    %      safety timing (liftoff-confirm, apogee-detect window, backup-fire
    %      delay -- see StateMachine.cpp/BackupDeploy.cpp) is all driven by
    %      the Teensy's own hardware millis(), which this bridge cannot
    %      override or feed simulated time into (see the header comment on
    %      SensorInjectPacket in Packet.h for why). Real time is the only
    %      clock both sides can agree on.
    %   2. Builds one SensorInjectPacket from the plant state and writes it
    %      to the Teensy.
    %   3. Blocks (up to ResponseTimeoutS) for exactly one HITLResponsePacket
    %      back -- the real, compiled StateMachine/AttitudeController/
    %      FinController's actual decision for this sample, clamping
    %      included.
    %   4. Recovers pre-allocation roll/pitch/yaw commands (deg) from the 4
    %      real per-fin degrees (CH1=S, CH2=E, CH3=N, CH4=W -- see
    %      AttitudeController.h's allocation table) so they drop straight
    %      into RocketPowerSim's existing per-axis actuator/moment chain
    %      (the "15deg sat"/"1000deg/s"/"Servo lag" blocks) in place of the
    %      model's own simulated Roll/Pitch/Yaw PID -- see
    %      build_hitl_model.m, which does that splice. As of the firmware's
    %      config.h/AttitudeController.cpp axis-naming fix, "roll"/"pitch"/
    %      "yaw" mean the SAME physical thing on both sides now (roll/pitch
    %      = the two transverse-tilt axes, yaw = spin about the rocket's
    %      own longitudinal axis, matching Constants.m's own convention) --
    %      build_hitl_model.m wires output i straight to actuator row i.
    %
    % Input (1 port, 10-wide double vector) -- pack these 3 signals
    % straight off RocketPowerSim's "6DOF (Quaternion)" block via a Mux, in
    % this order (empirically confirmed against the live model -- see
    % Simulation/hitl/README.md for how):
    %   u(1:4)  quaternion [w x y z]                    (6DOF out4)
    %   u(5:7)  body angular rate [wx wy wz], rad/s      (6DOF out7, "Wb")
    %   u(8:10) position [x y z], m, Xe(3) UP-positive    (6DOF out2, "Xe")
    %           -- confirmed empirically against a real ascent/descent run,
    %           see stepImpl's comment on it; this is NOT the NED Z-down
    %           convention animate_flight.m's own comment describes
    %
    % The 6DOF block has NO acceleration output at all (confirmed
    % empirically -- differentiating every 3-wide output against every
    % other found d(out2)/dt == out1 almost exactly, i.e. out1 is Ve,
    % velocity in the SAME world/Xe frame, not acceleration; nothing else
    % matched an acceleration relationship either). So body-frame kinematic
    % acceleration is derived here instead, by differentiating Xe (world
    % position, already in u) TWICE and rotating the result into body frame
    % with the quaternion -- see stepImpl.
    %
    % Outputs:
    %   1 rollCmd, 2 pitchCmd, 3 yawCmd  -- degrees, the real firmware's
    %     recovered per-axis commands (see stepImpl).
    %   4 state  -- the real firmware's own FlightState this step (0=IDLE,
    %     1=ARMED, 2=POWERED_ASCENT, 3=COAST, 4=APOGEE, 5=DESCENT,
    %     6=LANDED, -1=no response received yet). Scope this alongside the
    %     commands to confirm directly whether a command is still changing
    %     because the firmware is still in POWERED_ASCENT/COAST, rather
    %     than cross-referencing GroundStation's own (differently-clocked)
    %     state display against this model's simulation-time x-axis.
    %   5 highg  -- 1x3 [x y z], g's, RAW specific force exactly as sent to
    %     the firmware (same value that feeds StateMachine's/BackupDeploy's
    %     highg_mag_g threshold checks) -- this is the plant-side signal,
    %     computed straight from the input regardless of whether the
    %     firmware actually responds this step.
    %   6 tiltDeg  -- angle (deg) between the current quaternion and
    %     IDENTITY ([1 0 0 0]), via 2*acosd(|q_w|). NOT necessarily "degrees
    %     off vertical" -- that depends on what orientation this model's own
    %     convention treats as identity -- but a monotonically growing value
    %     here means the airframe is rotating away from wherever it started,
    %     independent of any HITL/firmware behavior.
    %   7 altitude -- m, up-positive (same value sent as baro_alt_m).
    %   8 vertVel  -- m/s, up-positive (same value sent as vert_vel_ms).
    %   9 gyro  -- 1x3 [x y z] rad/s, body rate, straight passthrough of the
    %     input (same value sent as gyro_x/y/z).

    properties (Nontunable)
        ComPort (1,:) char = 'COM16'
        BaudRate (1,1) double = 115200
        Ts (1,1) double {mustBePositive} = 0.01
        ResponseTimeoutS (1,1) double {mustBePositive} = 0.5
    end

    properties (Access = private)
        Port
        StepIdx
        WallClockRef
        PrevXe
        PrevVelWorld
        HasPrevXe
        HasPrevVelWorld
        LastState
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Deliberately does NOT open the serial port -- see
            % ensurePortOpen(), called lazily from stepImpl() instead, for
            % why. Only cheap, hardware-free state goes here.
            obj.Port             = [];
            obj.StepIdx          = uint64(0);
            obj.WallClockRef     = tic;
            obj.PrevXe           = [0 0 0];
            obj.PrevVelWorld     = [0 0 0];
            obj.HasPrevXe        = false;
            obj.HasPrevVelWorld  = false;
            obj.LastState        = -1;   % -1 = no response received yet
        end

        function ensurePortOpen(obj)
            if ~isempty(obj.Port)
                return;
            end
            % Deliberately NOT in setupImpl: matlab.System's setupImpl runs
            % once during Simulink's initialization phase for EVERY block
            % in the diagram, INCLUDING ones inside a currently-disabled
            % Enabled Subsystem -- confirmed empirically (an Enabled
            % Subsystem's enable signal only gates stepImpl, not setup).
            % Opening the port in setupImpl would mean a plain "Normal Sim"
            % run (HITL switched off) still requires the Teensy plugged in
            % and running rocketpower_hitl, defeating the whole point of
            % the switch in RocketPowerSim.slx. stepImpl only ever actually
            % runs when this subsystem is enabled, so opening it here,
            % lazily, on the first REAL call is what makes "Normal Sim"
            % mode genuinely hardware-free.
            %
            % MATLAB's serialport() can be genuinely unstable (not just
            % "throws a catchable error") when handed a port that doesn't
            % exist or is already held open by another process -- a known
            % rough edge of that interface on Windows, not something this
            % class can fully paper over. So: never call it on a port we
            % haven't first confirmed is actually present. This is what
            % catches "wrong COM number" / "Teensy not plugged in" /
            % "GroundStation or another MATLAB session already has it
            % open" cleanly, as a normal Simulink error dialog, instead of
            % risking whatever the underlying crash was.
            available = serialportlist('available');
            if ~any(strcmpi(available, obj.ComPort))
                if isempty(available)
                    availStr = '(none detected)';
                else
                    availStr = strjoin(available, ', ');
                end
                error('TeensyBridge:portNotFound', [ ...
                    '%s is not an available serial port. Available right now: %s.\n' ...
                    'Check: is the Teensy plugged in and running the rocketpower_hitl build? ' ...
                    'Is this the SECOND port (SerialUSB1), not the one GroundStation is using? ' ...
                    'Does another program (GroundStation, a serial monitor, a previous MATLAB ' ...
                    'session that didn''t clean up) already have it open?'], ...
                    obj.ComPort, availStr);
            end

            try
                obj.Port = serialport(obj.ComPort, obj.BaudRate, 'Timeout', obj.ResponseTimeoutS);
                flush(obj.Port);
            catch cause
                obj.Port = [];
                err = MException('TeensyBridge:openFailed', ...
                    'Failed to open %s: %s', obj.ComPort, cause.message);
                throw(err);
            end
            obj.WallClockRef = tic;   % pace relative to when the link actually opened, not to setupImpl
            obj.StepIdx       = uint64(0);
        end

        function [rollCmd, pitchCmd, yawCmd, state, highg, tiltDeg, altOut, vvelOut, gyroOut] = stepImpl(obj, u)
            obj.ensurePortOpen();

            % ---- 1. pace to wall clock ----
            targetT = double(obj.StepIdx) * obj.Ts;
            nowT    = toc(obj.WallClockRef);
            if targetT > nowT
                pause(targetT - nowT);
            end
            obj.StepIdx = obj.StepIdx + 1;

            % ---- 2. unpack plant state ----
            quat = u(1:4);
            gyro = u(5:7);
            Xe   = u(8:10);
            % Xe(3) INCREASES as the rocket climbs (confirmed empirically
            % against a real ascent/descent run -- see README's "sign
            % conventions" note; this is the opposite of the NED Z-down
            % convention animate_flight.m's own pos_log/alt comment
            % describes, which evidently doesn't apply to this raw port).
            % Getting this backwards silently breaks StateMachine's COAST->
            % APOGEE zero-crossing check, since a real ascent-to-descent
            % transition then reads as vert_vel going NEGATIVE-to-POSITIVE
            % instead of the POSITIVE-to-NEGATIVE the firmware checks for
            % -- it can never fire, so the state machine gets stuck in
            % COAST for the rest of the flight no matter how long it runs.
            alt = Xe(3);   % up-positive

            % World-frame velocity and acceleration, both by straight finite
            % difference -- the 6DOF block has no acceleration output at
            % all (see class header), so this is derived, not read
            % directly. Two-stage warm-up: velocity needs one prior Xe,
            % acceleration needs one prior velocity, so acceleration isn't
            % valid until the third call.
            if obj.HasPrevXe
                velWorld = (Xe(:)' - obj.PrevXe) / obj.Ts;
            else
                velWorld = [0 0 0];
            end
            if obj.HasPrevVelWorld
                accelWorld = (velWorld - obj.PrevVelWorld) / obj.Ts;
            else
                accelWorld = [0 0 0];
            end
            obj.PrevXe          = Xe(:)';
            obj.PrevVelWorld    = velWorld;
            obj.HasPrevXe       = true;
            obj.HasPrevVelWorld = true;

            vvel = velWorld(3);   % up-positive, same Xe(3) convention as alt
            Ab   = quatRotateWorldToBody(quat, accelWorld(:));   % body-frame kinematic accel, m/s^2

            % Specific force (what a REAL accelerometer reads): kinematic
            % accel minus gravity's own projection into body frame. Gravity
            % pulls in -Z here (this world frame's +Z is up, per the same
            % empirical finding as altitude above) -- getting this sign
            % wrong doesn't just flip highg's per-axis direction, it
            % corrupts its MAGNITUDE too (the one thing StateMachine/
            % BackupDeploy actually threshold on) any time the airframe
            % isn't sitting still or perfectly vertical, since Ab and gBody
            % then aren't parallel and the wrong-signed subtraction doesn't
            % just negate cleanly. See quatRotateWorldToBody.m's header.
            g0    = 9.81;
            gBody = quatRotateWorldToBody(quat, [0; 0; -g0]);
            highgVec = (Ab(:) - gBody) / g0;                   % g's

            linAccel = Ab;   % already gravity-compensated -- matches the
                              % real BNO085 "linear acceleration" convention

            % ---- plant-side debug outputs -- valid regardless of whether
            % the firmware actually responds this step, since they're all
            % derived from the input u, not from the reply. ----
            highg   = highgVec(:)';
            tiltDeg = 2*acosd(min(1, abs(quat(1))));
            altOut  = alt;
            vvelOut = vvel;
            gyroOut = gyro(:)';

            % ---- 3. send SensorInjectPacket, block for the real reply ----
            pkt = hitl_encode_sensor(obj.StepIdx, quat, gyro, linAccel, highgVec, alt, vvel);
            write(obj.Port, pkt, 'uint8');

            resp = hitl_read_response(obj.Port, obj.ResponseTimeoutS);
            if isempty(resp)
                warning('TeensyBridge:timeout', ...
                    'No HITLResponsePacket within %.2fs -- link dropped or Teensy not in rocketpower_hitl build. Holding zero correction.', ...
                    obj.ResponseTimeoutS);
                rollCmd = 0; pitchCmd = 0; yawCmd = 0;
                state = obj.LastState;
                return;
            end

            % Recovers the firmware's own roll/pitch/yaw commands from its 4
            % real per-fin degrees, per AttitudeController.cpp's allocation
            % (S=yaw-roll, E=yaw-pitch, N=yaw+roll, W=yaw+pitch -- yaw is
            % the uniform/spin term, roll/pitch the two transverse
            % differentials, in this file's Simulation-matching convention,
            % see config.h's axis-naming note):
            finDeg   = resp.fin_deg;   % [S E N W], real post-clamp degrees
            rollCmd  = (finDeg(3) - finDeg(1)) / 2;   % N - S
            pitchCmd = (finDeg(4) - finDeg(2)) / 2;   % W - E
            yawCmd   = sum(finDeg) / 4;          % (N-S cancels, W-E cancels)/4 = yaw;
                                                  % average of all 4 -- pre-clamp yaw exactly
            state    = resp.state;
            obj.LastState = state;
        end

        function releaseImpl(obj)
            obj.Port = [];   % serialport closes the port when its last reference is cleared
        end

        function num = getNumInputsImpl(~)
            num = 1;
        end
        function num = getNumOutputsImpl(~)
            num = 9;
        end
        function [n1, n2, n3, n4, n5, n6, n7, n8, n9] = getOutputNamesImpl(~)
            n1 = 'rollCmd'; n2 = 'pitchCmd'; n3 = 'yawCmd'; n4 = 'state';
            n5 = 'highg'; n6 = 'tiltDeg'; n7 = 'altitude'; n8 = 'vertVel'; n9 = 'gyro';
        end
        function [sz1, sz2, sz3, sz4, sz5, sz6, sz7, sz8, sz9] = getOutputSizeImpl(~)
            sz1 = [1 1]; sz2 = [1 1]; sz3 = [1 1]; sz4 = [1 1];
            sz5 = [1 3]; sz6 = [1 1]; sz7 = [1 1]; sz8 = [1 1]; sz9 = [1 3];
        end
        function [dt1, dt2, dt3, dt4, dt5, dt6, dt7, dt8, dt9] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double'; dt3 = 'double'; dt4 = 'double';
            dt5 = 'double'; dt6 = 'double'; dt7 = 'double'; dt8 = 'double'; dt9 = 'double';
        end
        function [cp1, cp2, cp3, cp4, cp5, cp6, cp7, cp8, cp9] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false; cp3 = false; cp4 = false;
            cp5 = false; cp6 = false; cp7 = false; cp8 = false; cp9 = false;
        end
        function [fs1, fs2, fs3, fs4, fs5, fs6, fs7, fs8, fs9] = isOutputFixedSizeImpl(~)
            fs1 = true; fs2 = true; fs3 = true; fs4 = true;
            fs5 = true; fs6 = true; fs7 = true; fs8 = true; fs9 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end
    end
end
