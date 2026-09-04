function v_body = quatRotateWorldToBody(q, v_world)
%QUATROTATEWORLDTOBODY Rotate a world-frame vector into body frame.
%   q = [w x y z], the SAME body-to-world quaternion convention as
%   RocketPowerSim's "6DOF (Quaternion)" block output (out4) and the real
%   BNO085/AttitudeController.cpp's quaternion math -- v_world = R(q)*v_body,
%   so v_body = R(q)' * v_world.
%
%   Only ever used here to project gravity into body frame for the HITL
%   bridge's highg (specific-force) estimate -- see TeensyBridge.m. Only the
%   MAGNITUDE of that result is safety-relevant (StateMachine/BackupDeploy
%   threshold on highg_mag_g only, never a per-axis component), so an exact
%   sign/axis convention match against the real BNO085 mounting isn't
%   required for correctness here.
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1-2*(qy^2+qz^2),   2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy)
         2*(qx*qy+qw*qz),   1-2*(qx^2+qz^2),   2*(qy*qz-qw*qx)
         2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx^2+qy^2)];
    v_body = R' * v_world(:);
end
