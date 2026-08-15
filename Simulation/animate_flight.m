%% animate_flight.m
% Post-processing 3D flight animation - reads logged simulation output
% from the base workspace and steps through it frame by frame.
%
% Requires three variables already in the base workspace (created by the
% "To Workspace" blocks added to RocketPowerSim.slx + the "Time" logging
% checkbox in Model Configuration Parameters -> Data Import/Export):
%   tout      - Nx1  simulation time, s
%   pos_log   - Nx3  position [Xx Xy Xz], m (Xz assumed positive-down,
%                     matching the rest of this model - flipped below)
%   q_log     - Nx4  attitude quaternion [q0 q1 q2 q3], q0 = scalar part
%
% Run this AFTER pressing Run on the model, in the same MATLAB session
% (so those variables are still sitting in the base workspace).

if ~exist('tout','var') || ~exist('pos_log','var') || ~exist('q_log','var')
    error(['animate_flight:missingLogs', newline, ...
        'tout / pos_log / q_log not found in the workspace. Run the ', ...
        'simulation first (with Time logging + the pos_log/q_log To ', ...
        'Workspace blocks wired up), then run this script.']);
end

t   = tout(:);
pos = pos_log;              % Nx3: [Xx Xy Xz]
q   = q_log;                % Nx4: [q0 q1 q2 q3]

alt = -pos(:,3);            % altitude, up-positive (Xz is down-positive)

%% --- Rocket icon geometry, in the model's own body frame (z = nose) ---
% Small cone (nose) + cylinder (body), built along local +z so no axis
% relabeling is ever needed - this is the model's own convention already.
% Sized as a fraction of the actual flight envelope (not real meters) so
% the icon stays visible whether the flight reaches 10 m or 10 km - a
% true-to-scale rocket would be an invisible dot against real altitude.
spanXY0 = max(0.5, max(abs([pos(:,1); pos(:,2)]))*1.2);
if spanXY0 == 0, spanXY0 = 1; end
spanZ0  = max(1, max(alt) - min(0,min(alt)));
overallSpan = max([2*spanXY0, spanZ0]);

rocketScale = 0.06*overallSpan;   % icon size, ~6% of the flight envelope
bodyLen   = rocketScale;
noseLen   = rocketScale/3;
rad       = rocketScale/7.5;

nseg = 12;
th = linspace(0, 2*pi, nseg);
circ_x = rad*cos(th);
circ_y = rad*sin(th);

% Body: a ring at z=0 and a ring at z=-bodyLen
ringLow  = [circ_x; circ_y; -bodyLen*ones(1,nseg)]';   % nseg x 3
ringHigh = [circ_x; circ_y; zeros(1,nseg)]';           % nseg x 3
noseTip  = [0 0 noseLen];

bodyV = [ringLow; ringHigh];               % 2*nseg x 3
noseV = [ringHigh; noseTip];               % nseg+1 x 3

%% --- Figure setup (standard light MATLAB theme) ---
f = figure('Name','Flight Animation','Color','w');
ax = axes('Parent',f); hold(ax,'on'); box(ax,'on');
grid(ax,'on');
axis(ax,'equal');
view(ax,45,20);
xlabel(ax,'x'); ylabel(ax,'y'); zlabel(ax,'z (altitude)');

xlim(ax,[-spanXY0 spanXY0]);
ylim(ax,[-spanXY0 spanXY0]);
zlim(ax,[min(0,min(alt)) max(1,max(alt)*1.1)]);

trajLine = plot3(ax, pos(1,1), pos(1,2), alt(1), 'b-','LineWidth',1.2);

bodyPatch = patch('Parent',ax,'Vertices',bodyV,'Faces',reshape(1:2*nseg,nseg,2)', ...
    'FaceColor',[0.2 0.2 0.2],'EdgeColor','none');
% simple side faces for the cylinder body (quad strip)
sideFaces = zeros(nseg,4);
for i = 1:nseg
    j = mod(i,nseg)+1;
    sideFaces(i,:) = [i, j, j+nseg, i+nseg];
end
set(bodyPatch,'Faces',sideFaces);

nosePatch = patch('Parent',ax,'Vertices',noseV,'Faces',[],'FaceColor',[0.2 0.2 0.2],'EdgeColor','none');
noseFaces = zeros(nseg,3);
for i = 1:nseg
    j = mod(i,nseg)+1;
    noseFaces(i,:) = [i, j, nseg+1];
end
set(nosePatch,'Faces',noseFaces);

triadLen = 3*bodyLen;
triadX = plot3(ax,[0 0],[0 0],[0 0],'r-','LineWidth',2);
triadY = plot3(ax,[0 0],[0 0],[0 0],'g-','LineWidth',2);
triadZ = plot3(ax,[0 0],[0 0],[0 0],'b-','LineWidth',2);

infoBox = annotation(f,'textbox',[0.62 0.75 0.3 0.15], ...
    'String', {'Flight'}, 'FitBoxToText','on', ...
    'BackgroundColor','w','EdgeColor','k');

%% --- Replay button: re-runs the same flight without re-running this script ---
skip = max(1, floor(numel(t)/300));   % cap ~300 drawn frames for speed
replayCallback = @(~,~) playFrames(t, pos, alt, q, bodyV, noseV, triadLen, ...
    bodyPatch, nosePatch, triadX, triadY, triadZ, trajLine, infoBox, skip);

uicontrol('Parent',f,'Style','pushbutton','String','Replay', ...
    'Units','normalized','Position',[0.02 0.02 0.12 0.06], ...
    'Callback',replayCallback);

%% --- Play once immediately ---
playFrames(t, pos, alt, q, bodyV, noseV, triadLen, ...
    bodyPatch, nosePatch, triadX, triadY, triadZ, trajLine, infoBox, skip);

%% --- Local function: steps through the log and redraws each frame ---
function playFrames(t, pos, alt, q, bodyV, noseV, triadLen, ...
        bodyPatch, nosePatch, triadX, triadY, triadZ, trajLine, infoBox, skip)
for k = 1:skip:numel(t)
    q0 = q(k,1); q1 = q(k,2); q2 = q(k,3); q3 = q(k,4);
    R = [1-2*(q2^2+q3^2),   2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2)
         2*(q1*q2+q0*q3),   1-2*(q1^2+q3^2),   2*(q2*q3-q0*q1)
         2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1),   1-2*(q1^2+q2^2)];

    p = [pos(k,1); pos(k,2); alt(k)];

    bV = (R*bodyV')' + p';
    nV = (R*noseV')' + p';
    set(bodyPatch,'Vertices',bV);
    set(nosePatch,'Vertices',nV);

    ex = R*[triadLen;0;0]; ey = R*[0;triadLen;0]; ez = R*[0;0;triadLen];
    set(triadX,'XData',[p(1) p(1)+ex(1)],'YData',[p(2) p(2)+ex(2)],'ZData',[p(3) p(3)+ex(3)]);
    set(triadY,'XData',[p(1) p(1)+ey(1)],'YData',[p(2) p(2)+ey(2)],'ZData',[p(3) p(3)+ey(3)]);
    set(triadZ,'XData',[p(1) p(1)+ez(1)],'YData',[p(2) p(2)+ez(2)],'ZData',[p(3) p(3)+ez(3)]);

    set(trajLine,'XData',pos(1:k,1),'YData',pos(1:k,2),'ZData',alt(1:k));

    speed = 0;
    if k > 1
        dt = t(k)-t(k-1);
        if dt > 0
            speed = norm(p - [pos(k-1,1);pos(k-1,2);alt(k-1)])/dt;
        end
    end
    set(infoBox,'String', { ...
        sprintf('t = %.2f s', t(k)), ...
        sprintf('altitude = %.1f m', alt(k)), ...
        sprintf('speed = %.1f m/s', speed) });

    drawnow;
end
end
