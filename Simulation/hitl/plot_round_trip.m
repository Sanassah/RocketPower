function plot_round_trip()
%PLOT_ROUND_TRIP Combined report of the last HITL run's real diagnostics --
%   round-trip latency, the real firmware's own loop timing, and its state
%   trace, all in one place. TeensyBridge.m logs all three on the SAME
%   step index and, when the run stops, dumps them to the base workspace
%   as hitlRoundTripMs (ms), hitlLoopUs (us), hitlState (0-6) -- see
%   TeensyBridge.m's releaseImpl. Run this AFTER stopping a HITL run
%   (Normal Sim runs never populate them, since TeensyBridge's stepImpl
%   never actually executes then).
    if ~evalin('base', 'exist(''hitlRoundTripMs'', ''var'')')
        error('plot_round_trip:noData', ...
            ['hitlRoundTripMs not found in the base workspace -- run a HITL ' ...
             'simulation (HITL Mode Select flipped to HITL) and stop it first.']);
    end
    rt = evalin('base', 'hitlRoundTripMs');

    haveLoopUs = evalin('base', 'exist(''hitlLoopUs'', ''var'')');
    haveState  = evalin('base', 'exist(''hitlState'', ''var'')');

    fprintf('Round trip (ms) over %d steps: mean=%.2f median=%.2f max=%.2f std=%.2f\n', ...
        numel(rt), mean(rt), median(rt), max(rt), std(rt));

    if haveLoopUs
        lu = evalin('base', 'hitlLoopUs');
        fprintf('Real firmware loop (us): mean=%.0f median=%.0f max=%.0f std=%.0f (%.1f Hz avg)\n', ...
            mean(lu), median(lu), max(lu), std(lu), 1e6 / mean(lu));
    end

    stateNames = {'IDLE','ARMED','POWERED_ASCENT','COAST','APOGEE','DESCENT','LANDED'};
    if haveState
        st = evalin('base', 'hitlState');
        uState = unique(st);
        fprintf('States seen this run: ');
        for i = 1:numel(uState)
            s = uState(i);
            if s >= 0 && s <= 6
                fprintf('%s ', stateNames{s+1});
            else
                fprintf('STATE_%d ', s);
            end
        end
        fprintf('\n');
    end

    nPlots = 2 + haveLoopUs + haveState;
    figure('Name', 'HITL run report');
    p = 1;

    subplot(nPlots, 1, p); p = p + 1;
    plot(rt, '.-');
    xlabel('step'); ylabel('round trip (ms)');
    title('Round-trip latency per step');
    grid on;

    if haveLoopUs
        subplot(nPlots, 1, p); p = p + 1;
        plot(lu / 1000, '.-', 'Color', [0.85 0.33 0.1]);
        xlabel('step'); ylabel('loop time (ms)');
        title('Real firmware main.cpp loop duration per step (one-loop-lagged)');
        grid on;
    end

    if haveState
        subplot(nPlots, 1, p); p = p + 1;
        plot(st, '.-', 'Color', [0.47 0.67 0.19]);
        xlabel('step'); ylabel('state');
        yticks(0:6); yticklabels(stateNames);
        title('Real firmware FlightState per step');
        grid on;
    end

    subplot(nPlots, 1, p);
    histogram(rt);
    xlabel('round trip (ms)');
    ylabel('count');
    title('Round-trip distribution');
    grid on;
end
