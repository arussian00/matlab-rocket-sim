function fig = plotFlight(out)
%PLOTFLIGHT  Nine-panel dashboard of a simulateBooster result.

P = out.P;
t = out.t; X = out.X; a = out.aux;
x = X(:,1); h = X(:,2); vx = X(:,3); vh = X(:,4); m = X(:,5);
spd = hypot(vx, vh);
nPh = numel(out.phaseList);
labels = cell(1, nPh);
for k = 1:nPh, labels{k} = sprintf('%d: %s', k, out.phaseList{k}); end
C = lines(nPh);                         % one color per phase

fig = figure('Name', ['Flight data - ' P.vehicleName], 'Color', 'w', ...
             'Position', [40 40 1300 820]);
tl = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('%s  -  %s', P.vehicleName, strjoin(out.phaseList, ' > ')));

%% 1. Trajectory (altitude vs downrange), colored by phase
nexttile; hold on
for k = 1:nPh
    idx = out.phase == k;
    plot(x(idx)/1e3, h(idx)/1e3, 'LineWidth', 2, 'Color', C(k,:));
end
yline(100, '--k', 'Karman line');
plot(P.xPad/1e3, 0, 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
xlabel('Downrange [km]'); ylabel('Altitude [km]'); title('Trajectory');
legend(labels, 'Location', 'best', 'FontSize', 7); grid on

%% 2. Altitude vs time
nexttile; phasePlot(t, h/1e3, out.phase, C); yline(100, '--k');
xlabel('Time [s]'); ylabel('Altitude [km]'); title('Altitude'); grid on

%% 3. Speed
nexttile; phasePlot(t, spd, out.phase, C);
xlabel('Time [s]'); ylabel('Speed [m/s]'); title('Speed'); grid on

%% 4. g-load
nexttile; phasePlot(t, a.gLoad, out.phase, C);
xlabel('Time [s]'); ylabel('[g]'); title('Sensed acceleration (g-load)'); grid on

%% 5. Dynamic pressure
nexttile; phasePlot(t, a.q/1e3, out.phase, C);
xlabel('Time [s]'); ylabel('q [kPa]'); title('Dynamic pressure'); grid on

%% 6. Mass and throttle (with the minimum-throttle limit)
nexttile;
yyaxis left;  plot(t, m/1e3, 'LineWidth', 2); ylabel('Mass [t]');
yyaxis right; plot(t, 100*a.throttle, 'LineWidth', 1.2); hold on
yline(100*P.thrMin, ':', 'min throttle'); ylabel('Throttle [%]'); ylim([0 110]);
xlabel('Time [s]'); title('Mass & throttle'); grid on

%% 7. Pitch attitude: actual vs commanded
nexttile; hold on
plot(t, rad2deg(a.thetaCmd), 'k:', 'LineWidth', 1.2);
plot(t, rad2deg(X(:,6)), 'LineWidth', 1.6);
xlabel('Time [s]'); ylabel('Pitch from vertical [deg]');
title('Attitude (solid) vs command (dotted)'); grid on

%% 8. Control effort: gimbal angle and RCS torque
nexttile;
yyaxis left;  plot(t, rad2deg(a.gimbal), 'LineWidth', 1.2); ylabel('Gimbal [deg]');
yline(P.gimbalMaxDeg, ':'); yline(-P.gimbalMaxDeg, ':');
yyaxis right; plot(t, a.tauRCS/1e3, 'LineWidth', 1.2); ylabel('RCS torque [kN m]');
xlabel('Time [s]'); title('Control effort'); grid on

%% 9. Close-up of the final descent
nexttile; hold on
idx = h < 3000 & t > t(end) - 120;
plot(x(idx) - P.xPad, h(idx), 'LineWidth', 2);
plot(0, 0, 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
xlabel('Distance from pad [m]'); ylabel('Altitude [m]');
title('Final approach (last 3 km)'); grid on
end

function phasePlot(t, y, ph, C)
% Plot y(t) with a different color for every flight phase.
hold on
for k = unique(ph).'
    idx = ph == k;
    plot(t(idx), y(idx), 'LineWidth', 1.8, 'Color', C(k,:));
end
end
