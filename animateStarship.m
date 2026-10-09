function animateStarship(B, Sh, tSep, speed, fps, gifFile)
%ANIMATESTARSHIP  Live "mission control" view of a Starship flight.
%
%   animateStarship(fullB, fullS, tSep)            normal speed, 20 fps
%   animateStarship(fullB, fullS, tSep, 2)         twice as fast
%   animateStarship(..., speed, fps, 'x.gif')      also save a GIF (R2022a+)
%
%   fullB, fullS : complete flight histories of Super Heavy and Starship,
%                  starting at liftoff (built by rocket_starship_sim.m)
%   tSep         : hot-staging time [s]
%
%   ONE window, everything live at the same time (12 panels):
%   Top row   : trajectory overview with clock and event log, Earth view,
%               Super Heavy camera, Starship camera. Both vehicles are drawn
%               in both cameras: the full stack on the pad, hot staging, the
%               tower catch, the belly-flop and the flip.
%   Rows 2-3  : altitude, speed, g-load, dynamic pressure, throttle, angle
%               of attack, mass and heating of both vehicles. Every graph
%               grows as the flight plays.
%   Time runs faster during the quiet parts (shown as "time x...").

if nargin < 4 || isempty(speed), speed = 1;  end
if nargin < 5 || isempty(fps),   fps   = 20; end
if nargin < 6,                   gifFile = ''; end
octaveCompat();                                   % Octave support (no-op in MATLAB)
Pb = B.P;  Ps = Sh.P;
cB = [0.85 0.25 0.15];                 % Super Heavy colour in graphs
cS = [0.10 0.35 0.85];                 % Starship colour in graphs

%% STEP 1 - Frame times with a variable "time warp"
tCatch = B.t(end);
tEnd   = max(B.t(end), Sh.t(end));
[tuS, iuS] = unique(Sh.t, 'last');
hShip  = @(t) interp1(tuS, Sh.X(iuS,2), min(t, tuS(end)));
tf = zeros(1, 0);  warp = zeros(1, 0);  t = 0;
while t < tEnd
    w = 8;                                         % liftoff, staging, booster return
    if t > tCatch + 8                              % only the ship is still flying
        hs = hShip(t);
        if hs > 80e3,      w = 150;                % quiet coast above the atmosphere
        elseif hs > 5e3,   w = 30;                 % re-entry
        else,              w = 5;                  % flip and landing burn
        end
    end
    tf(end+1)   = t;  warp(end+1) = w;             %#ok<AGROW>
    t = t + speed * w / fps;
end
tf(end+1) = tEnd;  warp(end+1) = warp(end);

%% STEP 2 - Sample both vehicles at the frame times
b = sampleFlight(B, tf, Pb);
s = sampleFlight(Sh, tf, Ps);
s.thr(tf <= tSep) = 0;                             % ship engines light at separation
b.thr(tf > tCatch) = 0;  s.thr(tf >= Sh.t(end)) = 0;
% Once caught, the booster's flight data stops (it is held by the tower)
for f = {'gL', 'q', 'thr', 'aoa', 'heat'}
    b.(f{1})(tf > tCatch + 1) = NaN;
    s.(f{1})(tf > Sh.t(end) + 1) = NaN;
end
xPad = Pb.xPad;                                    % tower position

% Where to draw the ship: on top of the booster until separation, then
% slide smoothly onto its own (point-mass) position over ~40 s.
iSep  = find(tf >= tSep, 1);
nSep  = [sin(b.th(iSep)); cos(b.th(iSep))];
fade  = max(0, 1 - (tf - tSep)/40);
sx = s.x;  sh = s.h;
for k = 1:numel(tf)
    if tf(k) <= tSep
        n = [sin(b.th(k)); cos(b.th(k))];
        s.th(k) = b.th(k);  s.aoa(k) = b.aoa(k);   % attached: same attitude
    else
        n = nSep * fade(k);
    end
    sx(k) = s.x(k) + Pb.L*n(1);
    sh(k) = s.h(k) + Pb.L*n(2);
end

% Earth-centred coordinates for the Earth view [km]
toX = @(x, h) (Pb.Re + h).*sin(x/Pb.Re)/1e3;
toY = @(x, h) (Pb.Re + h).*cos(x/Pb.Re)/1e3;
ev  = missionEvents(B, Sh, tSep);                  % for the event log

%% STEP 3 - Build the mission-control window (12 panels)
fig = figure('Name', 'Starship flight test - mission control', 'Color', 'w', ...
             'Position', [20 20 1600 900]);
col = [0.035 0.285 0.535 0.785];  wid = 0.20;      % 4 columns
% Top row (taller): overview, Earth view, two cameras
axO = axes('Position', [col(1) 0.66 wid 0.29]);
axE = axes('Position', [col(2) 0.66 wid 0.29]);
axB = axes('Position', [col(3) 0.66 wid 0.30]);
axS = axes('Position', [col(4) 0.66 wid 0.30]);

% (a) Trajectory overview, with the clock and the event log
hold(axO, 'on'); grid(axO, 'on');
hTrB = plot(axO, nan, nan, '-', 'Color', cB, 'LineWidth', 1.5);
hTrS = plot(axO, nan, nan, '-', 'Color', cS, 'LineWidth', 1.5);
hDtB = plot(axO, nan, nan, 'o', 'Color', cB, 'MarkerFaceColor', cB, 'MarkerSize', 5);
hDtS = plot(axO, nan, nan, 'o', 'Color', cS, 'MarkerFaceColor', cS, 'MarkerSize', 5);
plot(axO, xPad/1e3, 0, 'k^', 'MarkerFaceColor', [1 0.8 0], 'MarkerSize', 7);
xlabel(axO, 'Downrange [km]'); ylabel(axO, 'Altitude [km]');
legend(axO, [hTrB hTrS], {'Super Heavy', 'Starship'}, 'Location', 'northwest', ...
       'AutoUpdate', 'off', 'FontSize', 7);
hClock = title(axO, '', 'FontName', 'Monospaced');
hLog = text(axO, 0.98, 0.97, '', 'Units', 'normalized', 'FontSize', 7, ...
            'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'FontName', 'Monospaced');

% (b) Earth view: the ship flies a third of the way around the planet
hold(axE, 'on'); axis(axE, 'equal');
th = linspace(0, 2*pi, 360);
fill(axE, Pb.Re/1e3*cos(th), Pb.Re/1e3*sin(th), [0.75 0.85 1], 'EdgeColor', [0.3 0.5 0.8]);
plot(axE, (Pb.Re + 100e3)/1e3*cos(th), (Pb.Re + 100e3)/1e3*sin(th), ':', 'Color', [0.5 0.5 0.5]);
hEB = plot(axE, nan, nan, '-', 'Color', cB, 'LineWidth', 1.5);
hES = plot(axE, nan, nan, '-', 'Color', cS, 'LineWidth', 1.5);
hEdB = plot(axE, nan, nan, 'o', 'Color', cB, 'MarkerFaceColor', cB, 'MarkerSize', 4);
hEdS = plot(axE, nan, nan, 'o', 'Color', cS, 'MarkerFaceColor', cS, 'MarkerSize', 4);
R  = 1.1*Pb.Re/1e3;
axis(axE, [-R R -R R]);
title(axE, 'Earth view (dotted: 100 km)'); xlabel(axE, 'km'); ylabel(axE, 'km');

% (c, d) Chase cameras
W = 100;                                           % half-width of each view [m]
gB = buildCamera(axB, Pb, Ps, xPad, 'Super Heavy camera');
gS = buildCamera(axS, Pb, Ps, xPad, 'Starship camera');

% (e-l) Live telemetry, two rows of four
names = {'Altitude [km]', 'Speed [km/s]', 'g-load [g]', 'Dynamic pressure [kPa]', ...
         'Throttle [%]', 'Angle of attack [deg]', 'Mass [t] (log)', 'Heating [kW/m^2]'};
yB = {b.h/1e3, b.v/1e3, b.gL, b.q/1e3, 100*b.thr, b.aoa, b.m/1e3, b.heat/1e3};
yS = {sh/1e3,  s.v/1e3, s.gL, s.q/1e3, 100*s.thr, s.aoa, s.m/1e3, s.heat/1e3};
nT = numel(names);  axT = cell(1, nT);  lB = cell(1, nT);  lS = cell(1, nT);
for j = 1:nT
    r = 1 + (j > 4);  c = j - 4*(r - 1);
    axT{j} = axes('Position', [col(c) 0.36 - 0.30*(r - 1) wid 0.21]);
    [axT{j}, lB{j}, lS{j}] = telemetryAxes(axT{j}, names{j}, cB, cS);
end
set(axT{7}, 'YScale', 'log');
set(axT{6}, 'YLim', [0 180], 'YTick', 0:45:180);
set(axT{5}, 'YLim', [0 110]);

%% STEP 4 - Play
for k = 1:numel(tf)
    if ~ishghandle(fig), return; end                 % window closed: stop quietly
    tk = tf(k);

    % Trajectory overview: trails, and limits that grow with the flight
    set(hTrB, 'XData', b.x(1:k)/1e3, 'YData', b.h(1:k)/1e3);
    set(hTrS, 'XData', sx(1:k)/1e3,  'YData', sh(1:k)/1e3);
    set(hDtB, 'XData', b.x(k)/1e3,   'YData', b.h(k)/1e3);
    set(hDtS, 'XData', sx(k)/1e3,    'YData', sh(k)/1e3);
    xs = [b.x(1:k), sx(1:k)]/1e3;  hs = [b.h(1:k), sh(1:k)]/1e3;
    xr = [min(xs), max(xs)];  pad = max(5, 0.05*diff(xr));
    axis(axO, [xr(1) - pad, xr(2) + pad, 0, max(10, 1.15*max(hs))]);

    % Earth view
    set(hEB, 'XData', toX(b.x(1:k), b.h(1:k)), 'YData', toY(b.x(1:k), b.h(1:k)));
    set(hES, 'XData', toX(sx(1:k), sh(1:k)),   'YData', toY(sx(1:k), sh(1:k)));
    set(hEdB, 'XData', toX(b.x(k), b.h(k)), 'YData', toY(b.x(k), b.h(k)));
    set(hEdS, 'XData', toX(sx(k), sh(k)),   'YData', toY(sx(k), sh(k)));

    % Clock, time warp and event log
    set(hClock, 'String', sprintf('T+%02d:%02d   (time x%d)', floor(tk/60), floor(mod(tk, 60)), round(speed*warp(k))));
    done = ev.t <= tk;
    set(hLog, 'String', ev.text(done(:).' & cumsum(done(:).') > nnz(done) - 7));

    % Cameras: both vehicles are drawn in both views
    for cam = 1:2
        if cam == 1, g = gB; ax = axB; cx = b.x(k); cy = b.h(k) + Pb.L/2*cos(b.th(k));
        else,        g = gS; ax = axS; cx = sx(k);  cy = sh(k)  + Ps.L/2*cos(s.th(k));
        end
        if tk <= tSep, cy = b.h(k) + (Pb.L + Ps.L)/2*cos(b.th(k)); end   % whole stack in view
        drawVehicle(g.booster, [b.x(k); b.h(k)], b.th(k), b.gim(k), b.thr(k), 45);
        drawVehicle(g.ship,    [sx(k); sh(k)],   s.th(k), s.gim(k), s.thr(k), 35);
        cy  = max(cy, 0.75*W);                     % keep the ground at the bottom when low
        sky = min(max(cy, 0)/80e3, 1)^0.5;
        set(ax, 'Color', (1 - sky)*[0.53 0.81 0.98] + sky*[0.02 0.02 0.08]);
        axis(ax, [cx - W, cx + W, cy - W, cy + W]);
        if cam == 1
            str = hudText('SUPER HEAVY', B.phaseList{b.ph(k)}, b, k, tk > tSep);
        else
            str = hudText('STARSHIP', Sh.phaseList{s.ph(k)}, s, k, tk > tSep);
        end
        if sky > 0.5, hc = 'w'; else, hc = 'k'; end
        set(g.hud, 'String', str, 'Color', hc);
    end

    % Live telemetry graphs
    tm = tf(1:k)/60;
    xl = [0, max(1, tf(k)/60*1.05)];
    for j = 1:nT
        set(lB{j}, 'XData', tm, 'YData', yB{j}(1:k));
        set(lS{j}, 'XData', tm, 'YData', yS{j}(1:k));
        xlim(axT{j}, xl);
    end

    drawnow
    if ~isempty(gifFile)
        exportgraphics(fig, gifFile, 'Append', k > 1);
    end
end
end


%% =====================================================================
%  HELPERS
%  =====================================================================
function v = sampleFlight(out, tf, P)
% Interpolate one flight onto the frame times (holding the last value
% after the vehicle has stopped), plus angle of attack and heating.
[tu, iu] = unique(out.t, 'last');
tq = min(max(tf, tu(1)), tu(end));
I  = @(y) interp1(tu, y(iu), tq);
v.x   = I(out.X(:,1));  v.h  = I(out.X(:,2));
v.v   = I(hypot(out.X(:,3), out.X(:,4)));
v.m   = I(out.X(:,5));  v.th = I(out.X(:,6));
v.thr = I(out.aux.throttle);  v.gim = I(out.aux.gimbal);  v.gL = I(out.aux.gLoad);
v.ph  = interp1(tu, out.phase(iu), tq, 'previous');
v.q   = I(out.aux.q);
vx = I(out.X(:,3));  vh = I(out.X(:,4));
% angle between the nose and the direction of travel (0 = nose first,
% 90 = flat/belly first, 180 = engines first)
v.aoa = acosd(max(-1, min(1, (sin(v.th).*vx + cos(v.th).*vh) ./ max(v.v, 1e-6))));
v.aoa(v.v < 1) = 0;
v.heat = heatFlux(v.h, v.v, P.noseRadius, P);
end

function ev = missionEvents(B, Sh, tSep)
% Times and labels of the key moments, for the on-screen log.
t = 0;  txt = {'LIFTOFF'};
[~, iq] = max(B.aux.q .* (B.t < tSep));
t(end+1) = B.t(iq);  txt{end+1} = 'MAX-Q';
t(end+1) = tSep;     txt{end+1} = 'HOT STAGING';
names = {'boostback', 'BOOSTBACK BURN'; 'brake', 'BOOSTER LANDING BURN'};
for j = 1:size(names, 1)
    i = find(strcmp(B.phaseList, names{j,1}), 1);
    if ~isempty(i) && ~isnan(B.phaseStart(i))
        t(end+1) = B.phaseStart(i);  txt{end+1} = names{j,2};      %#ok<AGROW>
    end
end
Mb = flightMetrics(B, false);
t(end+1) = B.t(end);
if Mb.success, txt{end+1} = 'TOWER CATCH'; else, txt{end+1} = 'BOOSTER LOST'; end
i = find(strcmp(Sh.phaseList, 'bellyflop'), 1);
t(end+1) = Sh.phaseStart(i);  txt{end+1} = 'SHIP ENGINE CUT-OFF';
iE = find(Sh.t > Sh.phaseStart(i) & Sh.X(:,2) < 100e3 & Sh.X(:,4) < 0, 1);
if ~isempty(iE), t(end+1) = Sh.t(iE); txt{end+1} = 'ENTRY (100 km)'; end
i = find(strcmp(Sh.phaseList, 'landing'), 1);
if ~isnan(Sh.phaseStart(i)), t(end+1) = Sh.phaseStart(i); txt{end+1} = 'FLIP + LANDING BURN'; end
Ms = flightMetrics(Sh, false);
t(end+1) = Sh.t(end);
if Ms.success, txt{end+1} = 'SPLASHDOWN'; else, txt{end+1} = 'SHIP LOST'; end
[t, o] = sort(t);
ev.t = t;
ev.text = cellfun(@(tt, s) sprintf('T+%02d:%02d %s', floor(tt/60), floor(mod(tt, 60)), s), ...
                  num2cell(t), txt(o), 'UniformOutput', false);
end

function g = buildCamera(ax, Pb, Ps, xPad, ttl)
% Ground, sea, launch tower and the two vehicles' shapes for one camera.
hold(ax, 'on'); axis(ax, 'equal');
patch(ax, [-1e7 3e5 3e5 -1e7], [-1e3 -1e3 0 0], [0.35 0.55 0.25], 'EdgeColor', 'none');   % land
patch(ax, [3e5 1e8 1e8 3e5],   [-1e3 -1e3 0 0], [0.10 0.30 0.60], 'EdgeColor', 'none');   % ocean
plot(ax, xPad + [-12 12], [0.5 0.5], 'Color', [0.2 0.2 0.2], 'LineWidth', 6);               % launch mount
hArms = Pb.hLand + 0.9*Pb.L;                                                                 % catch arms
xT = xPad + Pb.diam/2 + 12;
patch(ax, xT + [0 8 8 0], [0 0 hArms + 25 hArms + 25], [0.3 0.3 0.33], 'EdgeColor', 'none');
plot(ax, [xPad - Pb.diam/2 - 3, xT], [hArms hArms], 'Color', [0.2 0.2 0.22], 'LineWidth', 4);
g.booster = vehicleShapes(ax, 'booster', Pb);
g.ship    = vehicleShapes(ax, 'ship', Ps);
g.hud = text(ax, 0.02, 0.98, '', 'Units', 'normalized', 'FontName', 'Monospaced', ...
             'FontSize', 8, 'VerticalAlignment', 'top');
xlabel(ax, 'Downrange [m]'); ylabel(ax, 'Altitude [m]'); title(ax, ttl);
end

function v = vehicleShapes(ax, kind, P)
% Outline polygons in body coordinates: bx = sideways, by = from the base
% toward the nose. Each polygon gets its own patch.
L = P.L; d = P.diam; r = d/2;
steel = [0.80 0.82 0.85]; dark = [0.22 0.22 0.25];
switch kind
    case 'booster'   % Super Heavy: steel body, hot-staging ring, 4 grid fins (2 visible)
        polys = {[-r r r -r; 0 0 L-3 L-3],               steel
                 [-r r r -r; L-3 L-3 L L],               dark      % vented hot-staging ring
                 [r r+3.5 r+3.5 r; L-11 L-11 L-6 L-6],   dark      % grid fin
                 [-r -r-3.5 -r-3.5 -r; L-11 L-11 L-6 L-6], dark
                 [-0.85*r 0.85*r 0.6*r -0.6*r; 0 0 -2.5 -2.5], dark};  % engine skirt
    case 'ship'      % Starship: steel body, black heat shield, ogive nose, 4 flaps
        polys = {[-r r r 0.85*r 0.5*r 0 -0.5*r -0.85*r -r; ...
                   0 0 0.78*L 0.87*L 0.95*L L 0.95*L 0.87*L 0.78*L], steel
                 [r-1.3 r r r-1.3; 0 0 0.78*L 0.78*L],           dark   % heat-shield tiles (belly, faces the air)
                 [r r+3 r+3 r; 1 1 11 14],                       dark   % aft flaps
                 [-r -r-3 -r-3 -r; 1 1 11 14],                   dark
                 [0.8*r 0.8*r+2.2 0.8*r+2.2 0.85*r; 0.80*L 0.80*L 0.87*L 0.90*L], dark  % forward flaps
                 [-0.8*r -0.8*r-2.2 -0.8*r-2.2 -0.85*r; 0.80*L 0.80*L 0.87*L 0.90*L], dark};
end
v.flame = patch(ax, nan, nan, [1 0.55 0], 'EdgeColor', [1 0.9 0.2]);
v.poly  = polys(:,1);
v.h     = cell(size(polys, 1), 1);
for j = 1:size(polys, 1)
    v.h{j} = patch(ax, nan, nan, polys{j,2}, 'EdgeColor', [0.15 0.15 0.15]);
end
v.w = d;
end

function drawVehicle(v, pos, th, gim, thr, flameLen)
% Place a vehicle's polygons at position pos with pitch th, plus its flame.
n = [sin(th); cos(th)];  sd = [n(2); -n(1)];
for j = 1:numel(v.h)
    p = v.poly{j};
    R = pos + sd*p(1,:) + n*p(2,:);
    set(v.h{j}, 'XData', R(1,:), 'YData', R(2,:));
end
fl = flameLen * thr * (1 + 0.15*rand);
nf = [sin(th - gim); cos(th - gim)];  sf = [nf(2); -nf(1)];
F  = pos + sf*[-0.4*v.w 0 0.4*v.w] + nf*[-2 -2 - fl -2];
if thr > 0.01, vis = 'on'; else, vis = 'off'; end
set(v.flame, 'XData', F(1,:), 'YData', F(2,:), 'Visible', vis);
end

function [ax, lB, lS] = telemetryAxes(ax, ylab, cB, cS)
hold(ax, 'on'); grid(ax, 'on');
lB = plot(ax, nan, nan, '-', 'Color', cB, 'LineWidth', 1.6);
lS = plot(ax, nan, nan, '-', 'Color', cS, 'LineWidth', 1.6);
xlabel(ax, 'Time [min]'); title(ax, ylab);
set(ax, 'FontSize', 8);
end

function str = hudText(name, phase, v, k, separated)
if ~separated, phase = 'stack ascent (attached)'; end
thr = v.thr(k);  aoa = v.aoa(k);
if isnan(thr), thr = 0; end                        % after the catch / splashdown
if isnan(aoa), aoa = 0; end
str = sprintf(['%s\nPhase : %s\nAlt   : %8.2f km\nSpeed : %8.0f m/s\n' ...
               'Throt : %6.0f %%\nPitch : %6.1f deg\nAoA   : %6.1f deg\nMass  : %8.0f t'], ...
              name, phase, v.h(k)/1e3, v.v(k), 100*thr, ...
              rad2deg(atan2(sin(v.th(k)), cos(v.th(k)))), aoa, v.m(k)/1e3);
end
