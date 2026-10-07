function animateFlight(out, speedup, fps, gifFile)
%ANIMATEFLIGHT  Two-panel animation of a simulateBooster result.
%
%   animateFlight(out)                       10x real time, 25 fps
%   animateFlight(out, speedup, fps)         custom playback
%   animateFlight(out, speedup, fps, 'x.gif') also save a GIF (MATLAB R2022a+)
%
%   Left : whole trajectory with a moving marker.
%   Right: a camera that follows the booster. The body is drawn at its TRUE
%          pitch angle, the flame points along the gimballed thrust, and the
%          sky darkens as the booster climbs.

if nargin < 2 || isempty(speedup), speedup = 10;  end
if nargin < 3 || isempty(fps),     fps     = 25;  end
if nargin < 4,                     gifFile = '';  end
P = out.P;

%% STEP 1 - Resample the simulation onto evenly spaced animation frames
[tu, iu] = unique(out.t, 'last');                % ode45 repeats phase-boundary times
tf = [0:(speedup/fps):(tu(end) - tu(1)), tu(end) - tu(1)] + tu(1);
I  = @(y) interp1(tu, y(iu), tf);
xa = I(out.X(:,1)); ha = I(out.X(:,2)); va = I(hypot(out.X(:,3), out.X(:,4)));
ma = I(out.X(:,5)); tha = I(out.X(:,6));
thr = I(out.aux.throttle); gim = I(out.aux.gimbal);
pa  = interp1(tu, out.phase(iu), tf, 'previous');

%% STEP 2 - Build the figure
fig = figure('Name', 'Flight animation', 'Color', 'w', 'Position', [80 80 1150 620]);

% Left panel: trajectory overview
ax1 = subplot(1, 2, 1); hold(ax1, 'on'); grid(ax1, 'on');
plot(ax1, out.X(:,1)/1e3, out.X(:,2)/1e3, 'Color', [0.75 0.75 0.75]);
yline(ax1, 100, '--k', 'Karman line (space)');
plot(ax1, P.xPad/1e3, 0, 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
hTrail = plot(ax1, nan, nan, 'b', 'LineWidth', 2);
hDot   = plot(ax1, nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
xlabel(ax1, 'Downrange [km]'); ylabel(ax1, 'Altitude [km]'); title(ax1, 'Trajectory');
xr = [min(out.X(:,1)), max(out.X(:,1))]/1e3;
xlim(ax1, xr + [-1 1]*max(2, 0.05*diff(xr)));
ylim(ax1, [0, max(out.X(:,2))/1e3*1.1]);

% Right panel: chase camera
ax2 = subplot(1, 2, 2); hold(ax2, 'on'); axis(ax2, 'equal');
W = 150;                                           % half-width of the view [m]
patch(ax2, [-1e7 1e7 1e7 -1e7], [-1e3 -1e3 0 0], [0.35 0.55 0.25], 'EdgeColor', 'none');
plot(ax2, [-15 15], [0.3 0.3], 'k', 'LineWidth', 5);                       % launch pad
plot(ax2, P.xPad + [-15 15], [0.3 0.3], 'Color', [1 0.8 0], 'LineWidth', 5); % landing pad
hFlame  = patch(ax2, nan, nan, [1 0.55 0], 'EdgeColor', [1 0.9 0.2]);
hRocket = patch(ax2, nan, nan, [0.93 0.93 0.96], 'EdgeColor', 'k');
hHud = text(ax2, 0.02, 0.98, '', 'Units', 'normalized', 'FontName', 'Monospaced', ...
            'FontSize', 9, 'VerticalAlignment', 'top');
xlabel(ax2, 'Downrange [m]'); ylabel(ax2, 'Altitude [m]'); title(ax2, 'Chase camera');

% Rocket outline in body coordinates: bx = sideways, by = along the nose
L = P.L; w = P.diam;
bodyX = [-w/2 -w/2 -w/4 0 w/4 w/2 w/2  w*0.9 -w*0.9];
bodyY = [ 2    L*0.85 L*0.95 L L*0.95 L*0.85 2 0 0];

%% STEP 3 - Draw every frame
for k = 1:numel(tf)
    % Body axes in the world: nose direction n, sideways direction s
    n = [sin(tha(k)); cos(tha(k))];
    s = [n(2); -n(1)];
    pos = [xa(k); ha(k)];
    toWorld = @(bx, by) pos + s*bx + n*by;

    R = toWorld(bodyX, bodyY);
    set(hRocket, 'XData', R(1,:), 'YData', R(2,:));

    % Flame: along the thrust line (body axis rotated by the gimbal angle)
    fl = 30*thr(k)*(1 + 0.15*rand);
    nf = [sin(tha(k) - gim(k)); cos(tha(k) - gim(k))];
    sf = [nf(2); -nf(1)];
    F  = pos + sf*[-w*0.4 0 w*0.4] + nf*[0 -fl 0];
    set(hFlame, 'XData', F(1,:), 'YData', F(2,:), 'Visible', thr(k) > 0.01);

    set(hTrail, 'XData', xa(1:k)/1e3, 'YData', ha(1:k)/1e3);
    set(hDot,   'XData', xa(k)/1e3,   'YData', ha(k)/1e3);

    % Sky color: blue near the ground, black in space
    sky = min(ha(k)/80e3, 1)^0.5;
    ax2.Color = (1 - sky)*[0.53 0.81 0.98] + sky*[0.02 0.02 0.08];
    yLo = max(ha(k) - 0.6*W, -0.2*W);
    axis(ax2, [xa(k) - W, xa(k) + W, yLo, yLo + 2*W]);

    if sky > 0.5, hHud.Color = 'w'; else, hHud.Color = 'k'; end
    hHud.String = sprintf(['T+%6.1f s\nPhase : %s\nAlt   : %8.2f km\nSpeed : %8.0f m/s\n' ...
                           'Throt : %6.0f %%\nPitch : %6.1f deg\nGimbal: %6.1f deg\nProp  : %8.0f kg'], ...
        tf(k), out.phaseList{pa(k)}, ha(k)/1e3, va(k), 100*thr(k), rad2deg(tha(k)), ...
        rad2deg(gim(k)), ma(k) - P.m_dry*P.dryScale);
    drawnow

    if ~isempty(gifFile)
        exportgraphics(fig, gifFile, 'Append', k > 1);
    end
end

M = flightMetrics(out, false);
if M.success
    hHud.String = [hHud.String newline 'TOUCHDOWN - booster landed'];
else
    hHud.String = [hHud.String newline 'LANDING FAILED'];
end
end
