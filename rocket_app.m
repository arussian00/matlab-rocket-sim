function rocket_app()
%ROCKET_APP  Interactive launcher: move sliders, press LAUNCH, see the flight.
%
%   Type  rocket_app  in the Command Window.
%   Built with uifigure (works in MATLAB Online, no App Designer file needed).
%
%   Each slider overrides one field of rocket_params('hop'). Pressing LAUNCH
%   runs simulateBooster with those values and refreshes the four plots and
%   the summary. ANIMATE plays the last flight in a separate window.

%% STEP 1 - Window and layout
fig  = uifigure('Name', 'Rocket Hop Simulator', 'Position', [60 60 1250 720]);
main = uigridlayout(fig, [1 2]);
main.ColumnWidth = {320, '1x'};

% Left column: controls
ctl = uigridlayout(main, [20 1]);
ctl.RowHeight = repmat({'fit'}, 1, 20);
ctl.Scrollable = 'on';

% Right column: 2x2 plots + summary box
right = uigridlayout(main, [3 2]);
right.RowHeight = {'1x', '1x', 150};
axTraj = uiaxes(right);  title(axTraj, 'Trajectory');
axAlt  = uiaxes(right);  title(axAlt,  'Altitude');
axThr  = uiaxes(right);  title(axThr,  'Throttle & pitch');
axLand = uiaxes(right);  title(axLand, 'Final approach');
txt = uitextarea(right, 'Editable', 'off', 'FontName', 'Monospaced', ...
                 'Value', {'Set the sliders and press LAUNCH.'});
txt.Layout.Row = 3; txt.Layout.Column = [1 2];

%% STEP 2 - Sliders: {label, field, min, max, default, scale-to-SI}
P0 = rocket_params('hop');
spec = {
    'Target apogee [km]',      'h_target',    50,  200, P0.h_target/1e3, 1e3
    'Pitch kick [deg]',        'kickDeg',      0,   10, P0.kickDeg,      1
    'Engine thrust [kN]',      'T_eng',      300,  600, P0.T_eng/1e3,    1e3
    'Propellant [t]',          'm_prop',      12,   25, P0.m_prop/1e3,   1e3
    'Min throttle [%]',        'thrMin',      10,   60, P0.thrMin*100,   0.01
    'Wind at 10 km [m/s]',     'wind',       -25,   25, 0,               1
    'Drag error (x nominal)',  'CdScale',    0.7,  1.3, 1,               1
    'Thrust error [%]',        'thrustScale', -5,    5, 0,               NaN   % special
    };
nS = size(spec, 1);
sliders = gobjects(nS, 1);
valLbl  = gobjects(nS, 1);
for k = 1:nS
    row = uigridlayout(ctl, [1 2]); row.ColumnWidth = {'1x', 60}; row.Padding = [0 0 0 0];
    uilabel(row, 'Text', spec{k,1}, 'FontWeight', 'bold');
    valLbl(k) = uilabel(row, 'Text', sprintf('%.2f', spec{k,5}), 'HorizontalAlignment', 'right');
    sliders(k) = uislider(ctl, 'Limits', [spec{k,3} spec{k,4}], 'Value', spec{k,5});
    sliders(k).ValueChangingFcn = @(src, ev) set(valLbl(k), 'Text', sprintf('%.2f', ev.Value));
    sliders(k).ValueChangedFcn  = @(src, ev) set(valLbl(k), 'Text', sprintf('%.2f', src.Value));
end

%% STEP 3 - Buttons
btnRun  = uibutton(ctl, 'Text', 'LAUNCH',  'FontWeight', 'bold', 'FontSize', 16, ...
                   'BackgroundColor', [0.2 0.6 0.3], 'FontColor', 'w', ...
                   'ButtonPushedFcn', @(~,~) onLaunch());
uibutton(ctl, 'Text', 'Animate last flight', 'ButtonPushedFcn', @(~,~) onAnimate());
uibutton(ctl, 'Text', 'Full dashboard (9 plots)', 'ButtonPushedFcn', @(~,~) onDashboard());
uibutton(ctl, 'Text', 'Reset sliders', 'ButtonPushedFcn', @(~,~) onReset());

lastOut = [];   % shared with the nested callbacks below

%% =====================================================================
%  CALLBACKS (nested functions share the variables above)
%  =====================================================================
    function P = paramsFromSliders()
        % Start from the defaults and apply every slider.
        P = rocket_params('hop');
        for j = 1:nS
            field = spec{j,2}; val = sliders(j).Value; scale = spec{j,6};
            if strcmp(field, 'thrustScale')
                P.thrustScale = 1 + val/100;      % percent -> multiplier
            else
                P.(field) = val * scale;
            end
        end
    end

    function onLaunch()
        btnRun.Enable = 'off'; btnRun.Text = 'Flying...'; drawnow
        try
            P = paramsFromSliders();
            out = simulateBooster(P);
            M   = flightMetrics(out, false);
            lastOut = out;
            drawResults(out, M);
        catch err
            txt.Value = {['Error: ' err.message]};
        end
        btnRun.Enable = 'on'; btnRun.Text = 'LAUNCH';
    end

    function drawResults(out, M)
        X = out.X; t = out.t; P = out.P;
        C = lines(numel(out.phaseList));

        cla(axTraj); hold(axTraj, 'on'); grid(axTraj, 'on');
        for k2 = 1:numel(out.phaseList)
            idx = out.phase == k2;
            plot(axTraj, X(idx,1)/1e3, X(idx,2)/1e3, 'LineWidth', 2, 'Color', C(k2,:));
        end
        yline(axTraj, 100, '--k');
        plot(axTraj, P.xPad/1e3, 0, 'k^', 'MarkerFaceColor', 'y');
        xlabel(axTraj, 'Downrange [km]'); ylabel(axTraj, 'Altitude [km]');
        legend(axTraj, out.phaseList, 'Location', 'best', 'FontSize', 8);

        cla(axAlt); plot(axAlt, t, X(:,2)/1e3, 'LineWidth', 2); grid(axAlt, 'on');
        yline(axAlt, 100, '--k', 'space');
        xlabel(axAlt, 'Time [s]'); ylabel(axAlt, 'Altitude [km]');

        yyaxis(axThr, 'right'); cla(axThr);      % clear both y-axes from the last run
        yyaxis(axThr, 'left');  cla(axThr); grid(axThr, 'on');
        plot(axThr, t, 100*out.aux.throttle, 'LineWidth', 1.2);
        ylabel(axThr, 'Throttle [%]'); ylim(axThr, [0 110]);
        yyaxis(axThr, 'right'); plot(axThr, t, rad2deg(X(:,6)), 'LineWidth', 1.2);
        ylabel(axThr, 'Pitch [deg]'); xlabel(axThr, 'Time [s]');

        cla(axLand); hold(axLand, 'on'); grid(axLand, 'on');
        idx = X(:,2) < 3000 & t > t(end) - 120;
        plot(axLand, X(idx,1) - P.xPad, X(idx,2), 'LineWidth', 2);
        plot(axLand, 0, 0, 'k^', 'MarkerFaceColor', 'y');
        xlabel(axLand, 'From pad [m]'); ylabel(axLand, 'Altitude [m]');

        if M.success, verdict = 'SUCCESSFUL LANDING'; else, verdict = ['FAILED: ' out.status]; end
        txt.Value = {
            sprintf('RESULT: %s', verdict)
            sprintf('Apogee %.1f km | %.0f s above 100 km | max q %.1f kPa | max %.1f g', ...
                    M.apogee/1e3, M.timeInSpace, M.maxQ/1e3, M.maxG)
            sprintf('Touchdown: %.2f m/s down, %.2f m/s sideways, %.2f deg tilt, %.1f m from pad', ...
                    M.vVert, M.vHoriz, M.tiltDeg, M.miss)
            sprintf('Propellant left %.0f kg | landing burn lit at %.0f m', M.propLeft, M.landIgnAlt)
            strjoin(out.notes, ' | ')};
    end

    function onAnimate()
        if isempty(lastOut), uialert(fig, 'Press LAUNCH first.', 'No flight yet'); return; end
        animateFlight(lastOut, 10, 25);
    end

    function onDashboard()
        if isempty(lastOut), uialert(fig, 'Press LAUNCH first.', 'No flight yet'); return; end
        plotFlight(lastOut);
    end

    function onReset()
        for j = 1:nS
            sliders(j).Value = spec{j,5};
            valLbl(j).Text = sprintf('%.2f', spec{j,5});
        end
    end
end
