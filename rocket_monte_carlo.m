%% ROCKET_MONTE_CARLO  Fly the hop many times with random errors
%
% Real rockets never fly exactly as designed: engines are a bit stronger or
% weaker, drag is uncertain, the wind blows. A Monte Carlo analysis flies
% the mission many times, each with randomly drawn errors, and asks:
%   "How often do we land, and how close to the pad?"
%
% Important: the GUIDANCE never sees these errors. It still believes the
% nominal numbers, exactly like a real flight computer. Only the physics
% ("truth") uses the dispersed values.
%
% Runtime: roughly 1-3 s per flight in MATLAB Online. Start with N = 50.

clear; close all; clc

%% STEP 1 - Settings
N = 50;              % number of flights
rng(42);             % fixed random seed -> repeatable results

% 1-sigma (standard deviation) of each error source
sig.thrust = 0.02;   % 2% thrust error
sig.Isp    = 0.01;   % 1% engine efficiency error
sig.Cd     = 0.10;   % 10% drag-coefficient error
sig.dry    = 0.02;   % 2% dry-mass error
sig.wind   = 8;      % 8 m/s wind at 10 km

%% STEP 2 - Fly N missions
Pnom = rocket_params('hop');
R = struct('success', false(N,1), 'miss', zeros(N,1), 'vVert', zeros(N,1), ...
           'vHoriz', zeros(N,1), 'tilt', zeros(N,1), 'propLeft', zeros(N,1), ...
           'apogee', zeros(N,1), 'maxG', zeros(N,1));
D = struct('thrust', zeros(N,1), 'Isp', zeros(N,1), 'Cd', zeros(N,1), ...
           'dry', zeros(N,1), 'wind', zeros(N,1));
trajX = cell(N,1); trajH = cell(N,1);

tic
for i = 1:N
    % 2a. Draw random errors (randn = normal distribution, mean 0, std 1)
    P = Pnom;
    P.thrustScale = 1 + sig.thrust*randn;
    P.IspScale    = 1 + sig.Isp*randn;
    P.CdScale     = max(0.5, 1 + sig.Cd*randn);
    P.dryScale    = 1 + sig.dry*randn;
    P.wind        = sig.wind*randn;

    % 2b. Fly
    out = simulateBooster(P);
    M   = flightMetrics(out, false);

    % 2c. Record results
    R.success(i) = M.success;   R.miss(i)   = M.miss;
    R.vVert(i)   = M.vVert;     R.vHoriz(i) = M.vHoriz;
    R.tilt(i)    = M.tiltDeg;   R.propLeft(i) = M.propLeft;
    R.apogee(i)  = M.apogee;    R.maxG(i)   = M.maxG;
    D.thrust(i) = P.thrustScale; D.Isp(i) = P.IspScale; D.Cd(i) = P.CdScale;
    D.dry(i)    = P.dryScale;    D.wind(i) = P.wind;
    trajX{i} = out.X(:,1); trajH{i} = out.X(:,2);

    fprintf('Run %3d/%d  %-7s miss %6.2f m  vVert %5.2f  vHoriz %5.2f  tilt %5.2f deg\n', ...
            i, N, ternary(M.success, 'LANDED', 'FAILED'), M.miss, M.vVert, M.vHoriz, M.tiltDeg);
end
fprintf('Monte Carlo took %.0f s.\n', toc);

%% STEP 3 - Statistics
fprintf('\n============== MONTE CARLO RESULTS (%d flights) ==============\n', N);
fprintf('Success rate          : %5.1f %%\n', 100*mean(R.success));
fprintf('Miss distance  mean   : %6.2f m   std %5.2f m   worst %6.2f m\n', ...
        mean(R.miss), std(R.miss), max(abs(R.miss)));
fprintf('Vertical speed worst  : %6.2f m/s\n', max(abs(R.vVert)));
fprintf('Sideways speed worst  : %6.2f m/s\n', max(abs(R.vHoriz)));
fprintf('Tilt worst            : %6.2f deg\n', max(abs(R.tilt)));
fprintf('Propellant left min   : %6.0f kg\n', min(R.propLeft));
fprintf('Apogee range          : %6.1f - %6.1f km\n', min(R.apogee)/1e3, max(R.apogee)/1e3);
fprintf('==============================================================\n');

%% STEP 4 - Plots
ok = R.success;
figure('Name', 'Monte Carlo', 'Color', 'w', 'Position', [40 40 1300 780]);
tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Monte Carlo: %d flights, %.0f%% landed', N, 100*mean(ok)));

% 4a. All trajectories on top of each other
nexttile; hold on
for i = 1:N, plot(trajX{i}/1e3, trajH{i}/1e3, 'Color', [0.55 0.72 0.9]); end
yline(100, '--k'); xlabel('Downrange [km]'); ylabel('Altitude [km]');
title('All trajectories'); grid on

% 4b. Touchdown "footprint": miss distance vs sideways speed
nexttile; hold on
scatter(R.miss(ok),  R.vHoriz(ok),  30, 'g', 'filled');
scatter(R.miss(~ok), R.vHoriz(~ok), 40, 'r', 'x', 'LineWidth', 1.5);
S = Pnom.success;
rectangle('Position', [-S.missMax, -S.vHorizMax, 2*S.missMax, 2*S.vHorizMax], 'LineStyle', '--');
xlabel('Miss distance [m]'); ylabel('Sideways speed [m/s]');
title('Touchdown footprint (box = allowed)'); grid on

% 4c. Touchdown speed vs tilt
nexttile; hold on
scatter(R.vVert(ok),  R.tilt(ok),  30, 'g', 'filled');
scatter(R.vVert(~ok), R.tilt(~ok), 40, 'r', 'x', 'LineWidth', 1.5);
xline(S.vVertMax, '--'); yline(S.tiltMaxDeg, '--'); yline(-S.tiltMaxDeg, '--');
xlabel('Vertical speed at touchdown [m/s]'); ylabel('Tilt [deg]');
title('Touchdown speed vs tilt'); grid on

% 4d. Histogram of miss distance
nexttile; histogram(R.miss, 15); xlabel('Miss distance [m]'); ylabel('Flights');
title('Miss distance'); grid on

% 4e. Histogram of propellant left
nexttile; histogram(R.propLeft, 15); xlabel('Propellant left [kg]'); ylabel('Flights');
title('Propellant margin'); grid on

% 4f. Sensitivity: which error drives the miss? (wind vs miss)
nexttile; scatter(D.wind, R.miss, 30, D.Cd, 'filled'); cb = colorbar; cb.Label.String = 'Cd scale';
xlabel('Wind at 10 km [m/s]'); ylabel('Miss distance [m]');
title('Sensitivity: wind & drag'); grid on

%% Local helper
function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
