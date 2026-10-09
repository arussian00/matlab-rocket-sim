%% ROCKET_STARSHIP_SIM  Starship-style flight test: tower catch + belly-flop splashdown
%
% Mission (modelled on SpaceX's Starship flight tests):
%   1. Super Heavy + Starship lift off on 33 Raptor engines (~5,000 t)
%   2. HOT STAGING - the ship lights its six engines while still attached,
%      so there is no coast gap between the stages
%   3. From here we follow TWO vehicles:
%        a) Super Heavy flips, burns back toward the launch site (boostback),
%           steers with its grid fins and is CAUGHT by the tower arms
%           ("chopsticks") while hovering above the pad
%        b) Starship burns to just short of orbit (perigee below the surface),
%           coasts about half a lap of the Earth, re-enters BELLY-FIRST,
%           then flips upright and does a landing burn to a soft splashdown
%
% Both returns re-use simulateBooster.m (same physics, attitude control,
% grid fins and landing guidance as the hop). The stack and the ship's
% ascent are point masses with ideal steering, like rocket_two_stage_sim.
%
% Numbers are rounded public estimates, not official SpaceX data.
% Runs on MATLAB Online Basic (core MATLAB only).

clear; close all; clc
octaveCompat();          % also runs in GNU Octave (no-op in MATLAB)

%% STEP 1 - Parameters
Pb = rocket_params('superheavy');  % booster return (also planet + atmosphere)
Ps = rocket_params('starship');    % ship re-entry
mu = Pb.g0 * Pb.Re^2;              % Earth's gravitational parameter [m^3/s^2]

S.m1dry   = Pb.m_dry;              % Super Heavy empty mass [kg]
S.m1prop  = Pb.m_prop;             % Super Heavy propellant [kg]
S.T1      = Pb.engAscent * Pb.T_eng;   % 33 Raptors [N]
S.Isp1sl  = Pb.Isp_sl;
S.Isp1vac = Pb.Isp_vac;
S.reserve1= 400e3;     % propellant Super Heavy keeps for boostback + catch [kg]

S.m2dry   = Ps.m_dry;  % ship empty mass [kg]
S.m2prop  = Ps.m_prop; % ship propellant [kg]
S.T2      = 3*2.3e6 + 3*2.5e6;   % 3 sea-level + 3 vacuum Raptors [N]
S.Isp2    = 365;       % average of sea-level and vacuum Raptors [s]
S.reserve2= 30e3;      % landing propellant kept in the header tanks [kg]
S.mPay    = 10e3;      % test payload (e.g. Starlink mass simulators) [kg]
S.gLimit  = 4;         % ship throttles down to keep acceleration below this [g]

S.Cd      = 0.4;       % drag coefficient of the stack (subsonic; rises near Mach 1)
S.thrBucket = 0.75;    % MAX-Q THROTTLE BUCKET: 33 engines throttle to 75%
S.bucketMach = [0.6 0.8 1.5 1.8];   % ... ramping down from Mach 0.6-0.8, back up 1.5-1.8
S.A       = Pb.A;

S.tKick    = 8;        % start of pitch kick [s]
S.kickDur  = 5;        % pitch kick lasts this long [s]
S.kickDeg  = 0.8;      % pitch kick angle [deg]; then gravity turn

S.hCoast   = 190e3;    % ship holds this altitude while it builds speed [m]
S.perigee  = -50e3;    % cut-off when the orbit's perigee reaches this [m]
                       % (below the surface: re-enters by itself, like the test flights)
S.wnG      = 0.01;     % ship altitude-hold guidance frequency [rad/s]
S.zG       = 0.8;      % ship guidance damping

m0 = S.m1dry + S.m1prop + S.m2dry + S.m2prop + S.mPay;
fprintf('Liftoff mass %.0f t, thrust %.1f MN, thrust-to-weight %.2f\n', ...
        m0/1e3, S.T1/1e6, S.T1/(m0*Pb.g0));

%% STEP 2 - Ascent on 33 engines (whole stack)
% State for the point-mass model: Y = [x; h; vx; vh; m]
Y0 = [0; 0; 0; 0; m0];
mMECO = m0 - (S.m1prop - S.reserve1);
opts = odeset('Events', @(t,Y) evMECO(t, Y, mMECO), 'RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[tA, YA] = ode45(@(t,Y) starshipStack(t, Y, S, Pb, mu, 'stack'), [0 400], Y0, opts);
tSep = tA(end);  Ysep = YA(end,:).';
fprintf('Hot staging at t = %.1f s, h = %.1f km, speed = %.0f m/s, flight-path angle = %.1f deg\n', ...
        tSep, Ysep(2)/1e3, hypot(Ysep(3), Ysep(4)), atan2d(Ysep(4), Ysep(3)));

%% STEP 3 - Hot staging: the ship lights up and pushes away. No coast gap.
mShip  = S.m2dry + S.m2prop + S.mPay;
mBoost = Ysep(5) - mShip;                     % booster with its return propellant

%% STEP 4 - Ship burns to just short of orbit
opts = odeset('Events', @(t,Y) perigeeSECO(t, Y, S, Pb, mu), 'RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[tC, YC] = ode45(@(t,Y) starshipStack(t, Y, S, Pb, mu, 'ship'), [tSep, tSep + 1500], ...
                 [Ysep(1:4); mShip], opts);
tSECO = tC(end); Yseco = YC(end,:).';
[perigee, apogee] = orbitOf(Yseco, Pb.Re, mu);

%% STEP 5 - Super Heavy: flip, boostback, grid fins, tower catch
Xb0 = [Ysep(1:4); mBoost; atan2(Ysep(3), Ysep(4)); Ysep(3)/(Pb.Re + Ysep(2))];
outB = simulateBooster(Pb, Xb0, tSep, Pb.phaseList);

%% STEP 6 - Starship: coast, belly-flop re-entry, flip, landing burn
Ps.m_dry = S.m2dry + S.mPay;              % payload stays on board (test flight)
Xs0 = [Yseco(1:4); Yseco(5); atan2(Yseco(3), Yseco(4)); Yseco(3)/(Pb.Re + Yseco(2))];
outS = simulateBooster(Ps, Xs0, tSECO, Ps.phaseList);

%% STEP 7 - Stitch each vehicle's complete flight together, from the launch pad
% Until hot staging both vehicles ARE the stack, so each history starts with
% the stack ascent. The ship's also includes its burn to (almost) orbit.
segA = pointMassSegment(tA, YA, Pb, @(t,Y) starshipStack(t, Y, S, Pb, mu, 'stack'), S.T1);
segC = pointMassSegment(tC, YC, Pb, @(t,Y) starshipStack(t, Y, S, Pb, mu, 'ship'),  S.T2);
fullB = fullFlight({segA}, {'stack ascent'}, outB);
fullS = fullFlight({segA, segC}, {'stack ascent', 'ship burn'}, outS);

%% STEP 8 - Print results
fprintf('\n================ STARSHIP ASCENT ================\n');
fprintf('Engine cut-off (SECO) : t = %.1f s, h = %.1f km, speed = %.0f m/s\n', ...
        tSECO, Yseco(2)/1e3, hypot(Yseco(3), Yseco(4)));
fprintf('Trajectory            : %.0f x %.0f km (perigee below ground = re-enters)\n', ...
        perigee/1e3, apogee/1e3);
fprintf('Ship propellant left  : %.1f t (landing reserve %.0f t)\n', ...
        (Yseco(5) - S.m2dry - S.mPay)/1e3, S.reserve2/1e3);
flightMetrics(fullB);
flightMetrics(fullS);
Mr = flightMetrics(outS, false);            % re-entry only (after engine cut-off)
fprintf('Ship re-entry: max %.1f g, max q %.1f kPa, %.0f m/s at the flip, splashdown %.0f km downrange\n', ...
        Mr.maxG, Mr.maxQ/1e3, Mr.landIgnSpeed, outS.X(end,1)/1e3);
qS = heatFlux(outS.X(:,2), hypot(outS.X(:,3), outS.X(:,4)), Ps.noseRadius, Ps);
qB = heatFlux(outB.X(:,2), hypot(outB.X(:,3), outB.X(:,4)), Pb.noseRadius, Pb);
[qSmax, iq] = max(qS);
fprintf('Peak heating (estimate): ship %.0f kW/m^2 at %.0f km, booster %.0f kW/m^2\n', ...
        qSmax/1e3, outS.X(iq,2)/1e3, max(qB)/1e3);
fprintf('Flight time to splashdown: %.1f min\n\n', outS.t(end)/60);

%% STEP 9 - Live mission animation, from the launch pad
% Both vehicles, two chase cameras, and telemetry graphs that grow as the
% flight plays. Time runs faster during the long quiet coast (shown on screen).
playAnimation = true;    % set false to skip
speed         = 1;       % 2 = twice as fast, 0.5 = slower
if playAnimation
    animateStarship(fullB, fullS, tSep, speed);
end

%% STEP 10 - Summary plots (drawn after the animation)
% 10a. Earth view (Earth-centered coordinates, km)
toXY = @(x, h) deal((Pb.Re + h).*sin(x/Pb.Re)/1e3, (Pb.Re + h).*cos(x/Pb.Re)/1e3);
[xA, yA] = toXY(YA(:,1), YA(:,2));
[xC, yC] = toXY(YC(:,1), YC(:,2));
[xS, yS] = toXY(outS.X(:,1), outS.X(:,2));
[xB, yB] = toXY(outB.X(:,1), outB.X(:,2));

figure('Name', 'Starship flight test', 'Color', 'w', 'Position', [40 40 1300 600]);
subplot(1, 2, 1); hold on; axis equal
th = linspace(0, 2*pi, 400);
fill(Pb.Re/1e3*cos(th), Pb.Re/1e3*sin(th), [0.75 0.85 1], 'EdgeColor', [0.3 0.5 0.8]);
plot(xC, yC, 'b', 'LineWidth', 1.5);
plot(xS, yS, 'm', 'LineWidth', 1.5);
plot(xA, yA, 'k', 'LineWidth', 2);
plot(xB, yB, 'r', 'LineWidth', 1.5);
title('Whole flight (Earth view)'); xlabel('km'); ylabel('km');
legend('Earth', 'Ship ascent', 'Ship coast + re-entry', 'Stack ascent', 'Super Heavy', ...
       'Location', 'southoutside');

subplot(1, 2, 2); hold on; grid on
plot(YA(:,1)/1e3, YA(:,2)/1e3, 'k', 'LineWidth', 2);
plot(YC(:,1)/1e3, YC(:,2)/1e3, 'b', 'LineWidth', 1.5);
plot(outB.X(:,1)/1e3, outB.X(:,2)/1e3, 'r', 'LineWidth', 1.5);
plot(0, 0, 'ks', 'MarkerFaceColor', [1 0.8 0], 'MarkerSize', 10);
yline(100, '--k', 'Karman line');
xlim([min(outB.X(:,1))/1e3 - 20, 1500]);
xlabel('Downrange [km]'); ylabel('Altitude [km]');
title('Ascent and booster return (flat view)');
legend('Stack ascent', 'Ship ascent', 'Super Heavy return', 'Launch tower', 'Location', 'best');

% 10b. Altitude and speed of both vehicles vs time
figure('Name', 'Starship time histories', 'Color', 'w', 'Position', [60 60 1100 450]);
subplot(1, 2, 1); hold on; grid on
plot(tA/60, YA(:,2)/1e3, 'k', 'LineWidth', 2);
plot([tC; outS.t]/60, [YC(:,2); outS.X(:,2)]/1e3, 'b', 'LineWidth', 1.5);
plot(outB.t/60, outB.X(:,2)/1e3, 'r', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('Altitude [km]'); title('Altitude');
legend('Stack', 'Starship', 'Super Heavy', 'Location', 'best');
subplot(1, 2, 2); hold on; grid on
plot(tA/60, hypot(YA(:,3), YA(:,4)), 'k', 'LineWidth', 2);
plot([tC; outS.t]/60, hypot([YC(:,3); outS.X(:,3)], [YC(:,4); outS.X(:,4)]), 'b', 'LineWidth', 1.5);
plot(outB.t/60, hypot(outB.X(:,3), outB.X(:,4)), 'r', 'LineWidth', 1.5);
yline(sqrt(mu/(Pb.Re + S.hCoast)), '--', 'orbital speed');
xlabel('Time [min]'); ylabel('Speed [m/s]'); title('Speed');

% 10c. Full 9-panel dashboards, liftoff to touchdown
plotFlight(fullB);
plotFlight(fullS);
