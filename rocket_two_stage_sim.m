%% ROCKET_TWO_STAGE_SIM  Two-stage rocket to orbit + booster lands on a drone ship
%
% Mission (a small Falcon-9-like reusable launcher):
%   1. Stage 1 + Stage 2 lift off together and pitch over (gravity turn)
%   2. Stage 1 cuts off (MECO), keeping fuel for its own landing
%   3. STAGE SEPARATION - from here we follow TWO vehicles:
%        a) Stage 2 burns into a ~250 km circular orbit, then coasts one orbit
%        b) Stage 1 (booster) flips, does an entry burn, steers with grid
%           fins, and lands on a drone ship waiting downrange
%
% The booster part re-uses simulateBooster.m (same physics, attitude
% control, grid fins and landing guidance as the hop).
% The stack and Stage 2 are modelled as point masses with ideal steering.
%
% Runs on MATLAB Online Basic (core MATLAB only).

clear; close all; clc

%% STEP 1 - Parameters
E = rocket_params('booster');     % reuse planet + atmosphere values
mu = E.g0 * E.Re^2;               % Earth's gravitational parameter [m^3/s^2]

S.m1dry   = 14000;    % stage 1 empty mass [kg]
S.m1prop  = 130000;   % stage 1 propellant [kg]
S.T1      = 2.6e6;    % stage 1 thrust, 9 engines [N]
S.Isp1sl  = 282;      % stage 1 Isp at sea level [s]
S.Isp1vac = 311;      % stage 1 Isp in vacuum [s]
S.reserve1= 14000;    % propellant stage 1 keeps for entry + landing [kg]

S.m2dry   = 3000;     % stage 2 empty mass [kg]
S.m2prop  = 38000;    % stage 2 propellant [kg]
S.T2      = 300e3;    % stage 2 thrust (vacuum engine) [N]
S.Isp2    = 348;      % stage 2 Isp [s]
S.mPay    = 1500;     % payload (satellite) [kg]

S.diam    = 3.7;      % body diameter [m]
S.Cd      = 0.4;      % drag coefficient of the stack
S.A       = pi*(S.diam/2)^2;

S.tKick    = 10;      % start of pitch kick [s]
S.kickDur  = 5;       % pitch kick lasts this long [s]
S.kickDeg  = 1.5;     % pitch kick angle [deg]; then gravity turn
S.sepCoast = 3;       % seconds between MECO and stage-2 ignition

S.hOrbit  = 250e3;    % target circular-orbit altitude [m]
S.wnG     = 0.01;     % stage-2 altitude-hold guidance frequency [rad/s]
S.zG      = 0.8;      % stage-2 guidance damping

m0 = S.m1dry + S.m1prop + S.m2dry + S.m2prop + S.mPay;
fprintf('Liftoff mass %.1f t, thrust-to-weight %.2f\n', m0/1e3, S.T1/(m0*E.g0));

%% STEP 2 - Stage 1 ascent (whole stack)
% State for the point-mass model: Y = [x; h; vx; vh; m]
Y0 = [0; 0; 0; 0; m0];
mMECO = m0 - (S.m1prop - S.reserve1);
opts = odeset('Events', @(t,Y) evMECO(t, Y, mMECO), 'RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[tA, YA] = ode45(@(t,Y) stackDynamics(t, Y, S, E, mu, 'stage1'), [0 400], Y0, opts);
tSep = tA(end);  Ysep = YA(end,:).';
fprintf('MECO at t = %.1f s, h = %.1f km, speed = %.0f m/s, flight-path angle = %.1f deg\n', ...
        tSep, Ysep(2)/1e3, hypot(Ysep(3), Ysep(4)), atan2d(Ysep(4), Ysep(3)));

%% STEP 3 - Separation: split the state into two vehicles
mUpper = S.m2dry + S.m2prop + S.mPay;
Yup    = [Ysep(1:4); mUpper];                 % stage 2 + payload
mBoost = Ysep(5) - mUpper;                    % booster with its reserve propellant

%% STEP 4 - Stage 2: short coast, then burn to orbit
[tB, YB] = ode45(@(t,Y) stackDynamics(t, Y, S, E, mu, 'coast'), ...
                 [tSep, tSep + S.sepCoast], Yup, odeset('RelTol', 1e-8, 'MaxStep', 0.5));
opts = odeset('Events', @(t,Y) evSECO(t, Y, S, E, mu), 'RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[tC, YC] = ode45(@(t,Y) stackDynamics(t, Y, S, E, mu, 'stage2'), [tB(end), tB(end) + 1500], YB(end,:).', opts);
tSECO = tC(end); Yseco = YC(end,:).';

% Orbit check from the cut-off state (classical two-body formulas)
r  = E.Re + Yseco(2);
v2 = Yseco(3)^2 + Yseco(4)^2;
energy = v2/2 - mu/r;                         % < 0 means we are bound to Earth
aOrb   = -mu/(2*energy);                      % semi-major axis
hMom   = r*Yseco(3);                          % angular momentum per kg
ecc    = sqrt(max(0, 1 - hMom^2/(mu*aOrb)));  % eccentricity (0 = circle)
perigee = aOrb*(1 - ecc) - E.Re;
apogee  = aOrb*(1 + ecc) - E.Re;
period  = 2*pi*sqrt(aOrb^3/mu);
inOrbit = energy < 0 && perigee > 150e3;

%% STEP 5 - Stage 2 coasts one full orbit (if it made it)
if inOrbit
    [tD, YD] = ode45(@(t,Y) stackDynamics(t, Y, S, E, mu, 'coast'), [tSECO, tSECO + period], ...
                     Yseco, odeset('RelTol', 1e-9, 'AbsTol', 1e-6, 'MaxStep', 10));
else
    tD = tSECO; YD = Yseco.';
end
tUp = [tB; tC; tD];  YUp = [YB; YC; YD];

%% STEP 6 - Booster return (re-uses the hop simulator)
Pb = rocket_params('booster');
Xb0 = [Ysep(1:4); mBoost; atan2(Ysep(3), Ysep(4)); Ysep(3)/(E.Re + Ysep(2))];
% pitch = flight-path direction; rotation rate = following the local vertical
outB = simulateBooster(Pb, Xb0, tSep, Pb.phaseList);

%% STEP 7 - Print results
fprintf('\n================ STAGE 2 / ORBIT ================\n');
fprintf('SECO at t = %.1f s, h = %.1f km, speed = %.0f m/s\n', tSECO, Yseco(2)/1e3, sqrt(v2));
fprintf('Orbit                 : %.1f x %.1f km, period %.1f min\n', perigee/1e3, apogee/1e3, period/60);
fprintf('Stage 2 prop left     : %.0f kg\n', Yseco(5) - S.m2dry - S.mPay);
if inOrbit, fprintf('RESULT                : PAYLOAD IN ORBIT\n');
else,       fprintf('RESULT                : ORBIT NOT REACHED\n'); end
flightMetrics(outB);

%% STEP 8 - Plots
% 8a. Earth view (Earth-centered coordinates, km)
toXY = @(x, h) deal((E.Re + h).*sin(x/E.Re)/1e3, (E.Re + h).*cos(x/E.Re)/1e3);
[xA, yA] = toXY(YA(:,1), YA(:,2));
[xU, yU] = toXY(YUp(:,1), YUp(:,2));
[xBo, yBo] = toXY(outB.X(:,1), outB.X(:,2));

figure('Name', 'Two-stage mission', 'Color', 'w', 'Position', [40 40 1300 600]);
subplot(1, 2, 1); hold on; axis equal
th = linspace(0, 2*pi, 400);
fill(E.Re/1e3*cos(th), E.Re/1e3*sin(th), [0.75 0.85 1], 'EdgeColor', [0.3 0.5 0.8]);
plot(xU, yU, 'b', 'LineWidth', 1.2);
plot(xA, yA, 'k', 'LineWidth', 2);
title('Stage 2: one full orbit'); xlabel('km'); ylabel('km');
legend('Earth', 'Stage 2', 'Stage 1 + 2 ascent', 'Location', 'southoutside');

subplot(1, 2, 2); hold on; grid on
plot(YA(:,1)/1e3, YA(:,2)/1e3, 'k', 'LineWidth', 2);
idx = tUp <= tSECO + 60;
plot(YUp(idx,1)/1e3, YUp(idx,2)/1e3, 'b', 'LineWidth', 1.5);
plot(outB.X(:,1)/1e3, outB.X(:,2)/1e3, 'r', 'LineWidth', 1.5);
plot(outB.P.xPad/1e3, 0, 'ks', 'MarkerFaceColor', [1 0.8 0], 'MarkerSize', 10);
yline(100, '--k', 'Karman line');
xlabel('Downrange [km]'); ylabel('Altitude [km]');
title('Ascent and booster return (flat view)');
legend('Stack ascent', 'Stage 2 to orbit', 'Booster return', 'Drone ship', 'Location', 'best');

% 8b. Altitude and speed of both vehicles vs time
figure('Name', 'Two-stage time histories', 'Color', 'w', 'Position', [60 60 1100 450]);
subplot(1, 2, 1); hold on; grid on
plot(tA, YA(:,2)/1e3, 'k', 'LineWidth', 2);
plot(tUp(idx), YUp(idx,2)/1e3, 'b', 'LineWidth', 1.5);
plot(outB.t, outB.X(:,2)/1e3, 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Altitude [km]'); title('Altitude');
legend('Stack', 'Stage 2', 'Booster', 'Location', 'best');
subplot(1, 2, 2); hold on; grid on
plot(tA, hypot(YA(:,3), YA(:,4)), 'k', 'LineWidth', 2);
plot(tUp(idx), hypot(YUp(idx,3), YUp(idx,4)), 'b', 'LineWidth', 1.5);
plot(outB.t, hypot(outB.X(:,3), outB.X(:,4)), 'r', 'LineWidth', 1.5);
yline(sqrt(mu/(E.Re + S.hOrbit)), '--', 'orbital speed');
xlabel('Time [s]'); ylabel('Speed [m/s]'); title('Speed');

% 8c. Full booster dashboard and animation
plotFlight(outB);
playBoosterAnimation = true;
if playBoosterAnimation
    animateFlight(outB, 10, 25);
end


%% =====================================================================
%  LOCAL FUNCTIONS
%  =====================================================================
function dY = stackDynamics(t, Y, S, E, mu, mode)
% Point-mass equations of motion on a round, non-rotating Earth.
% mode: 'stage1' (stack burning), 'stage2' (upper stage burning), 'coast'
x = Y(1); h = Y(2); vx = Y(3); vh = Y(4); m = Y(5); %#ok<NASGU>
r = E.Re + h;
g = mu / r^2;
[~, rho] = earthModel(h, E);
v = hypot(vx, vh);
D = 0.5*rho*v^2*S.Cd*S.A;

switch mode
    case 'stage1'
        % STEERING: straight up, short pitch kick, then GRAVITY TURN
        % (thrust along the velocity, so gravity gradually bends the path
        %  over without the rocket ever flying sideways through thick air)
        F   = S.T1;
        Isp = S.Isp1vac - (S.Isp1vac - S.Isp1sl)*rho/E.rho0;
        if t < S.tKick
            u = [0; 1];
        elseif t < S.tKick + S.kickDur
            u = [sind(S.kickDeg); cosd(S.kickDeg)];
        else
            u = [vx; vh]/v;
        end
    case 'stage2'
        % STEERING: split the available acceleration into a vertical part
        % (a PD controller that drives altitude to hOrbit and vertical speed
        % to zero) and use everything left over to build horizontal speed.
        F   = S.T2;
        Isp = S.Isp2;
        aT  = F/m;
        aUp = S.wnG^2*(S.hOrbit - h) - 2*S.zG*S.wnG*vh ...  % PD on altitude
              + g - vx^2/r;                                % cancel net gravity
        aUp = min(max(aUp, -aT), aT);
        u   = [sqrt(max(aT^2 - aUp^2, 0)); aUp] / aT;
    otherwise
        F = 0; Isp = 1; u = [0; 0];
end

if v > 0, aDrag = -D*[vx; vh]/(v*m); else, aDrag = [0; 0]; end
a  = F*u/m + aDrag;
dY = [vx*E.Re/r; vh; a(1) - vx*vh/r; a(2) - g + vx^2/r; -F/(Isp*E.g0)];
end

function [value, isterminal, direction] = evMECO(~, Y, mMECO)
% Stage 1 cut-off when only the landing reserve is left.
value = Y(5) - mMECO; isterminal = 1; direction = -1;
end

function [value, isterminal, direction] = evSECO(~, Y, S, E, mu)
% Stage 2 cut-off: (1) horizontal speed reaches circular-orbit speed,
%                  (2) or stage 2 runs dry.
r = E.Re + Y(2);
value      = [Y(3) - sqrt(mu/r);  Y(5) - (S.m2dry + S.mPay)];
isterminal = [1; 1];
direction  = [+1; -1];
end
