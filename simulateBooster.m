function out = simulateBooster(P, X0, t0, phaseList)
%SIMULATEBOOSTER  Fly a reusable booster through a list of flight phases.
%
%   out = simulateBooster(P)                    hop from the pad, P.phaseList
%   out = simulateBooster(P, X0, t0, phaseList) start from any state/time
%
%   INPUTS
%     P         parameter struct from rocket_params.m
%     X0        7x1 initial state (see STATE below). Default: on the pad, full tanks.
%     t0        start time [s]. Default 0.
%     phaseList cell array of phase names, flown in order. Any of:
%                 'ascent'    full thrust, pitch program, engine cut-off (MECO)
%                 'flip'      engine off, thrusters rotate the booster to point back
%                 'boostback' engine on, push back toward the landing pad
%                 'coast'     engine off, fall engines-first, grid fins steer
%                 'entry'     short burn to slow down before the thick air
%                 'landing'   landing burn ("hoverslam") to touchdown
%
%   OUTPUT  struct with fields
%     t, X            time vector and state history (one row per time)
%     phase           index into out.phaseList for every row
%     phaseList       the phase names flown
%     phaseStart      start time of every phase
%     aux             extra signals (throttle, gimbal, g-load, q, ...)
%     P               parameters, including values set during the flight
%                     (boostback direction, drone ship position)
%     crashed, status, notes
%
%   STATE  X = [x; h; vx; vh; m; theta; omega]
%     x      downrange distance along Earth's surface [m]
%     h      altitude [m]
%     vx     horizontal velocity (+ = downrange) [m/s]
%     vh     vertical velocity (+ = up) [m/s]
%     m      total mass [kg]
%     theta  pitch: angle of the nose from local vertical (+ = leaning downrange) [rad]
%     omega  pitch rate (inertial) [rad/s]

%% STEP 1 - Fill in defaults
if nargin < 2 || isempty(X0)
    X0 = [0; 0; 0; 0; P.m_dry*P.dryScale + P.m_prop; 0; 0];   % on the pad, upright
end
if nargin < 3 || isempty(t0),        t0 = 0;              end
if nargin < 4 || isempty(phaseList), phaseList = P.phaseList; end

T = []; X = []; PH = [];
phaseStart = nan(1, numel(phaseList));
crashed = false;
status  = 'completed';
notes   = {};

%% STEP 2 - Fly each phase in turn
for k = 1:numel(phaseList)
    name = phaseList{k};
    if k < numel(phaseList), nextName = phaseList{k+1}; else, nextName = ''; end
    phaseStart(k) = t0;

    % 2a. Phase set-up that needs the current state
    if strcmp(name, 'flip')
        % Which way do we need to push to get back to the pad?
        % -1 = thrust toward -x (back uprange), +1 = toward +x.
        xImpact = predictImpact(X0, P);
        P.bbDir = -sign(xImpact - P.xPad);
        if P.bbDir == 0, P.bbDir = -1; end
    end

    % 2b. Integrate the equations of motion until an event ends the phase.
    %     ode45 = adaptive Runge-Kutta solver built into core MATLAB.
    %     'Events' makes it stop exactly when a condition crosses zero.
    opts = odeset('Events',  @(t, Xs) phaseEvents(t, Xs, P, name, nextName), ...
                  'RelTol',  1e-7, 'AbsTol', 1e-6, 'MaxStep', P.maxStep);
    [tk, Xk, ~, ~, ie] = ode45(@(t, Xs) boosterDynamics(t, Xs, P, name), ...
                               [t0, t0 + P.tPhaseMax], X0, opts);

    % 2c. Store this phase's results
    T  = [T;  tk];                       %#ok<AGROW>
    X  = [X;  Xk];                       %#ok<AGROW>
    PH = [PH; k*ones(numel(tk), 1)];     %#ok<AGROW>
    X0 = Xk(end, :).';
    t0 = tk(end);

    % 2d. Decide what happens next based on WHICH event fired
    if isempty(ie)
        status = sprintf('phase "%s" timed out', name);
        crashed = true;
        break
    end
    switch name
        case 'ascent'
            if any(ie == 2)
                notes{end+1} = 'MECO on propellant reserve (target apogee not reached)'; %#ok<AGROW>
            end
        case 'boostback'
            if any(ie == 2)
                notes{end+1} = 'Boostback cut short: propellant at landing reserve'; %#ok<AGROW>
            end
        case 'coast'
            if any(ie == 2)                       % hit the ground with the engine off
                crashed = true;
                status  = 'hit the ground before the landing burn';
                break
            end
        case 'entry'
            if isnan(P.xPad)
                % Drone-ship mission: park the ship where the booster will fall.
                P.xPad = predictImpact(X0, P);
                notes{end+1} = sprintf('Drone ship positioned at %.2f km downrange', P.xPad/1e3); %#ok<AGROW>
            end
    end
end

%% STEP 3 - Recompute the extra signals (throttle, gimbal, ...) for plotting
N = numel(T);
aux = struct('throttle', zeros(N,1), 'thrust', zeros(N,1), 'gimbal', zeros(N,1), ...
             'gLoad', zeros(N,1), 'q', zeros(N,1), 'thetaCmd', zeros(N,1), ...
             'tauRCS', zeros(N,1), 'finForce', zeros(N,1), 'wind', zeros(N,1));
for i = 1:N
    [~, a] = boosterDynamics(T(i), X(i,:).', P, phaseList{PH(i)});
    aux.throttle(i) = a.throttle;   aux.thrust(i)   = a.F;
    aux.gimbal(i)   = a.delta;      aux.gLoad(i)    = a.gLoad;
    aux.q(i)        = a.q;          aux.thetaCmd(i) = a.thetaCmd;
    aux.tauRCS(i)   = a.tauRCS;     aux.finForce(i) = a.Ffin;
    aux.wind(i)     = a.wind;
end

out = struct('t', T, 'X', X, 'phase', PH, 'phaseList', {phaseList}, ...
             'phaseStart', phaseStart, 'aux', aux, 'P', P, ...
             'crashed', crashed, 'status', status, 'notes', {notes});
end


%% =====================================================================
%  EQUATIONS OF MOTION
%  =====================================================================
function [dX, aux] = boosterDynamics(t, X, P, phase)
% Returns dX/dt for the 7 states. Called thousands of times by ode45.

% --- unpack the state ---------------------------------------------------
x = X(1); h = X(2); vx = X(3); vh = X(4); m = X(5); theta = X(6); omega = X(7);
r = P.Re + h;                       % distance from Earth's center
[g, rho] = earthModel(h, P);
mDry = P.m_dry * P.dryScale;

% --- 1. GUIDANCE: where should we point, and how hard should we push? ----
[thetaCmd, thrReq] = guidanceLaw(t, X, P, phase);

% --- 2. ENGINE: real engines have limits -------------------------------
nEng = enginesLit(phase, P);
if nEng > 0 && m > mDry
    throttle = min(max(thrReq, P.thrMin), 1);   % clamp to [thrMin, 100%]
else
    throttle = 0;                                % engine off or tanks empty
end
F = throttle * nEng * P.T_eng * P.thrustScale;   % actual thrust [N]

% --- 3. ATTITUDE CONTROL: turn the body toward thetaCmd ------------------
[delta, tauG, tauR, I] = attitudeControl(X, P, thetaCmd, F);
thetaDot = omega - vx/r;           % pitch rate relative to the local vertical

% Thrust points along the body, minus the gimbal angle (the nozzle swivel).
u = [sin(theta - delta); cos(theta - delta)];

% --- 4. AERODYNAMICS: drag opposes motion relative to the air ------------
w   = windAt(h, P);
vRel = [vx - w; vh];               % velocity relative to the (moving) air
vr  = norm(vRel);
if vh >= 0, Cd = P.Cd_up; else, Cd = P.Cd_down; end
Cd  = Cd * P.CdScale;
q   = 0.5 * rho * vr^2;            % dynamic pressure [Pa]
D   = q * Cd * P.A;                % drag force [N]
if vr > 1e-6, dragDir = -vRel/vr; else, dragDir = [0; 0]; end

% --- 5. GRID FINS: steer sideways toward the pad while falling -----------
Ffin = 0;
if strcmp(phase, 'coast') && vh < 0 && h < P.finAlt && ~isnan(P.xPad)
    % Aim to be over the pad by finAimAlt (not at ground level), so the
    % booster has stopped sliding sideways before the landing burn starts.
    tFall = max((h - P.finAimAlt) / max(-vh, 1), 1);   % rough time to reach finAimAlt
    vxWanted = (P.xPad - x) / tFall;           % sideways speed that reaches the pad
    FfinMax = q * P.finCLA;                    % fins can only push this hard
    Ffin = min(max(m * (vxWanted - vx) / P.tauFin, -FfinMax), FfinMax);
end

% --- 6. NEWTON'S 2nd LAW (on a round, non-rotating Earth) ----------------
aThrust = F * u / m;
aDrag   = D * dragDir / m;
aFin    = [Ffin / m; 0];
aNG     = aThrust + aDrag + aFin;     % everything except gravity (what you feel)

% The extra terms (-vx*vh/r and +vx^2/r) appear because "horizontal" and
% "vertical" rotate as we move around a round planet. They are tiny for the
% hop, but they are what lets an orbiting stage stay up.
dx  = vx * P.Re / r;
dh  = vh;
dvx = aNG(1) - vx*vh/r;
dvh = aNG(2) - g + vx^2/r;

% --- 7. MASS FLOW: thrust = mdot * Isp * g0 --------------------------------
Isp = (P.Isp_vac - (P.Isp_vac - P.Isp_sl) * rho/P.rho0) * P.IspScale;
dm  = -F / (Isp * P.g0);

% --- 8. ROTATION: torque = I * angular acceleration ----------------------
dtheta = thetaDot;
domega = (tauG + tauR) / I;

dX = [dx; dh; dvx; dvh; dm; dtheta; domega];

% --- optional extra outputs for plotting --------------------------------
if nargout > 1
    aux = struct('throttle', throttle, 'F', F, 'delta', delta, ...
                 'gLoad', norm(aNG)/P.g0, 'q', q, 'thetaCmd', thetaCmd, ...
                 'tauRCS', tauR, 'Ffin', Ffin, 'wind', w);
end
end


%% =====================================================================
%  GUIDANCE  (decides the desired direction and throttle)
%  =====================================================================
function [thetaCmd, thrReq] = guidanceLaw(t, X, P, phase)
% thetaCmd : desired thrust/body angle from vertical [rad]
% thrReq   : requested throttle (0..1+, engine limits are applied later)
vx = X(3); vh = X(4); m = X(5);

switch phase
    case 'ascent'
        % Go straight up, then lean over by kickDeg so we travel downrange.
        if t < P.tKick, thetaCmd = 0; else, thetaCmd = deg2rad(P.kickDeg); end
        thrReq = 1;

    case 'flip'
        % Engine off. Turn sideways so the engine can push back to the pad.
        thetaCmd = P.bbDir * pi/2;
        thrReq   = 0;

    case 'boostback'
        % Engine on, pushing horizontally back toward the pad.
        thetaCmd = P.bbDir * pi/2;
        thrReq   = 1;

    case 'coast'
        % Engine off. Point the engines into the direction of travel
        % ("engines-first"), mirrored to stay nose-up while still climbing.
        thetaCmd = atan2(-vx, abs(vh));
        thrReq   = 0;

    case 'entry'
        % Retrograde burn: thrust exactly opposite the velocity.
        thetaCmd = atan2(-vx, -vh);
        thrReq   = 1;

    case 'landing'
        % Ask the landing law for the acceleration we need, then convert it
        % into a direction and a throttle setting.
        aCmd     = landingAccel(X, P);
        thetaCmd = atan2(aCmd(1), aCmd(2));
        thrReq   = norm(aCmd) * m / (enginesLit('landing', P) * P.T_eng);

    otherwise
        error('Unknown phase "%s".', phase);
end
end


function aCmd = landingAccel(X, P)
% Thrust acceleration [ax; ay] for the landing burn.
%
% VERTICAL: constant-deceleration ("suicide burn") law. To go from vertical
% speed vh to touchdown speed v_td over the remaining height h, we need a
% net upward deceleration of (vh^2 - v_td^2)/(2h). The engine must supply
% that plus gravity, minus whatever drag is already helping.
%
% HORIZONTAL: steer toward a desired sideways speed that brings us over the
% pad, fading to "just stop sliding" in the last 50 m.
x = X(1); h = X(2); vx = X(3); vh = X(4); m = X(5);
hEff = max(h, P.hFloor);
[g, rho] = earthModel(h, P);

v  = hypot(vx, vh);
aD = P.dragCredit * 0.5 * rho * v^2 * P.Cd_down * P.A / m;   % nominal drag decel
if v > 0, aDrag = aD * [-vx; -vh] / v; else, aDrag = [0; 0]; end

ay = (vh^2 - P.v_td^2) / (2*hEff) + g - aDrag(2);
ay = max(ay, 0);

% time-to-go for a constant-deceleration descent
tgo = max(2*hEff / max(-vh + P.v_td, P.v_td), 0.5);

dxPad  = P.xPad - x;
vxWant = sign(dxPad) * min(P.wnDivert*abs(dxPad), 2*abs(dxPad)/tgo);
vxWant = vxWant * min(1, h/P.divertFadeAlt);
tau    = min(max(tgo/3, P.divertTauMin), P.divertTauMax);
ax     = (vxWant - vx)/tau - aDrag(1);

% Limit how far we tilt; the limit shrinks to zero near the ground so
% the booster touches down upright.
tiltLim = P.maxTilt * min(1, h/P.tiltFadeAlt);
axLim   = ay * tan(tiltLim);
ax      = min(max(ax, -axLim), axLim);

aCmd = [ax; ay];
end


%% =====================================================================
%  ATTITUDE CONTROL  (turns the body: engine gimbal + RCS thrusters)
%  =====================================================================
function [delta, tauG, tauR, I] = attitudeControl(X, P, thetaCmd, F)
% A PD (proportional-derivative) controller asks for a torque. The engine
% gimbal provides as much as it can; RCS thrusters supply the rest.
h = X(2); vx = X(3); m = X(5); theta = X(6); omega = X(7);
r = P.Re + h;

I   = m * P.L^2 / 12;        % moment of inertia of a uniform rod about its middle
arm = P.L / 2;               % engine sits half a body length below the center of mass

err      = wrapAngle(thetaCmd - theta);   % shortest way around
thetaDot = omega - vx/r;
tauCmd   = I * (P.attWn^2 * err - 2*P.attZeta*P.attWn * thetaDot);

if F > 0
    tauGmax = F * arm * sin(P.gimbalMax);          % most the gimbal can give
    tauG    = min(max(tauCmd, -tauGmax), tauGmax);
    delta   = asin(tauG / (F*arm));                % nozzle angle that gives tauG
else
    tauG  = 0;
    delta = 0;
end
tauR = min(max(tauCmd - tauG, -P.tauRCS), P.tauRCS);  % RCS covers the remainder
end


%% =====================================================================
%  EVENTS  (conditions that end each phase)
%  ode45 stops when 'value' crosses zero in the given 'direction'.
%  =====================================================================
function [value, isterminal, direction] = phaseEvents(~, X, P, phase, nextPhase)
h = X(2); vx = X(3); vh = X(4); m = X(5); theta = X(6); omega = X(7);
mDry = P.m_dry * P.dryScale;
g    = earthModel(h, P);

switch phase
    case 'ascent'
        % (1) predicted apogee = h + vh^2/(2g) reaches target -> MECO
        % (2) propellant down to reserve                       -> MECO
        value     = [h + vh^2/(2*g) - P.h_target;  m - (mDry + P.reserve)];
        direction = [+1; -1];

    case 'flip'
        % attitude error AND rotation rate both small -> flip complete
        err      = abs(wrapAngle(P.bbDir*pi/2 - theta));
        thetaDot = abs(omega - vx/(P.Re + h));
        value     = max(err - deg2rad(P.flipTolDeg), thetaDot - deg2rad(P.flipRateTol));
        direction = -1;

    case 'boostback'
        % (1) predicted impact point crosses the pad -> stop pushing
        % (2) propellant at landing reserve          -> stop pushing
        value     = [(predictImpact(X, P) - P.xPad) * P.bbDir;  m - (mDry + P.reserveLand)];
        direction = [+1; -1];

    case 'coast'
        if strcmp(nextPhase, 'entry')
            % (1) falling through the entry-burn altitude
            if vh < 0, v1 = P.entryAlt - h; else, v1 = -1; end
        else
            % (1) landing-burn ignition: thrust needed reaches ignFrac of max
            if vh < 0 && h < P.ignAlt
                aCmd = landingAccel(X, P);
                v1 = m*norm(aCmd) - P.ignFrac * P.engLand * P.T_eng;
            else
                v1 = -1;
            end
        end
        % (2) ground impact
        value     = [v1; h];
        direction = [+1; -1];

    case 'entry'
        value     = hypot(vx, vh) - P.entryEndSpeed;   % slowed down enough
        direction = -1;

    case 'landing'
        value     = h;                                  % touchdown
        direction = -1;
end
isterminal = ones(size(value));
end


%% =====================================================================
%  HELPERS
%  =====================================================================
function xImpact = predictImpact(X, P)
% Where will we hit the ground if the engine stays off from now on?
% A quick, coarse simulation (2-second steps, drag included, nominal Cd,
% no wind). It is the booster's "onboard computer" estimate, so it does
% NOT know about the Monte Carlo dispersions.
s  = X(1:4);           % [x; h; vx; vh]
m  = X(5);
dt = 2;
f  = @(s) ballistic(s, m, P);
for i = 1:2000
    k1 = f(s);
    k2 = f(s + dt/2*k1);
    sNew = s + dt*k2;                   % midpoint (RK2) step
    if sNew(2) <= 0                     % crossed the ground: interpolate
        frac = s(2) / (s(2) - sNew(2));
        xImpact = s(1) + frac*(sNew(1) - s(1));
        return
    end
    s = sNew;
end
xImpact = s(1);
end

function ds = ballistic(s, m, P)
% Unpowered point-mass equations used by predictImpact.
h = s(2); vx = s(3); vh = s(4);
r = P.Re + h;
[g, rho] = earthModel(h, P);
v = hypot(vx, vh);
if vh >= 0, Cd = P.Cd_up; else, Cd = P.Cd_down; end
k = 0.5*rho*v*Cd*P.A/m;
ds = [vx*P.Re/r; vh; -k*vx - vx*vh/r; -k*vh - g + vx^2/r];
end

function n = enginesLit(phase, P)
% How many engines burn in each phase.
switch phase
    case 'ascent',    n = P.engAscent;
    case 'boostback', n = P.engBoost;
    case 'entry',     n = P.engEntry;
    case 'landing',   n = P.engLand;
    otherwise,        n = 0;
end
end

function w = windAt(h, P)
% Simple wind profile: grows to full strength at 10 km (jet stream),
% then dies away above 12 km.
h = max(h, 0);
w = P.wind * min(h/10e3, 1) * exp(-max(h - 12e3, 0)/5e3);
end

function a = wrapAngle(a)
% Wrap an angle to [-pi, pi] so the controller turns the short way.
a = atan2(sin(a), cos(a));
end
