function P = rocket_params(vehicle)
%ROCKET_PARAMS  Every tunable number for the booster simulation, in one place.
%
%   P = rocket_params()          -> 'hop' vehicle (single stage, launches to
%                                   ~110 km, boosts back, lands on the pad)
%   P = rocket_params('hop')     -> same as above
%   P = rocket_params('booster') -> first stage of the two-stage orbital
%                                   rocket (lands on a drone ship)
%
%   The returned struct P is passed to simulateBooster.m. Every field is
%   explained below. Units are SI (m, kg, s, N) unless the name says
%   otherwise (e.g. ...Deg means degrees).
%
%   Tip: change values here, or override them in a script after calling
%   this function, e.g.   P = rocket_params(); P.kickDeg = 3;

if nargin < 1, vehicle = 'hop'; end

%% ---------------------------------------------------------------------
%  STEP 1 - Planet and atmosphere
%  ---------------------------------------------------------------------
P.g0     = 9.80665;   % standard gravity at sea level [m/s^2]
P.Re     = 6371e3;    % mean Earth radius [m]
P.rho0   = 1.225;     % air density at sea level [kg/m^3]
P.Hscale = 8500;      % density scale height: air thins by a factor e every 8.5 km [m]

%% ---------------------------------------------------------------------
%  STEP 2 - Vehicle: mass, engines, shape
%  ---------------------------------------------------------------------
P.vehicleName = 'Suborbital hop';
P.m_dry    = 6000;     % empty booster mass (structure + engines) [kg]
P.m_prop   = 19000;    % propellant loaded at liftoff [kg]
P.T_eng    = 400e3;    % thrust of ONE engine at full throttle [N]
P.Isp_sl   = 280;      % specific impulse at sea level [s]   (engine efficiency)
P.Isp_vac  = 310;      % specific impulse in vacuum   [s]   (better: no air pushing back)
P.thrMin   = 0.30;     % NEW: deepest throttle the engine can run at (30%).
                       %      A lit engine can never produce less than this.
P.diam     = 3.5;      % body diameter [m]
P.L        = 40;       % body length [m] (used for moment of inertia and lever arm)
P.Cd_up    = 0.5;      % drag coefficient when climbing nose-first
P.Cd_down  = 1.0;      % drag coefficient when falling engines-first (with drag brakes)

% Engines lit in each powered phase (NEW: lets a 9-engine booster land on 1)
P.engAscent = 1;
P.engBoost  = 1;
P.engEntry  = 1;
P.engLand   = 1;

%% ---------------------------------------------------------------------
%  STEP 3 - Attitude (rotation) dynamics and control          (NEW)
%  ---------------------------------------------------------------------
P.gimbalMaxDeg = 6;     % how far the engine nozzle can swivel [deg]
P.tauRCS       = 40e3;  % max torque from cold-gas thrusters (RCS) [N*m]
P.attWn        = 1.0;   % attitude controller natural frequency [rad/s] (speed of response)
P.attZeta      = 0.8;   % attitude controller damping ratio (0.7-1 = little overshoot)

%% ---------------------------------------------------------------------
%  STEP 4 - Ascent guidance
%  ---------------------------------------------------------------------
P.h_target = 110e3;    % cut the engine when the predicted apogee reaches this [m]
P.tKick    = 10;       % time to start tilting over [s]
P.kickDeg  = 5;        % tilt angle held after the kick [deg] (sends us downrange)
P.reserve  = 2500;     % propellant that MUST remain at engine cut-off [kg]

%% ---------------------------------------------------------------------
%  STEP 5 - Return: flip, boostback, entry burn                 (NEW)
%  ---------------------------------------------------------------------
P.xPad          = 0;     % landing pad position, downrange [m]. NaN = drone ship
                         % (placed automatically at the predicted impact point)
P.flipTolDeg    = 2;     % flip is "done" when attitude error < this [deg] ...
P.flipRateTol   = 0.5;   % ... and rotation rate < this [deg/s]
P.reserveLand   = 900;   % boostback stops early if propellant falls to this [kg]
P.entryAlt      = 55e3;  % entry burn starts when falling through this altitude [m]
P.entryEndSpeed = 900;   % entry burn stops once speed is below this [m/s]

%% ---------------------------------------------------------------------
%  STEP 6 - Descent steering with grid fins                      (NEW)
%  ---------------------------------------------------------------------
P.finAlt = 40e3;   % grid fins work below this altitude (need air) [m]
P.finCLA = 4.0;    % fin effectiveness: max side force = q * finCLA [m^2]
P.tauFin = 3.0;    % how quickly the fins correct sideways speed [s]
P.finAimAlt = 3e3; % fins steer to be over the pad by this altitude [m]
                   % (so there is no sideways drift left at landing-burn ignition)

%% ---------------------------------------------------------------------
%  STEP 7 - Landing burn guidance
%  ---------------------------------------------------------------------
P.ignFrac      = 0.75;   % light the engine when the needed thrust reaches 75% of max
P.ignAlt       = 10e3;   % never light the landing burn above this altitude [m]
P.v_td         = 1.0;    % target touchdown speed [m/s]
P.hFloor       = 0.2;    % altitude floor in guidance math (avoids divide-by-zero) [m]
P.dragCredit   = 0.5;    % fraction of current drag the guidance counts on for braking
P.maxTiltDeg   = 15;     % max thrust tilt from vertical while diverting [deg]
P.tiltFadeAlt  = 20;     % tilt limit fades to 0 below this altitude -> upright touchdown [m]
P.wnDivert     = 0.3;    % sideways correction gain [1/s]
P.divertFadeAlt= 50;     % stop chasing the pad position below this altitude [m]
P.divertTauMin = 2.5;    % sideways-speed correction time constant, min [s]
P.divertTauMax = 3.0;    % ... and max [s]

%% ---------------------------------------------------------------------
%  STEP 8 - Success criteria at touchdown
%  ---------------------------------------------------------------------
P.success.missMax    = 20;   % max distance from pad center [m]
P.success.vVertMax   = 3;    % max vertical speed [m/s]
P.success.vHorizMax  = 5;    % max sideways speed [m/s]
P.success.tiltMaxDeg = 6;    % max lean from vertical [deg]

%% ---------------------------------------------------------------------
%  STEP 9 - "Truth" dispersions (used by the Monte Carlo)        (NEW)
%  The GUIDANCE always assumes nominal values; only the PHYSICS uses these.
%  ---------------------------------------------------------------------
P.thrustScale = 1;   % actual thrust  = nominal * thrustScale
P.IspScale    = 1;   % actual Isp     = nominal * IspScale
P.CdScale     = 1;   % actual drag    = nominal * CdScale
P.dryScale    = 1;   % actual dry mass= nominal * dryScale
P.wind        = 0;   % wind speed at 10-12 km altitude [m/s] (+ = blowing downrange)

%% ---------------------------------------------------------------------
%  STEP 10 - Mission sequence and solver settings
%  ---------------------------------------------------------------------
P.phaseList = {'ascent', 'flip', 'boostback', 'coast', 'landing'};
P.maxStep   = 0.2;    % largest ode45 time step [s] (smooth plots, accurate attitude)
P.tPhaseMax = 3000;   % safety limit on any single phase [s]

%% ---------------------------------------------------------------------
%  Vehicle variants
%  ---------------------------------------------------------------------
switch lower(vehicle)
    case 'hop'
        % defaults above

    case 'booster'
        % First stage of the two-stage rocket (rocket_two_stage_sim.m).
        % It starts the simulation at stage separation, so ascent values
        % are handled by the two-stage script.
        P.vehicleName = 'Orbital booster (stage 1)';
        P.m_dry     = 14000;
        P.m_prop    = 130000;
        P.T_eng     = 2.6e6/9;    % nine engines share 2.6 MN
        P.Isp_sl    = 282;
        P.Isp_vac   = 311;
        P.thrMin    = 0.40;
        P.diam      = 3.7;
        P.L         = 45;
        P.tauRCS    = 150e3;
        P.engAscent = 9;
        P.engBoost  = 3;
        P.engEntry  = 3;          % entry burn on 3 engines
        P.engLand   = 1;          % landing burn on the center engine only
        P.xPad      = NaN;        % drone ship: placed where the booster will fall
        P.reserveLand = 2500;
        P.finAimAlt = 0;          % still ~150 m/s sideways at ignition: the landing
                                  % burn does the final divert, so fins aim at the ground
        P.phaseList = {'coast', 'entry', 'coast', 'landing'};

    otherwise
        error('rocket_params: unknown vehicle "%s" (use ''hop'' or ''booster'').', vehicle);
end

%% Derived values (computed from the numbers above - do not edit)
P.A         = pi*(P.diam/2)^2;          % frontal area [m^2]
P.gimbalMax = deg2rad(P.gimbalMaxDeg);  % [rad]
P.maxTilt   = deg2rad(P.maxTiltDeg);    % [rad]
P.bbDir     = 0;                        % boostback direction, set during the flight
end
