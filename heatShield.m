function hs = heatShield(t, h, v, P, tileModel)
%HEATSHIELD  Heat load and heat-shield temperature along a flight.
%
%   hs = heatShield(t, h, v, P)
%   hs = heatShield(t, h, v, P, tileModel)
%
%   t, h, v    time [s], altitude [m] and speed [m/s]. Vectors, any shape.
%   P          parameters from rocket_params (planet data, noseRadius, and
%              the tile* / skin* fields). Missing tile fields fall back to
%              the Starship defaults.
%   tileModel  true (default) runs the tile temperature model. false returns
%              only the heating rate and its integral (used for Super Heavy,
%              which is bare steel and has no tile heat shield).
%
%   Heating rate is the Sutton-Graves stagnation value from heatFlux.m,
%   applied here as the windward-side convective flux. Heat load is that
%   flux integrated over the flight:
%
%       Q(t) = ∫ q dt          [J/m^2]
%
%   Tile temperature is a 1-D slab (conduction through the thickness) with
%   the stainless skin lumped onto the inner face:
%
%       rho cp dT/dt = d/dx (k dT/dx)
%       outer face:  q_conv - eps sigma (T^4 - Tenv^4) = -k dT/dx
%       inner face:  adiabatic (the skin's heat capacity is included)
%
%   q_conv is the cold-wall Sutton-Graves flux, so it is not reduced as the
%   wall heats up. That makes the surface temperature a little high, which
%   is the conservative direction for a limit check.
%
%   Radiative equilibrium (no thermal mass, nothing conducted inward),
%
%       eps sigma (Trad^4 - Tenv^4) = q_conv
%
%   is stored as well. On a long, lifting entry the outer surface sits close
%   to Trad. The 1-D model is what carries the heat through to the steel.
%
%   hs.pass is true when the peak outer-surface temperature is at or below
%   P.tileLimitC. The margin is limit minus peak, in degrees Celsius.

arguments
    t {mustBeVector, mustBeFinite, mustBeNonempty}
    h {mustBeVector, mustBeFinite, mustBeNonempty}
    v {mustBeVector, mustBeFinite, mustBeNonempty}
    P (1,1) struct
    tileModel (1,1) logical = true
end

% Default if the caller omitted it. The arguments block already does this
% in MATLAB; the same line keeps a no-arguments caller working.
if nargin < 5 || isempty(tileModel)
    tileModel = true;
end

t = t(:);
h = h(:);
v = v(:);
if numel(h) ~= numel(t) || numel(v) ~= numel(t)
    error('heatShield:size', 't, h and v must have the same number of elements.');
end
if any(diff(t) < -1e-6)
    error('heatShield:time', 'Time must be non-decreasing.');
end

tp = tileProperties(P);
q  = heatFlux(h, v, P.noseRadius, P);
q  = max(q, 0);
Q  = cumtrapz(t, q);

hs = struct();
hs.t        = t;
hs.q        = q;
hs.Q        = Q;
hs.qMax     = max(q);
hs.iQmax    = find(q == hs.qMax, 1, 'first');
hs.Qtot     = Q(end);
hs.tileModel = tileModel;

if ~tileModel
    hs.Trad = nan(size(t));
    hs.Tsurf = nan(size(t));
    hs.Tback = nan(size(t));
    hs.TradMax = NaN;   hs.TradMaxC = NaN;
    hs.TsurfMax = NaN;  hs.TsurfMaxC = NaN;  hs.iTmax = 1;  hs.tTmax = t(1);
    hs.TbackMax = NaN;  hs.TbackMaxC = NaN;
    hs.limitC = tp.limitC;  hs.marginC = NaN;  hs.pass = false;
    hs.radiatedFrac = NaN;
    return
end

[Tsurf, Tback, Trad] = integrateTile(t, q, tp);
hs.Trad  = Trad;
hs.Tsurf = Tsurf;
hs.Tback = Tback;
[hs.TsurfMax, hs.iTmax] = max(Tsurf);
hs.tTmax     = t(hs.iTmax);
hs.TsurfMaxC = hs.TsurfMax - 273.15;
hs.TradMax   = max(Trad);
hs.TradMaxC  = hs.TradMax - 273.15;
hs.TbackMax  = max(Tback);
hs.TbackMaxC = hs.TbackMax - 273.15;
hs.limitC    = tp.limitC;
hs.marginC   = tp.limitC - hs.TsurfMaxC;
hs.pass      = hs.marginC >= 0;

% Share of the convective load that left again by radiation. The rest is
% still in the tile and the steel at the end of the flight. trapz uses the
% saved surface history, so this is the same order as the solver, not a
% separate energy identity.
qRad = tp.eps * tp.sigma .* (Tsurf.^4 - tp.Tenv.^4);
hs.radiatedFrac = trapz(t, qRad) / max(hs.Qtot, 1);
end

function tp = tileProperties(P)
% Starship defaults. rocket_params sets the same numbers; a hand-built
% struct that omits one of them still runs.
tp.thick  = need(P, 'tileThick',  0.025);
tp.rho    = need(P, 'tileRho',    320);
tp.cp     = need(P, 'tileCp',     1000);
tp.k      = need(P, 'tileK',      0.10);
tp.eps    = need(P, 'tileEps',    0.85);
tp.nodes  = round(need(P, 'tileNodes', 21));
tp.T0     = need(P, 'tileT0',     293.15);
tp.Tenv   = need(P, 'tileTenv',   250);
tp.limitC = need(P, 'tileLimitC', 1500);
tp.skinC  = need(P, 'skinRho', 7930) * need(P, 'skinThick', 0.004) * need(P, 'skinCp', 500);
tp.sigma  = 5.670374419e-8;
if tp.thick <= 0 || tp.nodes < 3 || tp.k <= 0 || tp.eps <= 0
    error('heatShield:tile', 'Tile thickness, node count, conductivity and emissivity must be positive (at least 3 nodes).');
end
end

function v = need(P, name, default)
if isfield(P, name) && ~isempty(P.(name))
    v = P.(name);
else
    v = default;
end
end

function [Tsurf, Tback, Trad] = integrateTile(t, q, tp)
% Backward Euler through the thickness. Conduction is implicit. Radiation
% is linearised about the temperature at the start of the step
% (T^4 ≈ 4 Told^3 T - 3 Told^4), which keeps the step stable when the
% surface is hot and the radiative time constant is a fraction of a second.
N  = tp.nodes;
dx = tp.thick / (N - 1);
Cend = tp.rho * tp.cp * dx / 2;          % outer half-cell [J/m^2/K]
Cint = tp.rho * tp.cp * dx;              % interior cell
Cback = Cend + tp.skinC;                 % inner half-cell + steel skin
cond = tp.k / dx;                        % conductance between nodes [W/m^2/K]
sigma = tp.sigma;

T = tp.T0 * ones(N, 1);
n = numel(t);
Tsurf = zeros(n, 1);
Tback = zeros(n, 1);
Tsurf(1) = T(1);
Tback(1) = T(N);
Trad = (max(q, 0) ./ (tp.eps * sigma) + tp.Tenv.^4).^(0.25);

dtMax = 0.1;                             % [s] resolves the hot-surface time constant
for i = 1:n-1
    span = t(i+1) - t(i);
    if span <= 0
        Tsurf(i+1) = T(1);
        Tback(i+1) = T(N);
        continue
    end
    nSub = max(1, ceil(span / dtMax));
    dt = span / nSub;
    for s = 1:nSub
        qs = q(i) + (q(i+1) - q(i)) * (s / nSub);   % flux at the new time
        T  = tileStep(T, qs, dt, N, Cend, Cint, Cback, cond, tp.eps, sigma, tp.Tenv);
    end
    Tsurf(i+1) = T(1);
    Tback(i+1) = T(N);
end
end

function T = tileStep(Told, qs, dt, N, Cend, Cint, Cback, cond, eps, sigma, Tenv)
% One backward-Euler step. a = subdiagonal, b = diagonal, c = superdiagonal.
a = -cond * ones(N-1, 1);
b = zeros(N, 1);
c = -cond * ones(N-1, 1);
d = zeros(N, 1);

To = max(Told(1), 1);
hrad = 4 * eps * sigma * To^3;
b(1) = Cend/dt + hrad + cond;
d(1) = Cend/dt * Told(1) + qs + 3 * eps * sigma * To^4 + eps * sigma * Tenv^4;

for i = 2:N-1
    b(i) = Cint/dt + 2*cond;
    d(i) = Cint/dt * Told(i);
end

b(N) = Cback/dt + cond;
d(N) = Cback/dt * Told(N);

T = thomas(a, b, c, d);
T = max(T, 1);                           % radiation linearisation needs T > 0
end

function x = thomas(a, b, c, d)
% Thomas algorithm for a tridiagonal system. a is below the diagonal.
n = numel(b);
cp = zeros(n-1, 1);
dp = zeros(n, 1);
cp(1) = c(1) / b(1);
dp(1) = d(1) / b(1);
for i = 2:n-1
    den = b(i) - a(i-1) * cp(i-1);
    cp(i) = c(i) / den;
    dp(i) = (d(i) - a(i-1) * dp(i-1)) / den;
end
den = b(n) - a(n-1) * cp(n-1);
dp(n) = (d(n) - a(n-1) * dp(n-1)) / den;
x = zeros(n, 1);
x(n) = dp(n);
for i = n-1:-1:1
    x(i) = dp(i) - cp(i) * x(i+1);
end
end
