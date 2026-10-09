function [g, rho, a] = earthModel(h, P)
%EARTHMODEL  Gravity, air density and speed of sound at altitude h.
%
%   [g, rho, a] = earthModel(h, P)
%     h   : altitude above sea level [m] (scalar or array)
%     P   : struct with fields g0, Re, rho0, Hscale and (optionally)
%           atmosphere = 'us76' (default) or 'exponential'
%     g   : gravitational acceleration [m/s^2]
%     rho : air density [kg/m^3]
%     a   : speed of sound [m/s] (Mach number = speed / a)

% Gravity follows the inverse-square law: at 110 km it is ~3% weaker,
% at 250 km about 7% weaker.
g = P.g0 * (P.Re ./ (P.Re + h)).^2;

if isfield(P, 'atmosphere') && strcmp(P.atmosphere, 'exponential')
    % Simple exponential atmosphere: density halves roughly every 5.9 km.
    rho = P.rho0 * exp(-max(h, 0) / P.Hscale);
    a   = 340.3 * ones(size(h));
else
    [rho, a] = us76(h);
end
end


function [rho, a] = us76(h)
% U.S. Standard Atmosphere 1976.
% Below 86 km: seven layers in which temperature changes linearly with
% (geopotential) height, and pressure follows from hydrostatic balance.
% Above 86 km: tabulated standard densities, interpolated in log space.
R  = 287.053;  gam = 1.4;  g0 = 9.80665;  r0 = 6356766;
Hb = [0 11 20 32 47 51 71 84.852] * 1e3;             % layer bases [m, geopotential]
Tb = [288.15 216.65 216.65 228.65 270.65 270.65 214.65 186.946];   % [K]
Lb = [-6.5 0 1.0 2.8 0 -2.8 -2.0] * 1e-3;            % lapse rates [K/m]
pb = [101325 22632.06 5474.889 868.0187 110.9063 66.93887 3.956420];   % [Pa]

hc = max(h, 0);
H  = r0 * hc ./ (r0 + hc);                            % geopotential altitude

if isscalar(h) && H < Hb(end)
    % Fast path for one altitude (this is how the ODE solver calls it)
    k = find(H >= Hb(1:7), 1, 'last');
    dH = H - Hb(k);
    if Lb(k) == 0
        T = Tb(k);  p = pb(k) * exp(-g0 * dH / (R * T));
    else
        T = Tb(k) + Lb(k) * dH;  p = pb(k) * (T / Tb(k))^(-g0 / (R * Lb(k)));
    end
    rho = p / (R * T);
    a   = sqrt(gam * R * T);
    return
end
zt = [86 100 110 120 150 200 300 500 1000];           % thermosphere table [km]
rt = [6.958e-6 5.604e-7 9.708e-8 2.222e-8 2.076e-9 2.541e-10 1.916e-11 5.215e-13 3.561e-15];
if isscalar(h)
    z = min(max(hc/1e3, zt(1)), zt(end));             % (H and z switch layers a few cm apart)
    k = min(find(z >= zt, 1, 'last'), numel(zt) - 1);
    f = (z - zt(k)) / (zt(k+1) - zt(k));
    rho = rt(k) * (rt(k+1)/rt(k))^f;                  % log-linear interpolation
    a   = sqrt(gam * R * 186.87);
    return
end
rho = zeros(size(h));  T = zeros(size(h));
low = H < Hb(end);
for k = 1:7
    in = low & H >= Hb(k) & (H < Hb(k+1) | k == 7);
    if ~any(in(:)), continue; end
    dH = H(in) - Hb(k);
    if Lb(k) == 0
        Tk = Tb(k) * ones(size(dH));
        pk = pb(k) * exp(-g0 * dH / (R * Tb(k)));
    else
        Tk = Tb(k) + Lb(k) * dH;
        pk = pb(k) * (Tk / Tb(k)).^(-g0 / (R * Lb(k)));
    end
    T(in)   = Tk;
    rho(in) = pk ./ (R * Tk);
end
if any(~low(:))
    % Thermosphere: standard densities (table above), log-interpolated
    z  = min(max(hc(~low) / 1e3, zt(1)), zt(end));
    rho(~low) = exp(interp1(zt, log(rt), z));
    T(~low)   = 186.87;                               % sound barely matters up here
end
a = sqrt(gam * R * T);
end
