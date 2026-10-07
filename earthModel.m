function [g, rho] = earthModel(h, P)
%EARTHMODEL  Gravity and air density at altitude h.
%
%   [g, rho] = earthModel(h, P)
%     h   : altitude above sea level [m] (scalar or array)
%     P   : struct with fields g0, Re, rho0, Hscale (see rocket_params.m)
%     g   : gravitational acceleration [m/s^2]
%     rho : air density [kg/m^3]

% Gravity follows the inverse-square law: at 110 km it is ~3% weaker,
% at 250 km about 7% weaker.
g = P.g0 * (P.Re ./ (P.Re + h)).^2;

% Exponential atmosphere: density halves roughly every 5.9 km.
% (Below ground we just use the sea-level value.)
rho = P.rho0 * exp(-max(h, 0) / P.Hscale);
end
