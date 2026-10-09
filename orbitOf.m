function [perigee, apogee] = orbitOf(Y, Re, mu)
% Perigee and apogee altitudes [m] from a state (classical two-body formulas).
% Used by rocket_starship_sim.m.
r  = Re + Y(2);
energy = (Y(3)^2 + Y(4)^2)/2 - mu/r;
if energy >= 0, perigee = Inf; apogee = Inf; return; end   % escaping
a   = -mu/(2*energy);
ecc = sqrt(max(0, 1 - (r*Y(3))^2/(mu*a)));
perigee = a*(1 - ecc) - Re;
apogee  = a*(1 + ecc) - Re;
end
