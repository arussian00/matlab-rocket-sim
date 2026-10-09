function qdot = heatFlux(h, v, Rn, P)
%HEATFLUX  Estimated stagnation-point heating rate during atmospheric flight.
%
%   qdot = heatFlux(h, v, Rn, P)
%     h   altitude [m],  v  speed [m/s]   (scalars or arrays)
%     Rn  effective nose radius [m]: a blunter shape spreads the heat
%     P   planet values from rocket_params (for the air density)
%     qdot heat flux into the surface [W/m^2]
%
%   Sutton-Graves formula, the classic first estimate for re-entry heating:
%   qdot = k * sqrt(rho/Rn) * v^3. Heating grows with the CUBE of speed,
%   so it peaks high up, where the vehicle is still fast, not at max-q.
%   It ignores radiation from the hot shock layer and the actual
%   tile shape, so treat it as an order-of-magnitude guide.

k = 1.7415e-4;                      % for Earth's air, SI units
[~, rho] = earthModel(h, P);
qdot = k .* sqrt(rho ./ Rn) .* abs(v).^3;
end
