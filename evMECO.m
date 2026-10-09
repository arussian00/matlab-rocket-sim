function [value, isterminal, direction] = evMECO(~, Y, mMECO)
% Booster cut-off (MECO) when only its return propellant is left.
% Used by rocket_two_stage_sim.m and rocket_starship_sim.m.
value = Y(5) - mMECO; isterminal = 1; direction = -1;
end
