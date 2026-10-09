function [value, isterminal, direction] = perigeeSECO(~, Y, S, E, mu)
% Ship cut-off: (1) the perigee rises to S.perigee, or
%               (2) only the landing reserve is left.
% Used by rocket_starship_sim.m.
rp = orbitOf(Y, E.Re, mu);
value      = [rp - S.perigee;  Y(5) - (S.m2dry + S.mPay + S.reserve2)];
isterminal = [1; 1];
direction  = [+1; -1];
end
