function [value, isterminal, direction] = circularSECO(~, Y, S, E, mu)
% Stage 2 cut-off: (1) horizontal speed reaches circular-orbit speed,
%                  (2) or stage 2 runs dry.
% Used by rocket_two_stage_sim.m.
r = E.Re + Y(2);
value      = [Y(3) - sqrt(mu/r);  Y(5) - (S.m2dry + S.mPay)];
isterminal = [1; 1];
direction  = [+1; -1];
end
