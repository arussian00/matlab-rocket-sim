function [dY, u, F, aNG] = starshipStack(t, Y, S, E, mu, mode)
% Point-mass equations of motion on a round, non-rotating Earth.
% mode: 'stack' (33 engines, gravity turn) or 'ship' (6 engines to orbit)
% Extra outputs (for plots): thrust direction u, thrust F, and the
% acceleration you would feel, aNG (everything except gravity).
% Used by rocket_starship_sim.m (moved out of the script so it also runs in Octave).
h = Y(2); vx = Y(3); vh = Y(4); m = Y(5);
r = E.Re + h;
g = mu / r^2;
[~, rho, aSound] = earthModel(h, E);
v = hypot(vx, vh);
M = v / aSound;                                     % Mach number
Cd = S.Cd;
if E.machDrag, Cd = Cd * machDrag(M); end           % sound barrier
D = 0.5*rho*v^2*Cd*S.A;

switch mode
    case 'stack'
        % Straight up, short pitch kick, then GRAVITY TURN (thrust along velocity).
        % Max-q throttle bucket: ease off through the sound barrier, where
        % dynamic pressure and drag loads on the structure are highest.
        mb  = S.bucketMach;
        w   = min(max((M - mb(1))/(mb(2) - mb(1)), 0), 1) * min(max((mb(4) - M)/(mb(4) - mb(3)), 0), 1);
        F   = S.T1 * (1 - (1 - S.thrBucket)*w);
        Isp = S.Isp1vac - (S.Isp1vac - S.Isp1sl)*rho/E.rho0;
        if t < S.tKick || v == 0
            u = [0; 1];
        elseif t < S.tKick + S.kickDur
            u = [sind(S.kickDeg); cosd(S.kickDeg)];
        else
            u = [vx; vh]/v;
        end
    case 'ship'
        % PD controller drives altitude to hCoast and vertical speed to zero;
        % everything left over builds horizontal speed.
        F   = min(S.T2, S.gLimit*E.g0*m);   % throttle down as it gets lighter
        Isp = S.Isp2;
        aT  = F/m;
        aUp = S.wnG^2*(S.hCoast - h) - 2*S.zG*S.wnG*vh + g - vx^2/r;
        aUp = min(max(aUp, -aT), aT);
        u   = [sqrt(max(aT^2 - aUp^2, 0)); aUp] / aT;
end

if v > 0, aDrag = -D*[vx; vh]/(v*m); else, aDrag = [0; 0]; end
aNG = F*u/m + aDrag;
dY = [vx*E.Re/r; vh; aNG(1) - vx*vh/r; aNG(2) - g + vx^2/r; -F/(Isp*E.g0)];
end
