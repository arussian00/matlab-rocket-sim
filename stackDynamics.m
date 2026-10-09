function [dY, u, F, a] = stackDynamics(t, Y, S, E, mu, mode)
% Point-mass equations of motion on a round, non-rotating Earth.
% mode: 'stage1' (stack burning), 'stage2' (upper stage burning), 'coast'
% Extra outputs (for plots): thrust direction u, thrust F, and the
% acceleration you would feel, a (everything except gravity).
% Used by rocket_two_stage_sim.m (moved out of the script so it also runs in Octave).
x = Y(1); h = Y(2); vx = Y(3); vh = Y(4); m = Y(5); %#ok<NASGU>
r = E.Re + h;
g = mu / r^2;
[~, rho, aSound] = earthModel(h, E);
v = hypot(vx, vh);
Cd = S.Cd;
if E.machDrag, Cd = Cd * machDrag(v / aSound); end  % sound barrier
D = 0.5*rho*v^2*Cd*S.A;

switch mode
    case 'stage1'
        % STEERING: straight up, short pitch kick, then GRAVITY TURN
        % (thrust along the velocity, so gravity gradually bends the path
        %  over without the rocket ever flying sideways through thick air)
        F   = S.T1;
        Isp = S.Isp1vac - (S.Isp1vac - S.Isp1sl)*rho/E.rho0;
        if t < S.tKick
            u = [0; 1];
        elseif t < S.tKick + S.kickDur
            u = [sind(S.kickDeg); cosd(S.kickDeg)];
        else
            u = [vx; vh]/v;
        end
    case 'stage2'
        % STEERING: split the available acceleration into a vertical part
        % (a PD controller that drives altitude to hOrbit and vertical speed
        % to zero) and use everything left over to build horizontal speed.
        F   = S.T2;
        Isp = S.Isp2;
        aT  = F/m;
        aUp = S.wnG^2*(S.hOrbit - h) - 2*S.zG*S.wnG*vh ...  % PD on altitude
              + g - vx^2/r;                                % cancel net gravity
        aUp = min(max(aUp, -aT), aT);
        u   = [sqrt(max(aT^2 - aUp^2, 0)); aUp] / aT;
    otherwise
        F = 0; Isp = 1; u = [0; 0];
end

if v > 0, aDrag = -D*[vx; vh]/(v*m); else, aDrag = [0; 0]; end
a  = F*u/m + aDrag;
dY = [vx*E.Re/r; vh; a(1) - vx*vh/r; a(2) - g + vx^2/r; -F/(Isp*E.g0)];
end
