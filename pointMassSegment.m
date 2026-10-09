function seg = pointMassSegment(t, Y, P, dynFcn)
%POINTMASSSEGMENT  Convert a point-mass ascent into simulateBooster's format.
%
%   seg = pointMassSegment(t, Y, P, dynFcn)
%     t, Y    ode45 result for the state [x; h; vx; vh; m]
%     P       planet values (g0, Re, rho0, Hscale) from rocket_params
%     dynFcn  @(t, Y) returning [dY, u, F, aNG]: derivative, thrust
%             direction, thrust [N] and felt acceleration (no gravity)
%
%   seg has fields t, X (7 states, pitch taken along the thrust) and aux,
%   so it can be joined to a booster flight with fullFlight.m.

n = numel(t);
X = [Y(:,1:5), zeros(n, 2)];
aux = struct('throttle', zeros(n,1), 'thrust', zeros(n,1), 'gimbal', zeros(n,1), ...
             'gLoad', zeros(n,1), 'q', zeros(n,1), 'thetaCmd', zeros(n,1), ...
             'tauRCS', zeros(n,1), 'finForce', zeros(n,1), 'wind', zeros(n,1));
for i = 1:n
    [~, u, F, aNG] = dynFcn(t(i), Y(i,:).');
    [~, rho] = earthModel(Y(i,2), P);
    if any(u), X(i,6) = atan2(u(1), u(2)); end  % body points along the thrust
    aux.throttle(i) = double(F > 0);
    aux.thrust(i)   = F;
    aux.gLoad(i)    = norm(aNG) / P.g0;
    aux.q(i)        = 0.5 * rho * (Y(i,3)^2 + Y(i,4)^2);
end
aux.thetaCmd = X(:,6);
X(:,7) = gradient(X(:,6), t) + Y(:,3)./(P.Re + Y(:,2));   % pitch rate [rad/s]
seg = struct('t', t, 'X', X, 'aux', aux);
end
