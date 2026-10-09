function f = machDrag(M)
%MACHDRAG  Drag multiplier versus Mach number (1 = the subsonic value).
%
%   f = machDrag(M)      M = speed / speed of sound (scalar or array)
%
%   Real rocket drag rises sharply through the "sound barrier": shock
%   waves form on the body, and the drag coefficient peaks around Mach 1.1
%   to 1.2 at roughly 1.7x its subsonic value. It then falls back toward
%   the subsonic value at hypersonic speed. Typical slender-body curve.
%   The drag coefficients in rocket_params.m are the SUBSONIC values.

Mt = [0   0.6  0.8  0.95 1.05 1.2  1.5  2.0  3.0  5.0];
ft = [1.0 1.0  1.1  1.45 1.65 1.7  1.5  1.3  1.12 1.0];
if isscalar(M)
    if M >= Mt(end), f = ft(end); return; end     % fast path for the ODE solver
    k = find(M >= Mt, 1, 'last');
    f = ft(k) + (ft(k+1) - ft(k)) * (M - Mt(k)) / (Mt(k+1) - Mt(k));
else
    f = interp1(Mt, ft, min(max(M, 0), Mt(end)));
end
end
