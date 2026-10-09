function M = flightMetrics(out, verbose)
%FLIGHTMETRICS  Key numbers from a simulateBooster result (+ optional printout).
%
%   M = flightMetrics(out)          compute and print a flight summary
%   M = flightMetrics(out, false)   compute silently (used by the Monte Carlo)

if nargin < 2, verbose = true; end
P = out.P;
t = out.t;  X = out.X;
x = X(:,1); h = X(:,2); vx = X(:,3); vh = X(:,4); m = X(:,5); theta = X(:,6);
mDry = P.m_dry * P.dryScale;

%% STEP 1 - Flight-wide numbers
[M.apogee, iApo] = max(h);
M.tApogee   = t(iApo);
inSpace     = h >= 100e3;
M.timeInSpace = sum(diff(t) .* inSpace(1:end-1));     % seconds above the Karman line
M.maxQ      = max(out.aux.q);
M.maxG      = max(out.aux.gLoad(t > t(1) + 1));       % skip the first second
M.propUsed  = m(1) - m(end);
M.propLeft  = m(end) - mDry;

%% STEP 2 - When did each phase start?
M.phaseStart = out.phaseStart;
iLand = find(strcmp(out.phaseList, 'landing'), 1);
if ~isempty(iLand) && ~isnan(out.phaseStart(iLand))
    j = find(t >= out.phaseStart(iLand), 1);
    M.landIgnAlt   = h(j);
    M.landIgnSpeed = hypot(vx(j), vh(j));
else
    M.landIgnAlt = NaN; M.landIgnSpeed = NaN;
end

%% STEP 3 - Touchdown state and pass/fail checks
M.vVert   = -vh(end);                       % + = moving down
M.vHoriz  = vx(end);
M.tiltDeg = rad2deg(atan2(sin(theta(end)), cos(theta(end))));   % wrapped to +-180 deg
M.miss    = x(end) - P.xPad;                % + = landed past the pad
M.xLand   = x(end);
M.tEnd    = t(end);

S = P.success;
M.success = ~out.crashed && strcmp(out.phaseList{out.phase(end)}, 'landing') ...
            && abs(M.miss)    <= S.missMax ...
            && abs(M.vVert)   <= S.vVertMax ...
            && abs(M.vHoriz)  <= S.vHorizMax ...
            && abs(M.tiltDeg) <= S.tiltMaxDeg;

if ~verbose, return; end

%% STEP 4 - Print
fprintf('\n================ FLIGHT SUMMARY: %s ================\n', P.vehicleName);
fprintf('Liftoff / start mass  : %9.0f kg\n', m(1));
for k = 1:numel(out.phaseList)
    if ~isnan(out.phaseStart(k))
        j = find(t >= out.phaseStart(k), 1);
        fprintf('  %-10s starts  : t = %6.1f s   h = %7.2f km   v = %6.0f m/s\n', ...
                out.phaseList{k}, t(j), h(j)/1e3, hypot(vx(j), vh(j)));
    end
end
fprintf('Apogee                : %9.1f km at t = %.1f s\n', M.apogee/1e3, M.tApogee);
fprintf('Time above 100 km     : %9.1f s\n', M.timeInSpace);
fprintf('Max dynamic pressure  : %9.1f kPa\n', M.maxQ/1e3);
fprintf('Max g-load            : %9.2f g\n', M.maxG);
fprintf('Landing burn ignition : %9.0f m altitude, %.0f m/s\n', M.landIgnAlt, M.landIgnSpeed);
fprintf('Touchdown             : %6.2f m/s down, %5.2f m/s sideways, %5.2f deg tilt\n', ...
        M.vVert, M.vHoriz, M.tiltDeg);
fprintf('Miss distance         : %9.2f m  (pad at %.2f km)\n', M.miss, P.xPad/1e3);
fprintf('Propellant remaining  : %9.0f kg\n', M.propLeft);
for k = 1:numel(out.notes), fprintf('Note: %s\n', out.notes{k}); end
if M.success
    fprintf('RESULT                : SUCCESSFUL LANDING\n');
else
    fprintf('RESULT                : FAILED (%s)\n', out.status);
end
fprintf('=========================================================\n\n');
end
