%% ROCKET_HOP_SIM  Launch to space, fly back, land on the pad (single run)
%
% This is the main entry point. It:
%   1. loads the vehicle parameters             (rocket_params.m)
%   2. flies the mission                        (simulateBooster.m)
%      ascent > flip > boostback > coast/re-entry > landing burn
%   3. prints a flight summary                  (flightMetrics.m)
%   4. draws a 9-panel dashboard                (plotFlight.m)
%   5. plays an animation                       (animateFlight.m)
%
% Runs on MATLAB Online Basic: core MATLAB only, no toolboxes.
% Keep ALL the .m files from this project in the same folder.
%
% Other entry points:
%   rocket_monte_carlo     - 50+ flights with random errors -> landing statistics
%   rocket_two_stage_sim   - two-stage rocket to orbit + booster drone-ship landing
%   rocket_app             - interactive app with sliders

clear; close all; clc
octaveCompat();          % also runs in GNU Octave (no-op in MATLAB)

%% STEP 1 - Load parameters (edit rocket_params.m, or override here)
P = rocket_params('hop');

% Examples of overrides - uncomment to experiment:
% P.kickDeg  = 8;      % lean further -> lands farther away -> longer boostback
% P.wind     = 15;     % 15 m/s wind at 10 km
% P.thrMin   = 0.5;    % shallower throttling -> harder landing
% P.h_target = 150e3;  % fly higher

%% STEP 2 - Fly the mission
tic
out = simulateBooster(P);
fprintf('Simulation took %.1f s of computer time.\n', toc);

%% STEP 3 - Summary
M = flightMetrics(out);

%% STEP 4 - Plots
plotFlight(out);

%% STEP 5 - Animation
playAnimation = true;   % set false to skip
speedup       = 10;     % 10x real time
saveGif       = false;  % true -> writes rocket_hop.gif to your MATLAB Drive (slow)

if playAnimation
    if saveGif, gifName = 'rocket_hop.gif'; else, gifName = ''; end
    animateFlight(out, speedup, 25, gifName);
end
