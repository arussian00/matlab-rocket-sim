function C = phaseColors(n)
%PHASECOLORS  n distinct line colours, one per flight phase.
%   In MATLAB this is lines(n), MATLAB's own colour map. Octave's lines()
%   reads the current axes and can open an empty figure, so there the
%   default axes colour order is copied out by hand. The colours match.
if ~exist('OCTAVE_VERSION', 'builtin')
    C = lines(n);
    return
end
co = get(0, 'DefaultAxesColorOrder');
C  = co(mod(0:n-1, size(co, 1)) + 1, :);
end
