function C = phaseColors(n)
%PHASECOLORS  n distinct line colours (one per flight phase).
%   Same colours as MATLAB's lines(n), taken from the default colour
%   order. (In Octave, lines() reads the *current* axes and can open an
%   empty figure, so the project uses this instead.)
co = get(0, 'DefaultAxesColorOrder');
C  = co(mod(0:n-1, size(co, 1)) + 1, :);
end
