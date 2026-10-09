function ax = nexttile()
%NEXTTILE  Octave stand-in for MATLAB's nexttile (see octaveCompat.m).
%   Places a new axes in the next cell of the grid set up by tiledlayout,
%   filling rows left to right, top to bottom.
fig = gcf;
g = getappdata(fig, 'tiledGrid');
if isempty(g), g = [1 1 0]; end
g(3) = g(3) + 1;
setappdata(fig, 'tiledGrid', g);
r = g(1);  c = g(2);  k = g(3);
row = ceil(k / c);  col = k - (row - 1)*c;
cw = 0.92 / c;  ch = 0.88 / r;                          % cell size
pos = [0.05 + (col - 1)*cw + 0.06*cw, 0.05 + (r - row)*ch + 0.14*ch, 0.82*cw, 0.70*ch];
ax = axes('Parent', fig, 'Position', pos);
end
