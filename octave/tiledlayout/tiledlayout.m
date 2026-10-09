function tl = tiledlayout(r, c, varargin)
%TILEDLAYOUT  Octave stand-in for MATLAB's tiledlayout (see octaveCompat.m).
%   Remembers the grid size for nexttile, and returns an invisible strip
%   across the top of the figure so that title(tl, ...) still works.
%   Spacing options are ignored.
fig = gcf;
setappdata(fig, 'tiledGrid', [r c 0]);
% (Octave hides the title of an invisible axes, so keep it "visible" but bare)
tl = axes('Parent', fig, 'Position', [0.05 0.955 0.9 0.001], 'Color', 'none', ...
          'XColor', 'none', 'YColor', 'none', 'XTick', [], 'YTick', [], 'HitTest', 'off');
set(fig, 'CurrentAxes', tl);
end
