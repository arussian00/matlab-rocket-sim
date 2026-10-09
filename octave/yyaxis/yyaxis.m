function yyaxis(varargin)
%YYAXIS  Octave stand-in for MATLAB's yyaxis (see octaveCompat.m).
%   yyaxis left / yyaxis right (optionally with an axes first). The right
%   side is a transparent second axes laid over the first, with its y-axis
%   on the right and its x-axis linked to the left one. Whichever side you
%   pick becomes the current axes, so plot, ylabel, ylim ... act on it.
if numel(varargin) == 2, ax = varargin{1}; side = varargin{2};
else,                    ax = gca;         side = varargin{1};
end
base = getappdata(ax, 'yyBase');
if isempty(base), base = ax; end
right = getappdata(base, 'yyRight');
if isempty(right) || ~ishghandle(right)
    fig = ancestor(base, 'figure');
    right = axes('Parent', fig, 'Position', get(base, 'Position'), 'Color', 'none', ...
                 'YAxisLocation', 'right', 'XTickLabel', [], 'Box', 'off', ...
                 'NextPlot', 'add');          % so plotting doesn't reset these settings
    co = get(base, 'ColorOrder');
    set(right, 'YColor', co(2,:), 'ColorOrder', co(2:end,:));
    set(base,  'YColor', co(1,:), 'Box', 'off');
    setappdata(right, 'yyBase', base);
    setappdata(base, 'yyRight', right);
    % Share the time axis. (linkaxes would freeze the limits as they are
    % now, before any data is plotted, so follow the left axis instead.)
    addlistener(base,  'xlim', @(~, ~) syncX(base, right));
    addlistener(right, 'xlim', @(~, ~) syncX(right, base));
end
if strcmpi(side, 'right'), set(ancestor(base, 'figure'), 'CurrentAxes', right);
else,                      set(ancestor(base, 'figure'), 'CurrentAxes', base);
end
end

function syncX(from, to)
if ishghandle(from) && ishghandle(to) && ~isequal(xlim(from), xlim(to))
    set(to, 'XLim', xlim(from));
end
end
