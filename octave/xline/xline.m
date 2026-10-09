function h = xline(varargin)
%XLINE  Octave stand-in for MATLAB's xline (see octaveCompat.m).
%   xline(x), xline(x, linespec), xline(x, linespec, label), and the same
%   with an axes first. The line always spans the axes, even after more
%   data is plotted (a listener follows the y-limits).
[ax, args] = splitAxes(varargin);
x = args{1};  spec = '-';  label = '';
if numel(args) >= 2, spec = args{2}; end
if numel(args) >= 3, label = args{3}; end
held = ishold(ax);  hold(ax, 'on');
yl = ylim(ax);
h = plot(ax, [x x], yl, spec, 'HandleVisibility', 'off');
if ~any(ismember(spec, 'rgbcmykw')), set(h, 'Color', [0.15 0.15 0.15]); end
t = text(ax, x, yl(2), [' ' label], 'HorizontalAlignment', 'left', ...
         'VerticalAlignment', 'top', 'FontSize', 8, 'HandleVisibility', 'off');
addlistener(ax, 'ylim', @(~, ~) follow(ax, h, t, x));
if ~held, hold(ax, 'off'); end
end

function follow(ax, h, t, x)
if ~ishghandle(h), return; end
yl = ylim(ax);
set(h, 'YData', yl);
set(t, 'Position', [x yl(2) 0]);
end

function [ax, args] = splitAxes(args)
if ~isempty(args) && isscalar(args{1}) && ishghandle(args{1}) && strcmp(get(args{1}, 'Type'), 'axes')
    ax = args{1};  args = args(2:end);
else
    ax = gca;
end
end
