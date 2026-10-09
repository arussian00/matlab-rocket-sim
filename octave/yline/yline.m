function h = yline(varargin)
%YLINE  Octave stand-in for MATLAB's yline (see octaveCompat.m).
%   yline(y), yline(y, linespec), yline(y, linespec, label), and the same
%   with an axes first. The line always spans the axes, even after more
%   data is plotted (a listener follows the x-limits).
[ax, args] = splitAxes(varargin);
y = args{1};  spec = '-';  label = '';
if numel(args) >= 2, spec = args{2}; end
if numel(args) >= 3, label = args{3}; end
held = ishold(ax);  hold(ax, 'on');
xl = xlim(ax);
h = plot(ax, xl, [y y], spec, 'HandleVisibility', 'off');
if ~any(ismember(spec, 'rgbcmykw')), set(h, 'Color', [0.15 0.15 0.15]); end
t = text(ax, xl(2), y, [label ' '], 'HorizontalAlignment', 'right', ...
         'VerticalAlignment', 'bottom', 'FontSize', 8, 'HandleVisibility', 'off');
addlistener(ax, 'xlim', @(~, ~) follow(ax, h, t, y));
if ~held, hold(ax, 'off'); end
end

function follow(ax, h, t, y)
if ~ishghandle(h), return; end
xl = xlim(ax);
set(h, 'XData', xl);
set(t, 'Position', [xl(2) y 0]);
end

function [ax, args] = splitAxes(args)
if ~isempty(args) && isscalar(args{1}) && ishghandle(args{1}) && strcmp(get(args{1}, 'Type'), 'axes')
    ax = args{1};  args = args(2:end);
else
    ax = gca;
end
end
