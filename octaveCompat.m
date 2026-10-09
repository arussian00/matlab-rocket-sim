function octaveCompat()
%OCTAVECOMPAT  Make this project run in GNU Octave as well as MATLAB.
%
%   Called at the start of every script and plotting function. In MATLAB
%   it does nothing. In Octave it adds small stand-ins for the MATLAB
%   plotting commands that Octave lacks (tiledlayout, nexttile, yline,
%   xline, yyaxis, histogram, exportgraphics). Each stand-in lives in its
%   own subfolder of octave/ and is only added if this Octave version
%   doesn't already have the real thing.

persistent done
if ~isempty(done), return; end
done = true;
if ~exist('OCTAVE_VERSION', 'builtin'), return; end    % MATLAB: nothing to do

base  = fullfile(fileparts(mfilename('fullpath')), 'octave');
names = {'tiledlayout', 'nexttile', 'yline', 'xline', 'yyaxis', 'histogram', 'exportgraphics'};
warning('off', 'Octave:legend:unimplemented-location');   % 'best' -> 'northeast' is fine
for k = 1:numel(names)
    if ~exist(names{k})                                  %#ok<EXIST> any kind of definition
        addpath(fullfile(base, names{k}));
    end
end
end
