function octaveCompat()
%OCTAVECOMPAT  Keep MATLAB on its own functions, and help GNU Octave along.
%
%   Called at the start of every script and plotting function.
%
%   In MATLAB this does not add anything and does not change numerics or
%   graphics. It only takes octave/ stand-in folders back OFF the path if
%   something (typically addpath(genpath(...))) put them there, so names
%   such as tiledlayout, yline and yyaxis stay MATLAB's own functions.
%   It is cheap and runs on every call, not just the first.
%
%   In Octave it adds those stand-ins, once, for the plotting commands
%   Octave does not have (tiledlayout, nexttile, yline, xline, yyaxis,
%   histogram, exportgraphics). Each one is added only when that Octave
%   is missing the real command. This path is best-effort. MATLAB is the
%   environment the project is written for.

if ~exist('OCTAVE_VERSION', 'builtin')
    stripOctaveShims(fullfile(fileparts(mfilename('fullpath')), 'octave'));
    return
end

persistent armed
if ~isempty(armed)
    return
end
armed = true;

base  = fullfile(fileparts(mfilename('fullpath')), 'octave');
names = {'tiledlayout', 'nexttile', 'yline', 'xline', 'yyaxis', 'histogram', 'exportgraphics'};
warning('off', 'Octave:legend:unimplemented-location');   % 'best' -> 'northeast' is fine
for k = 1:numel(names)
    if ~exist(names{k})                                  %#ok<EXIST> any kind of definition
        addpath(fullfile(base, names{k}));
    end
end
end

function stripOctaveShims(root)
% Remove every path entry inside the octave/ folder. rmpath on a folder
% that is not on the path errors, so check first.
if ~isfolder(root)
    return
end
rootPrefix = [root filesep];
entries = strsplit(path, pathsep);
removed = false;
for k = 1:numel(entries)
    folder = entries{k};
    if strcmp(folder, root) || startsWith(folder, rootPrefix)
        rmpath(folder);
        removed = true;
    end
end
if removed
    rehash path
end
end
