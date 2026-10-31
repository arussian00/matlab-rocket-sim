function varargout = dropOctaveShim(name, varargin)
%DROPOCTAVESHIM  Call the real MATLAB function when an Octave stand-in is in front of it.
%
%   The files under octave/ exist so GNU Octave can imitate a few plotting
%   commands. They must not run in MATLAB. If one does (its folder got onto
%   the path), this takes that folder off the path and calls MATLAB's own
%   function of the same name.
%
%   If the stand-in is still the first match afterwards, this errors rather
%   than calling itself again. Run "clear functions" and take octave/ off
%   the path if that happens.

if exist('OCTAVE_VERSION', 'builtin')
    error('dropOctaveShim:octave', ...
        'dropOctaveShim is only for MATLAB. In Octave, call %s directly.', name);
end

removed = stripShimFolders(name);
if removed
    rehash path
end

hits = which(name, '-all');
if ischar(hits) || isstring(hits)
    hits = cellstr(hits);
end
if isempty(hits) || isShimPath(hits{1})
    error('dropOctaveShim:shadow', ...
        ['%s still resolves to the Octave stand-in. Remove every octave/ ' ...
         'folder from the MATLAB path, then run "clear functions".'], name);
end

fh = str2func(name);
if nargout == 0
    fh(varargin{:});
else
    [varargout{1:nargout}] = fh(varargin{:});
end
end

function removed = stripShimFolders(name)
hits = which(name, '-all');
if ischar(hits) || isstring(hits)
    hits = cellstr(hits);
end
removed = false;
for k = 1:numel(hits)
    if ~isShimPath(hits{k})
        continue
    end
    rmpath(fileparts(hits{k}));
    removed = true;
end
end

function tf = isShimPath(p)
tf = contains(p, [filesep 'octave' filesep]);
end
