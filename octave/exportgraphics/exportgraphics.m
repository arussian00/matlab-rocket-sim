function exportgraphics(varargin)
%EXPORTGRAPHICS  Octave stand-in (see octaveCompat.m): GIF saving needs MATLAB.
%   Warns once and does nothing, so animations still play.
%   In MATLAB this file must not run: the call is handed to MATLAB's exportgraphics.
if ~exist('OCTAVE_VERSION', 'builtin')
    dropOctaveShim('exportgraphics', varargin{:});
    return
end
persistent warned
if isempty(warned)
    warning('octaveCompat:gif', 'Saving a GIF needs MATLAB R2022a+; skipping it in Octave.');
    warned = true;
end
end
