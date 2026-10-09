function exportgraphics(varargin)
%EXPORTGRAPHICS  Octave stand-in (see octaveCompat.m): GIF saving needs MATLAB.
%   Warns once and does nothing, so animations still play.
persistent warned
if isempty(warned)
    warning('octaveCompat:gif', 'Saving a GIF needs MATLAB R2022a+; skipping it in Octave.');
    warned = true;
end
end
