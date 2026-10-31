function h = histogram(varargin)
%HISTOGRAM  Octave stand-in for MATLAB's histogram (see octaveCompat.m).
%   histogram(x) or histogram(x, nbins): bar chart of counts.
%   In MATLAB this file must not run: the call is handed to MATLAB's histogram.
if ~exist('OCTAVE_VERSION', 'builtin')
    if nargout == 0
        dropOctaveShim('histogram', varargin{:});
    else
        h = dropOctaveShim('histogram', varargin{:});
    end
    return
end
x = varargin{1};
if numel(varargin) < 2 || isempty(varargin{2})
    nbins = 10;
else
    nbins = varargin{2};
end
[counts, centres] = hist(x(:), nbins);
h = bar(centres, counts, 1);
end
