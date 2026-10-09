function h = histogram(x, nbins)
%HISTOGRAM  Octave stand-in for MATLAB's histogram (see octaveCompat.m).
%   histogram(x) or histogram(x, nbins): bar chart of counts.
if nargin < 2, nbins = 10; end
[counts, centres] = hist(x(:), nbins);
h = bar(centres, counts, 1);
end
