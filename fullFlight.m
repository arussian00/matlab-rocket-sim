function full = fullFlight(segs, names, ret)
%FULLFLIGHT  Put the launch in front of a booster's return flight.
%
%   full = fullFlight({seg1, seg2}, {'name1', 'name2'}, out)
%     segs   segments from pointMassSegment.m (e.g. the stack ascent)
%     names  a phase name for each segment
%     ret    a simulateBooster result that starts where the segments end
%
%   full has the same fields as ret, but its history starts on the launch
%   pad, so flightMetrics, plotFlight and animateFlight show the whole
%   flight instead of starting mid-air at stage separation.

full = ret;
full.phaseList  = [names, ret.phaseList];
full.phaseStart = [cellfun(@(s) s.t(1), segs), ret.phaseStart];
T = []; X = []; PH = [];
f = fieldnames(ret.aux);
A = cell2struct(cell(numel(f), 1), f, 1);
for k = 1:numel(segs)
    T  = [T;  segs{k}.t];                          %#ok<AGROW>
    X  = [X;  segs{k}.X];                          %#ok<AGROW>
    PH = [PH; k*ones(numel(segs{k}.t), 1)];        %#ok<AGROW>
    for j = 1:numel(f), A.(f{j}) = [A.(f{j}); segs{k}.aux.(f{j})]; end
end
full.t     = [T;  ret.t];
full.X     = [X;  ret.X];
full.phase = [PH; ret.phase + numel(segs)];
for j = 1:numel(f), full.aux.(f{j}) = [A.(f{j}); ret.aux.(f{j})]; end
end
