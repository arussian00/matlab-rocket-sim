function fig = plotHeatShield(fullS, fullB, hsS, hsB)
%PLOTHEATSHIELD  Heat flux, heat load and tile temperature for the Starship flight.
%
%   fig = plotHeatShield(fullS, fullB, hsS, hsB)
%     fullS, fullB   complete ship and booster histories (liftoff onward)
%     hsS, hsB       results from heatShield (hsB may have tileModel false)

arguments
    fullS (1,1) struct
    fullB (1,1) struct
    hsS (1,1) struct
    hsB (1,1) struct
end

octaveCompat();
cB = [0.85 0.25 0.15];
cS = [0.10 0.35 0.85];
cTile = [0.75 0.35 0.05];
cSteel = [0.20 0.45 0.30];

if hsS.pass
    verdict = sprintf('PASS, margin %.0f C', hsS.marginC);
else
    verdict = sprintf('FAIL, %.0f C over the limit', -hsS.marginC);
end

fig = figure('Name', 'Starship heat shield', 'Color', 'w', 'Position', [80 40 1100 820]);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Heat shield: peak %.0f C, heat load %.1f MJ/m^2, limit %.0f C, %s', ...
    hsS.TsurfMaxC, hsS.Qtot/1e6, hsS.limitC, verdict));

% 1. Heating rate. Same Sutton-Graves flux the temperature model is driven by.
nexttile; hold on; grid on
plot(hsB.t/60, hsB.q/1e3, '-', 'Color', cB, 'LineWidth', 1.3);
plot(hsS.t/60, hsS.q/1e3, '-', 'Color', cS, 'LineWidth', 1.3);
plot(hsS.t(hsS.iQmax)/60, hsS.qMax/1e3, 'o', 'Color', cS, 'MarkerFaceColor', cS, 'MarkerSize', 5);
xlabel('Time [min]'); ylabel('Heat flux [kW/m^2]');
title(sprintf('Windward heating (ship peak %.0f kW/m^2 at %.0f km)', ...
    hsS.qMax/1e3, fullS.X(hsS.iQmax, 2)/1e3));
legend('Super Heavy', 'Starship', 'Ship peak', 'Location', 'northwest');

% 2. Running integral of that flux.
nexttile; hold on; grid on
plot(hsB.t/60, hsB.Q/1e6, '-', 'Color', cB, 'LineWidth', 1.3);
plot(hsS.t/60, hsS.Q/1e6, '-', 'Color', cS, 'LineWidth', 1.3);
xlabel('Time [min]'); ylabel('Heat load [MJ/m^2]');
title(sprintf('Total heat load, integral of q (ship %.3e J/m^2, booster %.3e J/m^2)', ...
    hsS.Qtot, hsB.Qtot));
legend('Super Heavy', 'Starship', 'Location', 'northwest');

% 3. Tile temperatures against the reuse limit.
nexttile; hold on; grid on
plot(hsS.t/60, hsS.Trad - 273.15, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.1);
plot(hsS.t/60, hsS.Tback - 273.15, '-', 'Color', cSteel, 'LineWidth', 1.3);
plot(hsS.t/60, hsS.Tsurf - 273.15, '-', 'Color', cTile, 'LineWidth', 1.6);
plot(hsS.tTmax/60, hsS.TsurfMaxC, 'o', 'Color', cTile, 'MarkerFaceColor', cTile, 'MarkerSize', 5);
yline(hsS.limitC, '--', sprintf('%.0f C tile limit', hsS.limitC), ...
    'Color', [0.65 0.10 0.10], 'LineWidth', 1.1, 'LabelHorizontalAlignment', 'left');
xlabel('Time [min]'); ylabel('Temperature [^{\circ}C]');
title(sprintf('Tile surface %.0f C (radiative equilibrium %.0f C), steel skin %.0f C', ...
    hsS.TsurfMaxC, hsS.TradMaxC, hsS.TbackMaxC));
legend('Radiative equilibrium', 'Steel skin', 'Tile surface', 'Peak surface', ...
    'Location', 'northwest');
tempC = [hsS.Trad; hsS.Tsurf; hsS.Tback] - 273.15;
ylim([min(0, min(tempC)) - 40, max([hsS.limitC, max(tempC)]) + 80]);
end
