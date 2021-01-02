%% Plot settings
customhexcolors = {'2ecc71', '3498db',  'f1c40f', 'e74c3c', '34495e',  ...
                   'e67e22', '9b59b6', '7f8c8d', }; 
customcolors = zeros(length(customhexcolors), 3);
for i = 1:length(customhexcolors)
   customcolors(i,:) = hex2dec({customhexcolors{i}(1:2), ...
                                customhexcolors{i}(3:4), ...
                                customhexcolors{i}(5:6)})./255;
end

set(groot, 'defaultAxesColorOrder', customcolors)
% set(groot, 'defaultAxesLineStyleOrder', {'-x', '-^', '-v', '->', '-<'})
set(groot, 'defaultAxesLineWidth', 1)
set(groot, 'defaultAxesXGrid', 'on');
set(groot, 'defaultAxesYGrid', 'on');
% set(groot, 'defaultAxesFontName', 'Lucida Sans');
set(groot,'defaultTextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultLegendFontSize', 10);
set(groot, 'defaultAxesFontSize', 10);
set(groot, 'defaultLineLineWidth', 1.5);
set(groot, 'defaultStairLineWidth', 1.5)
set(groot, 'defaultAxesLabelFontSizeMultiplier', 1.1);
set(groot, 'defaultAxesTitleFontSizeMultiplier', 1.15);
set(groot, 'defaultAxesTitleFontWeight', 'bold');
set(groot, 'defaultAxesBox', 'on');
set(groot, 'defaultAxesTickDir', 'both');
set(groot, 'defaultAxesTickLength', [0.005 0.005]);
set(groot,  'defaultAxesTickDirMode', 'manual');

% Figure settings
set(groot, 'defaultFigurePaperUnits', 'centimeters');
set(groot, 'defaultFigureUnits', 'centimeters');
set(groot, 'defaultFigureInvertHardcopy', 'off');
set(groot, 'defaultFigureColor', [1 1 1]);

% Position & size
set(groot, 'defaultFigurePosition', [10 10 12 8]);
set(groot, 'defaultAxesPosition', [0.1, 0.13, 0.85, 0.77]);

set(groot, 'defaultTiledLayoutTileSpacing', 'compact')
set(groot, 'defaultTiledLayoutPadding', 'compact')