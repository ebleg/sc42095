
close all; clearvars; clc;

s = tf('s');

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

%% Plant
Gc = tf([1 20], [1 24 144 0]);
Gcss = canon(ss(Gc), 'companion');

%% Q8: Control effort simulations

%% Reference tracking PDD
load 'controllers/K1'
h = K1.it6.Td2*0.7/10;
K = c2d(K1.it6.tf, h, 'tustin');
Gd = c2d(Gc, h, 'zoh');
sys = [feedback(K*Gd, 1); feedback(K, Gd)];

figure; hold on;
tin = 0:h:0.05;
sim_with_input(sys, tin);
title('\textbf{PDD controller (reference tracking)}')
exportgraphics(gcf, '../tex/media/q8/pdd.eps');

%% Reference tracking PD
h = K1.it5.Td1*0.7/10;
K = c2d(K1.it5.tf, h, 'tustin');
Gd = c2d(Gc, h, 'zoh');
sys = [feedback(K*Gd, 1); feedback(K, Gd)];

figure; hold on;
tin = 0:h:0.5;
sim_with_input(sys, tin);
title('\textbf{PD controller (reference tracking)}')
exportgraphics(gcf, '../tex/media/q8/pd.eps');

%% Disturbance rejection PIDD
load 'controllers/K2'
h = 15*K2.it5.Td1*0.2/10;
K = c2d(K2.it5.tf, h, 'tustin');
Gd = c2d(Gc, h, 'tustin');
sys = [feedback(Gd, K); feedback(1, K*Gd)];

figure; hold on;
tin = 0:h:0.5;
sim_with_input(sys, tin);
title('\textbf{PIDD controller (disturbance rejection)}')
exportgraphics(gcf, '../tex/media/q8/pidd.eps');

%% Pole placement controller: full state
OS_max = 1;
zeta_target = -log(OS_max/100)/sqrt(pi^2 + log(OS_max/100)^2);
w = 20; 
h = 0.5/1.3/w;

target_pole = complex(-zeta_target*w, sqrt(1 - zeta_target)*w);

Gdss = canon(c2d(Gcss, h, 'zoh'), 'canonical');

L = place(Gdss.A, Gdss.B, exp(h*[target_pole, conj(target_pole), -1.3*w]));
Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B); % FFW gain for correct ss value
sys = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, [Gdss.C; -L], [Gdss.D; 0], h);

figure; hold on;
tin = 0:h:0.5;
sim_with_input(sys, tin);
title('\textbf{Pole placement controller}')
exportgraphics(gcf, '../tex/media/q8/fullstate.eps');

%% Output controller: servo
poles_fb = [target_pole, conj(target_pole), -1.3*w];
L = place(Gdss.A, Gdss.B, exp(h*poles_fb));
K = place(Gdss.A', Gdss.C', exp(h*poles_fb.*1.3))';

Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B);

% Generalised plant
% IN: [r u] OUT: [y u u y r] --> extend with extra input 
Gdss_ext = ss(Gdss.A, ... % A 
              [zeros(3, 1) Gdss.B], ... % B 
              [Gdss.C; [0 0 0]; [0 0 0]; Gdss.C; [0 0 0]], ... % C
              [[0 0]; [0 1]; [0 1]; [0 0]; [1 0]], h, ... % D
              'InputName', {'r', 'u'}, 'OutputName', {'y', 'u', 'u', 'y', 'r'});

% Output feedback controller
% IN: [u y r] OUT: [u] 
K_obs = ss(Gdss.A - K*Gdss.C, ... % A
           [Gdss.B K zeros(3, 1)], ... % B
           -L, ... % C
           [0 0 Lc], ... % D
           h, 'InputName', {'u', 'y', 'r'}, 'OutputName', 'u');

sys = lft(Gdss_ext, K_obs);

figure; hold on;
tin = 0:h:0.5;
sim_with_input(sys, tin);
title('\textbf{Output feedback controller (servo)}')
exportgraphics(gcf, '../tex/media/q8/outputservo.eps');

%% Output disturbance rejection 
% poles_fb = [-8 -12.01 -12];
L = place(Gdss.A, Gdss.B, exp(h*poles_fb));
K = place([Gdss.A Gdss.B; [0 0 0 1]]', [Gdss.C 0]', ...
                                     exp(h*[poles_fb, -7]*2))';
% Extended system
% IN: [d u] OUT: [y u u y];
Gdss_ext = ss(Gdss.A, ...
              [Gdss.B Gdss.B], ...
              [Gdss.C; [0 0 0]; [0 0 0]; Gdss.C], ...
              [[0 0]; [0 1]; [0 1]; [0 0]], h, ...
              'InputName', {'v', 'u'}, ...
              'OutputName', {'y', 'u', 'u', 'y'});

K_obs = ss([[Gdss.A; [0 0 0]] - K*Gdss.C, [Gdss.B; 1]], ... % A
           [[Gdss.B; 0], K], ... % B
           [-L -1], ... % C
           0, h, ... % D
           'InputName', {'u', 'y'}, 'OutputName', 'u');

sys = lft(Gdss_ext, K_obs);

figure; hold on;
tin = 0:h:0.8;
sim_with_input(sys, tin);
title('\textbf{Output feedback controller (disturbance rejection)}')
exportgraphics(gcf, '../tex/media/q8/outputdistrej.eps');

%% LQR Controller
R = 1; Q = Gdss.C'*Gdss.C*5e4;
[L, ~, lqr_poles]= dlqr(Gdss.A, Gdss.B, Q, R);
Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B);
sys = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, [Gdss.C; -L], Gdss.D, h);

figure; hold on;
tin = 0:h:0.8;
sim_with_input(sys, tin);
title('\textbf{LQR controller}')
exportgraphics(gcf, '../tex/media/q8/lqr.eps');

%%
function ax = sim_with_input(sys, tin)
    set(gcf, 'Position', get(gcf, 'Position').*[1 1 1.3 0.8]);
    specialstep(sys(1,:), tin);
    ylabel('System output');
    yyaxis right
    u = specialstep(sys(2,:), tin);
    set(gca, 'Position', get(gca, 'Position').*[1.05 1.1 0.9 0.9]);
    set(gca, 'YColor', 'black')
    ylabel('Controller effort')
    
    [~, max_input_idx] = max(abs(u));
    scatter(tin(max_input_idx), u(max_input_idx), 50, 's', 'filled');
    subtitle(sprintf('$u_\\mathrm{max}$ = %.4g', u(max_input_idx)));
    legend('System output', 'Controller effort', 'Location', 'best')
end