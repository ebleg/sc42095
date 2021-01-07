%% REDESIGN CONTROLLERS FOR INPUT SATURATIION

close all; clearvars; clc;

s = tf('s');

run plot_settings;

%% Plant
Gc = tf([1 20], [1 24 144 0]);
Gcss = canon(ss(Gc), 'companion');

load 'controllers/K1'
load 'controllers/PIDstruct'

h = 0.05;
Gd = canon(c2d(Gcss, h, 'zoh'), 'companion');

%% PD redesign
K1_v2 = K1.it5;
K1_v2.Kp = 20/(K1_v2.N + 1);
% K_PD_v2.Td1 = 20*K_PD_v2.Td1;
% K_PD_v2.Ti = 2;
K1_v2.Kp = 20;
K1_v2.Td1 = 0;

K1_v2 = build_PIDD(K1_v2);

figure
sim_with_input([feedback(K1_v2.Kp*Gd, 1); feedback(K1_v2.Kp, Gd)], ...
                0:h:1.2);
sim_with_input([feedback(K1_v2.tf*Gc, 1); feedback(K1_v2.tf, Gc)], ...
                linspace(0, 1, 5e2));
legend({'DT system output', 'CT system output', 'CT controller effort',  ...
        'DT controller effort'}, 'location', 'best');
title('\textbf{Redesigned P-controller (servo)}')
% exportgraphics(gcf, '../tex/media/q9/pdredesign.eps');

%% Pole placement redesign
OS_max = 1;
zeta_target = -log(OS_max/100)/sqrt(pi^2 + log(OS_max/100)^2)*0.965;
Ts_target = 1.045;
w = -log(0.01*sqrt(1-zeta_target^2))/Ts_target/zeta_target;
% w = 5.5; 
h = 0.3/1.3/w;

target_pole = complex(-zeta_target*w, sqrt(1 - zeta_target^2)*w);
poles_fb = [target_pole, conj(target_pole), -1.4*w];

Gdss = canon(c2d(Gcss, h, 'zoh'), 'canonical');

L = place(Gdss.A, Gdss.B, exp(h*poles_fb));
Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B); % FFW gain for correct ss value
sys = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, [Gdss.C; -L], [Gdss.D; Lc], h);

figure; hold on;
tin = 0:h:1.2;
sim_with_input(sys, tin);
title('\textbf{Pole placement controller}')
fprintf('Settling time: %.3g\n', ...
    stepinfo(sys(1,1), 'SettlingTimeThreshold', 0.01).SettlingTime);
fprintf('Overshoot: %.3g\n', ...
    stepinfo(sys(1,1), 'SettlingTimeThreshold', 0.01).Overshoot);
title('\textbf{Redesigned full-information controller}')
% exportgraphics(gcf, '../tex/media/q10/ppredesign.eps');

%% Output feedback
% poles_fb2 = 1.132*poles_fb;
poles_fb2 = poles_fb;
L = place(Gdss.A, Gdss.B, exp(h*poles_fb2));
K = place(Gdss.A', Gdss.C', exp(h*poles_fb2.*1.3))';

Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B);

% Generalised plant
% IN: [r u] OUT: [y u u y r] --> extend with extra input 
Gdss_ext = ss(Gdss.A, ... % A 
              [zeros(3, 1) Gdss.B], ... % B 
              [Gdss.C; [0 0 0]; Gdss.C; [0 0 0]], ... % C
              [[0 0]; [0 1]; [0 0]; [1 0]], h, ... % D
              'InputName', {'r', 'u'}, 'OutputName', {'y', 'u', 'y', 'r'});

% Output feedback controller
% IN: [u y r] OUT: [u] 
K_obs = ss(Gdss.A - K*Gdss.C, ... % A
           [Gdss.B K zeros(3, 1)], ... % B
           repmat(-L, 2, 1), ... % C
           repmat([0 0 Lc], 2, 1), ... % D
           h, 'InputName', {'u', 'y', 'r'}, 'OutputName', {'u', 'u'});

sys = lft(Gdss_ext, K_obs, 1, 3);

figure; hold on;
tin = 0:h:1.2;
sim_with_input(sys, tin, [0 0 0 -2 3 5]*10);
title('\textbf{Redesigned output feedback controller (servo)}')
% exportgraphics(gcf, '../tex/media/q10/outppredesign.eps');

%% LQR design
Q0 = Gdss.C'*Gdss.C;
Q = 428*Q0;
R = 1;
L = dlqr(Gdss.A, Gdss.B, Q, R);
Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B);

sys = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, [Gdss.C; -L], [Gdss.D; Lc], h);

figure; hold on;
tin = 0:h:2;
sim_with_input(sys, tin);
title('\textbf{Pole placement controller}')
fprintf('Settling time: %.3g\n', ...
    stepinfo(sys(1,1), 'SettlingTimeThreshold', 0.01).SettlingTime);
fprintf('Overshoot: %.3g\n', ...
    stepinfo(sys(1,1), 'SettlingTimeThreshold', 0.01).Overshoot);
title('\textbf{Redesigned LQR controller (servo)}')
fprintf('Settling time: %.3g\n', ...
    stepinfo(sys(1,1), 'SettlingTimeThreshold', 0.01).SettlingTime);
fprintf('Overshoot: %.3g\n', ...
    stepinfo(sys(1,1), 'SettlingTimeThreshold', 0.01).Overshoot);
exportgraphics(gcf, '../tex/media/q11/lqrredesign.eps');
