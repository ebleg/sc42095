
close all; clearvars; clc;

s = tf('s');

run plot_settings;

%% Plant
Gc = tf([1 20], [1 24 144 0]);
Gcss = canon(ss(Gc), 'companion');

%% Q8: Control effort simulations

%% Reference tracking PDD
load 'controllers/K1'
load 'controllers/PIDstruct'
h = K1.it6.Td2*0.7/10;
K = c2d(K1.it6.tf, h, 'tustin');
Gd = c2d(Gc, h, 'zoh');
sys = [feedback(K*Gd, 1); feedback(K, Gd)];

figure; hold on;
tin = 0:h:0.05;
sim_with_input(sys, tin);
sim_with_input([feedback(K1.it6.tf*Gc, 1); feedback(K1.it6.tf, Gc)], ...
               linspace(0, max(tin)));
legend({'DT system output', 'CT system output', 'DT controller effort',  ...
   'CT controller effort'}, 'location', 'east');
title('\textbf{PDD controller (reference tracking)}');
exportgraphics(gcf, '../tex/media/q8/pdd.eps');

%% Reference tracking PD
h = K1.it5.Td1*0.7/10;

K = c2d(K1.it5.tf, h, 'tustin');
Gd = c2d(Gc, h, 'zoh');
sys = [feedback(K*Gd, 1); feedback(K, Gd)];

figure; hold on;
tin = 0:h:0.5;
sim_with_input(sys, tin);
sim_with_input([feedback(K1.it5.tf*Gc, 1); feedback(K1.it5.tf, Gc)], ...
               linspace(0, max(tin)));
title('\textbf{PD controller (servo)}')
legend({'DT system output', 'CT system output', 'DT controller effort',  ...
        'CT controller effort'}, 'location', 'east');
exportgraphics(gcf, '../tex/media/q8/pd.eps');

%% Disturbance rejection PIDD
load 'controllers/K2'
h = 15*K2.it5.Td1*0.2/10;
K = c2d(K2.it5.tf, h, 'tustin');
Gd = c2d(Gc, h, 'tustin');
sys = [feedback(Gd, K); -feedback(K*Gd, 1)];

figure; hold on;
tin = 0:h:0.5;
sim_with_input(sys, tin);
sim_with_input([feedback(Gc, K2.it5.tf); -feedback(Gc*K2.it5.tf, 1)], ...
               linspace(0, max(tin)));
legend({'DT system output', 'CT system output', 'DT controller effort',  ...
   'CT controller effort'}, 'location', 'best');
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
sys = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, [Gdss.C; -L], [Gdss.D; Lc], h);

figure; hold on;
tin = 0:h:0.5;
sim_with_input(sys, tin);
title('\textbf{Pole placement controller}')
exportgraphics(gcf, '../tex/media/q8/fullstate.eps');
% 

%% Output controller: servo
poles_fb = [target_pole, conj(target_pole), -1.3*w];
L = place(Gdss.A, Gdss.B, exp(h*poles_fb));
K = place(Gdss.A', Gdss.C', exp(h*poles_fb.*1.3))';

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
tin = 0:h:1;
sim_with_input(sys, tin, [0 0 0 -2 3 5]*10);
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
              [Gdss.C; [0 0 0]; Gdss.C], ...
              [[0 0]; [0 1]; [0 0]], h, ...
              'InputName', {'v', 'u'}, ...
              'OutputName', {'y', 'u', 'y'});

K_obs = ss([[Gdss.A; [0 0 0]] - K*Gdss.C, [Gdss.B; 1]], ... % A
           [[Gdss.B; 0], K], ... % B
           repmat([-L -1], 2, 1), ... % C
           zeros(2), h, ... % D
           'InputName', {'u', 'y'}, 'OutputName', {'u', 'u'});

sys = lft(Gdss_ext, K_obs, 1, 2);

figure; hold on;
tin = 0:h:0.8;
sim_with_input(sys, tin, [0 0 0 1 3 5 -1]);
title('\textbf{Output feedback controller (disturbance rejection)}')
exportgraphics(gcf, '../tex/media/q8/outputdistrej.eps');


%% LQR Controller
R = 1; Q = Gdss.C'*Gdss.C*5e4;
[L, ~, lqr_poles]= dlqr(Gdss.A, Gdss.B, Q, R);
Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B);
sys = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, [Gdss.C; -L], [Gdss.D; Lc], h);

figure; hold on;
tin = 0:h:0.8;
sim_with_input(sys, tin);
title('\textbf{LQR controller}')
exportgraphics(gcf, '../tex/media/q8/lqr.eps');
