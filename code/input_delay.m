close all; clearvars; clc;

s = tf('s');

run plot_settings;


%% Plant
Gc = tf([1 20], [1 24 144 0]);
Gcss = canon(ss(Gc), 'companion');
% Gdss = canon(c2d(Gcss, h, 'zoh'), 'canonical');

%% Tracking PD
load 'controllers/K1_v2';
h = 0.037622;
K1_v3 = K1_v2;
K1_v3.Kp = 16.75;

% h = K1_v3.Td1*0.7/10;
K1_v3 = build_PIDD(K1_v3);
K1_v3.dtf = c2d(K1_v3.tf, h, 'tustin');

Gd = c2d(Gc, h, 'zoh');
z = tf('z');
Gd_delay = Gd*1/z;

tin = 0:h:2; 

T = feedback(K1_v3.dtf*Gd, 1);
Tdelay = feedback(K1_v3.dtf*Gd_delay, 1);

figure; hold on;
sim_with_input([Tdelay; Tdelay/Gd_delay], tin);
fprintf('Settling time: %.3g\n', stepinfo(Tdelay, 'SettlingTimeThreshold', 0.01).SettlingTime);
fprintf('Overshoot: %.2g\n', stepinfo(Tdelay, 'SettlingTimeThreshold', 0.01).Overshoot);
title('\textbf{P-controller redesigned for time-delay}');
exportgraphics(gcf, '../tex/media/q13/pd_redesign.eps');

figure; hold on;
specialstep(feedback(K1_v2.Kp*Gd, 1), tin);
specialstep(feedback(K1_v2.Kp*Gd_delay, 1), tin);
ylabel('Amplitude')
legend({'Withouth delay', 'With delay'}, 'location', 'southeast');
title('\textbf{Effect of time-delay on the P-controller}')
exportgraphics(gcf, '../tex/media/q13/p_delay.eps');
            
%% Disturbance rejection PIDD
load 'controllers/K2'

h = 0.0012;
Gd = c2d(Gc, h, 'zoh');
Gd_delay = Gd*1/z;

K2_v2 = K2.it5;
K2_v2.Ti = 0.1;
K2_v2.Kp = 300;
K2_v2.Td1 = place_lead(20, K2_v2.N);
K2_v2.Td2 = place_lead(40, K2_v2.N);
% K2_v2.Td2 = 0;

K2_v2 = build_PIDD(K2_v2);
K2_v2.dtf = c2d(K2_v2.tf, h, 'tustin');

% figure
% specialbode(K2_v2.tf*Gc, 'ShowGoodies', 'on');
% 
% figure
% margin(K2_v2.dtf*Gd);
%  
% GS = feedback(Gd, K2_v2.dtf);
% GS_delay = feedback(Gd_delay, K2_v2.dtf);

% tin = 0:h:2; 
% figure
% specialstep(GS, tin); hold on;
% specialstep(GS_delay, tin);
% fprintf('Settling time: %.3g\n', stepinfo(GS, 'SettlingTimeThreshold', 0.01).SettlingTime);
% fprintf('Peak: %.2g\n', stepinfo(GS_delay, 'SettlingTimeThreshold', 0.01).Peak);

% figure
% sim_with_input([Tdelay; Tdelay/Gd_delay], ...
%                 tin);


%% Redesign pole-placement
% Influence on original controller
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
sys_delay = ss([Gdss.A Gdss.B; -L 0], [zeros(3, 1); Lc], [[Gdss.C 0]; [0 0 0 1]], 0, h);

figure; hold on;
tin = 0:h:1.5;
specialstep(sys(1,1), tin);
specialstep(sys_delay(1,1), tin);
legend({'Without delay', 'With delay'}, 'location', 'southeast')
title('\textbf{Effect of time-delay on the full-information controller}')
exportgraphics(gcf, '../tex/media/q13/full_info_delay.eps');

% Influence on disturbance rejection controller
L = place(Gdss.A, Gdss.B, exp(h*poles_fb));
K = place([Gdss.A Gdss.B; [0 0 0 1]]', [Gdss.C 0]', exp(h*[poles_fb, -7]*2))';

% Extended system
% IN: [d u] OUT: [y u u y];
Gdss_ext = ss(Gdss.A, ...
              [Gdss.B Gdss.B], ...
              [Gdss.C; [0 0 0]; Gdss.C], ...
              [[0 0]; [0 1]; [0 0]], h, ...
              'InputName', {'v', 'u'}, ...
              'OutputName', {'y', 'u', 'y'});

Gdss_ext_delay = ss([Gdss.A, Gdss.B; zeros(1, 4)], ...
              [zeros(3, 2); [1 1]], ...
              [[Gdss.C 0]; [0 0 0 0]; [Gdss.C 0]], ...
              [[0 0]; [0 1]; [0 0]], h, ...
              'InputName', {'v', 'u'}, ...
              'OutputName', {'y', 'u', 'y'});

K_obs = ss([[Gdss.A; [0 0 0]] - K*Gdss.C, [Gdss.B; 1]], ... % A
           [[Gdss.B; 0], K], ... % B
           repmat([-L -1], 2, 1), ... % C
           zeros(2), h, ... % D
           'InputName', {'u', 'y'}, 'OutputName', {'u', 'u'});

sys = lft(Gdss_ext, K_obs, 1, 2);
sys_delay = lft(Gdss_ext_delay, K_obs, 1, 2);

figure; hold on;
tin = 0:h:2;
y = lsim(sys(1,1), ones(size(tin)), tin, [0 0 0 1 3 5 -1]);
y_delay = lsim(sys_delay(1,1), ones(size(tin)), tin, [0 0 0 0 1 3 5 -1]);

stairs(tin, y);
stairs(tin, y_delay);
xlabel('Time (s)'); ylabel('Amplitude');
title('\textbf{Effect of time-delay on}')
subtitle('\textbf{output-feedback disturbance rejection}')
set(gca, 'Position', get(gca, 'Position').*[1.13 1.05 0.95 0.9]);
legend({'Without delay', 'With delay'});
exportgraphics(gcf, '../tex/media/q13/out_distrej_delay.eps');

% Redesign controller
OS_max = 1;
zeta_target = -log(OS_max/100)/sqrt(pi^2 + log(OS_max/100)^2)*1.193;
Ts_target = 1.015;
w = -log(0.01*sqrt(1-zeta_target^2))/Ts_target/zeta_target;
% w = 5.5; 
h = 0.3/1.3/w;

target_pole = complex(-zeta_target*w, sqrt(1 - zeta_target^2)*w);
poles_fb = [target_pole, conj(target_pole), -1.4*w];

Gdss = canon(c2d(Gcss, h, 'zoh'), 'canonical');

L = place(Gdss.A, Gdss.B, exp(h*poles_fb));
Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B); % FFW gain for correct ss value
sys_delay = ss([Gdss.A Gdss.B; -L 0], [zeros(3, 1); Lc], [[Gdss.C 0]; [-L 0]], [0; Lc], h);

figure; hold on;
tin = 0:h:1.2;
sim_with_input(sys_delay, tin);
title('\textbf{Full-information controller redesigned for time-delay}');
exportgraphics(gcf, '../tex/media/q13/full_info_redesign.eps');

fprintf('Settling time: %.3g\n', ...
    stepinfo(sys_delay(1,1), 'SettlingTimeThreshold', 0.01).SettlingTime);
fprintf('Overshoot: %.3g\n', ...
    stepinfo(sys_delay(1,1), 'SettlingTimeThreshold', 0.01).Overshoot);

% Influence on disturbance rejection controller
L = place(Gdss.A, Gdss.B, exp(h*poles_fb));
K = place([Gdss.A Gdss.B; [0 0 0 1]]', [Gdss.C 0]', exp(h*[poles_fb, -7]*2))';

% Extended system
% IN: [d u] OUT: [y u u y];
Gdss_ext = ss(Gdss.A, ...
              [Gdss.B Gdss.B], ...
              [Gdss.C; [0 0 0]; Gdss.C], ...
              [[0 0]; [0 1]; [0 0]], h, ...
              'InputName', {'v', 'u'}, ...
              'OutputName', {'y', 'u', 'y'});

Gdss_ext_delay = ss([Gdss.A, Gdss.B; zeros(1, 4)], ...
              [zeros(3, 2); [1 1]], ...
              [[Gdss.C 0]; [0 0 0 0]; [Gdss.C 0]], ...
              [[0 0]; [0 1]; [0 0]], h, ...
              'InputName', {'v', 'u'}, ...
              'OutputName', {'y', 'u', 'y'});

K_obs = ss([[Gdss.A; [0 0 0]] - K*Gdss.C, [Gdss.B; 1]], ... % A
           [[Gdss.B; 0], K], ... % B
           repmat([-L -1], 2, 1), ... % C
           zeros(2), h, ... % D
           'InputName', {'u', 'y'}, 'OutputName', {'u', 'u'});

sys_delay = lft(Gdss_ext_delay, K_obs, 1, 2);

figure; hold on;
tin = 0:h:2;
sim_with_input(sys_delay, tin);
title('\textbf{Redesigned output feedback disturbance rejection}');
yyaxis left; ylim([-0.01 0.03]); yyaxis right; ylim([-1.7 0.1]);
exportgraphics(gcf, '../tex/media/q13/out_distrej_redesign.eps');

%% Redesign LQR
Q0 = Gdss.C'*Gdss.C;
Q = 428*Q0;
R = 1;
L = dlqr(Gdss.A, Gdss.B, Q, R);
Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B);

sys = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, [Gdss.C; -L], [Gdss.D; Lc], h);



