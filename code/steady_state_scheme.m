close all; clearvars; clc;

s = tf('s');

run plot_settings;

%% Plant
Gc = tf([1 20], [1 24 144 0]);
Gcss = canon(ss(Gc), 'companion');
h = 0.037622;
Gdss = canon(c2d(Gcss, h, 'zoh'), 'canonical');

Q0 = Gdss.C'*Gdss.C;
Q = 428*Q0;
R = 1;

% L = dlqr(Gdss.A, Gdss.B, Q, R);
L = dlqr(Gdss.A, Gdss.B, Q, R);
LQR_sys = ss(Gdss.A - Gdss.B*L, [Gdss.B Gdss.B], Gdss.C, Gdss.D, h);

G_sys = ss(Gdss.A, ...
           [zeros(3, 1), Gdss.B, Gdss.B], ...
           [Gdss.C; eye(3); zeros(1, 3)], ...
           [[0, Gdss.D, Gdss.D]; zeros(3, 3); ...
           [1 0 0]], h, ...
           'InputName', {'r', 'd', 'u'}, ...
           'OutputName', {'y', 'x1', 'x2', 'x3', 'r'});

%% LQR Integral action
% specialstep(sys);

R = 1;
Q_ext = blkdiag(10*Q, 1e3);

L_ext = dlqr([Gdss.A, zeros(3, 1); -Gdss.C 1], [Gdss.B; 0], Q_ext, R);
L = L_ext(1:3); Li = L_ext(end);
% Li = -1;

% Build an extended system with integration of the states
K_sys = ss(1, [-Gdss.C 1], -Li, [-L 1], h, 'InputName', {'x1', 'x2', 'x3', ...
            'r'}, 'OutputName', {'u'});

% LQR with integral servo
tin = 0:h:1.5;
CL_sys = lft(G_sys, K_sys);
figure
set(gcf, 'Position', get(gcf, 'Position').*[1 1 1 1.4])
tile = tiledlayout(2, 1);
nexttile
specialstep(CL_sys(1, 1), tin); hold on;
specialstep(LQR_sys(1, 1), tin); xlabel('');
title('Reference tracking')
nexttile
specialstep(CL_sys(1, 2), tin); hold on;
specialstep(LQR_sys(1, 2), tin); xlabel('');

title('Disturbance rejection')
xlabel(tile, 'Time (s)', 'interpreter', 'latex');
lgd = legend({'Extended LQR', 'Normal LQR (no FFW)'}, 'Orientation', 'horizontal');
lgd.Layout.Tile = 'north';
title(tile, '\textbf{LQR with integral state}', 'interpreter', 'latex');
exportgraphics(gcf, '../tex/media/q12/extlqr.eps');

%% Output feedback 
% OS_max = 1;
% zeta_target = -log(OS_max/100)/sqrt(pi^2 + log(OS_max/100)^2)*0.965;
% Ts_target = 1.045;
% w = -log(0.01*sqrt(1-zeta_target^2))/Ts_target/zeta_target;
% % w = 5.5; 
% h = 0.3/1.3/w;
% 
% target_pole = complex(-zeta_target*w, sqrt(1 - zeta_target^2)*w);
% poles_fb = [target_pole, conj(target_pole), -1.4*w];
% 
% L_ext = place([Gdss.A zeros(3, 1); -Gdss.C 1], [Gdss.B; 0], ...
%                 exp(h*[poles_fb -4]));
% L = L_ext(1:3);
% Li = L(end);
% K = place([Gdss.A Gdss.B; [0 0 0 1]]', [Gdss.C 0]', ...
%                                      exp(h*[poles_fb, -7]*2))';
% % Extended system
% % IN: [v u] OUT: [y u u y];
% Gdss_ext = ss(Gdss.A, ...
%               [Gdss.B Gdss.B], ...
%               [Gdss.C; [0 0 0]; Gdss.C], ...
%               [[0 0]; [0 1]; [0 0]], h, ...
%               'InputName', {'v', 'u'}, ...
%               'OutputName', {'y', 'u', 'y'});
% 
% K_obs = ss([[Gdss.A; [0 0 0]] - K*Gdss.C, [Gdss.B; 1]], ... % A
%            [[Gdss.B; 0], K, zeros(4, 1)], ... % B
%            [-L -1], ... % C
%            [0 0 1], h, ... % D
%            'InputName', {'u', 'y', 'r'}, 'OutputName', {'u'});
% 
% sys_inner = lft(Gdss_ext, K_obs, 1, 2);
% 
% % Extend the system to pass through the reference and output
% % IN [r u] OUT: [y y r]
% % 
% % % IN: [y r]
% K_int = ss(1, [-1 1], [Li; 0], [0 0; 1 0], h, ...
%                 'InputName', {'y', 'r'}, 'OutputName', {'u', 'y'});
% sys_outer = lft(sys_inner, K_int, 1, 1);
% step(sys_outer)
