%% CONTROL ENGINEERING ASSIGNMENT

close all; clearvars; clc;

s = tf('s');

run plot_settings

%% Plant

Gc = tf([1 20], [1 24 144 0]);

% Visualize poles and zeros
% figure('Name', 'Q1: Plant pole-zero map')
% specialpzmap(Gc, 'ShowGoodies', 'on')
% exportgraphics(gcf, '../tex/media/q1/cont_plant_pzmap.eps')
% 
% figure('Name', 'Q1: Plant Bode plot')
% specialbode(Gc, 'ShowGoodies', 'on');
% ax = gca; ax.Children(1).Position(2) = 7;
% exportgraphics(gcf, '../tex/media/q1/cont_plant_bode.eps')

%% Question 1: PD(D) controller for reference tracking
fprintf('--------------- QUESTION 1 ---------------\n\n')
OS_max = 5;
zeta_target = -log(OS_max/100)/sqrt(pi^2 + log(OS_max/100)^2);
PM_target = rad2deg(atan(2*zeta_target ...
                    /sqrt(-2*zeta_target^2 + sqrt(1 + 4*zeta_target^4))));

fprintf('Target damping ratio: %.3g\n', zeta_target);
fprintf('Target phase margin: %.3g\n', PM_target);

% Controller synthesis
K1 = struct;
PIDDstruct = struct('Kp', 1, 'Ti', inf, 'Td1', 0, 'Td2', 0, 'N', 5);
for i = 1:6; K1.(sprintf('it%d', i)) = PIDDstruct; end

win = logspace(-1, 2, 500);
N = 5;

% Iteration #1: Add gain to get desired phase margin
[mag, phase] = bode(Gc, win);
[~, minidx] = min(abs(phase + 180 - PM_target));
K1.it1.Kp = 1/mag(minidx);
K1.it1.tf = tf(K1.it1.Kp);

% Iteration #2: Check overshoot and adjust accordingly
% Find gain to match overshoot perfectly
OSfun = @(Kp) (stepinfo(feedback(Kp*Gc, 1)).Overshoot - 5)^2;
K1.it2.Kp = fminbnd(OSfun, 0, 1e4);
K1.it2.tf = tf(K1.it2.Kp);

% Iteration #3: Add D-action
max_phase_add = rad2deg(asin((1-1/(N+1))/(1+1/(N+1))));

fprintf('Maximum phase contribution: %.3g\n', max_phase_add);
[~, minidx] = min(abs(-180 + PM_target - (max_phase_add + phase)));
w_max_phase = win(minidx);
K1.it3.Kp = K1.it2.Kp;
K1.it3.Td1 = place_lead(w_max_phase, N);
K1.it3.tf = K1.it3.Kp*(1 + K1.it3.Td1*s/(s*K1.it3.Td1/N + 1));
[mag, ~] = bode(K1.it3.tf*Gc, win);

% Iteration #4: Increase gain for desired phase margin
[~, minidx] = min(abs(win - w_max_phase));
K1.it4.Kp = K1.it3.Kp/mag(minidx);
K1.it4.Td1 = K1.it3.Td1;
K1.it4.tf = K1.it4.Kp*(1 + K1.it4.Td1*s/(s*K1.it4.Td1/N + 1));
[mag, phase] = bode(K1.it4.tf*Gc, win);

% Iteration %5: Tune PD
K1.it5.Kp = 102.4;
K1.it5.Td1 = 0.102;
                         
% Find best frequency to place in tight region
K1.it5.tf = K1.it5.Kp*(1 + K1.it5.Td1*s/(s*K1.it5.Td1/N + 1));

% Iteration #6: PDD
K1.it6.Td1 = place_lead(20, 5);
K1.it6.Td2 = place_lead(100, 5);
K1.it6.Kp = 6.6*K1.it5.Kp;
K1.it6.tf = K1.it6.Kp*(1 + K1.it6.Td1*s/(s*K1.it6.Td1/N + 1))...
                     *(1 + K1.it6.Td2*s/(s*K1.it6.Td2/N + 1));  

figure('Name', 'Q1: Step response', 'NumberTitle', 'on'); hold on;
nfig_q1_steps = gcf().Number;
figure('Name', 'Q1: Bode plots', 'NumberTitle', 'on'); hold on;
nfig_q1_bode = gcf().Number;

% Set x-ranges for all plots
tin = linspace(0, 2, 3e2);
win = logspace(-1, 3, 3e2);

for i = 1:6
    tmp = K1.(sprintf('it%d', i)); 
    T = feedback(tmp.tf*Gc, 1);
    
    if i ~= 1
        figure(nfig_q1_steps);
        specialstep(T, tin);

        figure(nfig_q1_bode);
        [~, ~, tile] = specialbode(tmp.tf*Gc, win);
    end
    
    info = stepinfo(T, 'SettlingTimeThreshold', 0.01);
    [~, PM] = margin(tmp.tf*Gc);
    
    % Generate LaTeX table
        % i Kp Td1 Td2 OS Ts  
    fprintf('%d & %5.4g & %8.3g & %8.3g & %5.3g & %5.2g & %5.3g\\\\ \n', ...
            i, tmp.Kp, tmp.Td1, tmp.Td2, PM,  info.Overshoot, info.SettlingTime);  
end

lgdtxt = {'\#2 P', '\#3 D-action', '\#4 D-action + target PM', ...
          '\#5 PD optimised', '\#6 PDD'};

% Export graphics
figure(nfig_q1_steps);
legend(lgdtxt, 'Location', 'southeast');
% exportgraphics(gcf, '../tex/media/q1/pd_controllers_step.eps');

figure(nfig_q1_bode);
lgd = legend(lgdtxt, 'Orientation', 'Horizontal', 'NumColumns', 3);
lgd.Layout.Tile = 'north';
set(gcf, 'Position', get(gcf, 'Position').*[1 1 1 1.3])
% exportgraphics(gcf, '../tex/media/q1/pd_controllers_bode.eps')

fprintf('Final controller:\n');
zpk(K1.it6.tf)

% Clean up the workspace
clearvars -except K1 Gc s PIDDstruct N

%% Question 2: Disturbance rejection
fprintf('\n\n--------------- QUESTION 2 ---------------\n\n')

K2 = struct();
for i = 1:10; K2.(sprintf('it%d', i)) = PIDDstruct; end

% Iteration A: PI controllers
K2.it1.Ti = 1.4*1/2;
K2.it1.Kp = 0.4*63.096; % Phase margin of about 45 degrees
K2.it1.tf = K2.it1.Kp*(1 + 1/s/K2.it1.Ti);

% This one is the best!!
K2.it2.Ti = 0.57;
K2.it2.Kp = 31.548; % Phase margin of about 40 degrees
K2.it2.tf = K2.it2.Kp*(1 + 1/s/K2.it2.Ti);

K2.it3.Ti = 0.45;
K2.it3.Kp = 0.5*63.096; % Phase margin of about 35 degrees
K2.it3.tf = K2.it3.Kp*(1 + 1/s/K2.it3.Ti);

% Iteration B: PID control
% PURPOSE: keeping the phase margin at sufficient level while maintaining
% adequate phase margin to damp oscillations
K2.it4.Ti = 0.12;
K2.it4.Kp = 6*31.548;
K2.it4.Td1 = place_lead(24, 5);
K2.it4.tf = K2.it4.Kp*(1 + 1/s/K2.it4.Ti)...
                     *(1 + K2.it4.Td1*s/(s*K2.it4.Td1/N + 1));

% Iteration C: PIDD control
% PURPOSE: place sufficient phase margin at even higher frequencies!
K2.it5.Ti = 0.05;
K2.it5.Kp = 24*186; % Phase margin of about 40 degrees
K2.it5.Td1 = place_lead(120, 5);
K2.it5.Td2 = place_lead(40, 5);
K2.it5.tf = K2.it5.Kp*(1 + 1/s/K2.it5.Ti)...
                     *(1 + K2.it5.Td1*s/(s*K2.it5.Td1/N + 1))...
                     *(1 + K2.it5.Td2*s/(s*K2.it5.Td2/N + 1));
                 
% figure(1); hold on; % Pzmap

figure('Name', 'Q2: Step response', 'NumberTitle', 'on'); hold on; % Step
nfig_q2_steps = gcf().Number;

tin = linspace(0, 4, 8e2);
win = logspace(-2, 3, 3e2);

lgdtxt = {};


for i = 1:3
   K = K2.(sprintf('it%d', i)).tf; 
   Gd = feedback(Gc, K);
   S = feedback(1, K*Gc);
   
   figure(nfig_q2_steps);
   specialstep(Gd, tin, 'ShowGoodies', 'off');
   
   info = stepinfo(Gd, 'SettlingTimeThreshold', 0.01);

   lgdtxt{end+1} = sprintf('PM = %.2g deg $\\quad T_s$ = %.3gs', ...
                            allmargin(K*Gc).PhaseMargin, ...
                            info.SettlingTime);

   fprintf('Iteration %d: Ts = %6.4g M = %6.4g\n', i, info.SettlingTime, ...
         info.Peak);
end

figure(nfig_q2_steps);
ylabel('Amplitude');
title('\textbf{Disturbance step}')
subtitle('Comparison between different values for the PM');
legend(lgdtxt);
% exportgraphics(gcf, '../tex/media/q2/pi_pm_comparison.eps');

figure; hold on; % Bode
nfig_q2_steps2 = gcf().Number;

figure; hold on; % Bode
nfig_q2_bode = gcf().Number;

for i = [2 4 5]
   K = K2.(sprintf('it%d', i)).tf; 
   Gd = feedback(Gc, K);
   S = feedback(1, K*Gc);
   
   figure(nfig_q2_steps2);
   specialstep(Gd, tin, 'ShowGoodies', 'off');
   
   info = stepinfo(Gd, 'SettlingTimeThreshold', 0.01);

   lgdtxt{end+1} = sprintf('PM = %.2g deg $\\quad T_s$ = %.3gs', ...
                            allmargin(K*Gc).PhaseMargin, ...
                            info.SettlingTime);
   figure(nfig_q2_bode);
   [~, ~, ~, tile] = specialbode(K*Gc, win, 'ShowGoodies', 'off');
   
   fprintf('Iteration %d: Ts = %6.4g M = %6.4g\n', i, info.SettlingTime, ...
         info.Peak);
end

lgd = legend({'PI', 'PID', 'PIDD'}, 'Orientation', 'horizontal');
lgd.Layout.Tile = 'north';
exportgraphics(gcf, '../tex/media/q2/pi_bode_comparison.eps');

figure
specialstep(feedback(Gc, K2.it5.tf), 'ShowGoodies', 'off');
set(gcf, 'Position', get(gcf, 'Position').*[1 1 1 0.6]);
set(gca, 'Position', get(gca, 'Position').*[1 1.3 1 0.75]);
title('\textbf{Disturbance step}');
subtitle('PIDD controller');
ylabel('Amplitude')
exportgraphics(gcf, '../tex/media/q2/pidd_response.eps');

fprintf('Final controller:\n');
zpk(K2.it5.tf)

clearvars -except K1 K2 Gc s PIDDstruct N

%% Question 3: Discretize the plant
% Convert the system to state-space notation
fprintf('\n\n--------------- QUESTION 3 ---------------\n\n')
Gcss = canon(ss(Gc), 'companion');
disp('Continous-time state-space model (obs. canonical form):');

disp('A'); disp(Gcss.A');
disp('B'); disp(Gcss.C');
disp('C'); disp(Gcss.B');

alpha = poly(Gcss.A);
Talpha = eye(3);
Talpha(1, 2:3) = alpha(2:3);
Talpha(2, 3) = alpha(2);
S = ctrb(Gcss.A', Gcss.C')*Talpha;

% disp('A'); disp((S\Gcss.A'*S)');
% disp('B'); disp((Gcss.B'*S)');
% disp('C'); disp((S\Gcss.C')');
[~, ~, ~, wc] = margin(Gc);
h3 = 1/12/5;
% h3 = 0.4/max(abs(pole(Gc))); % Based on w0 of the fastest pole
Gdss = canon(c2d(Gcss, h3, 'zoh'), 'modal');
% Check whether the sampling time makes sense
% figure
% impulse(Gdss)
format long
disp('Discrete-time state-space model:');
disp('A'); disp(Gdss.A);
disp('B'); disp(Gdss.B);
disp('C'); disp(Gdss.C);
format short

figure; set(gcf, 'Position', get(gcf, 'Position').*[1 1 1 0.8]);
[y1, t1] = impulse(Gc);
[y2, t2] = impulse(Gdss, 0:h3:max(t1));
plot(t1, y1); hold on;
stairs(t2, y2);
xlim([0 max(t2)]);
legend({'Continuous system', 'Discrete system'}, 'Location', 'best');
title('\textbf{Impulse response}'); xlabel('Time (s)'); ylabel('Amplitude');
% exportgraphics(gcf, '../tex/media/q3/dt_plant_impulse.eps');

clearvars -except K1 K2 Gc s PIDDstruct N Gcss

%% Question 4: Discretize the controllers
% h4_tracking = stepinfo(feedback(K1.it6.tf*Gc, 1)).RiseTime/8;
% [~, ~, ~, wc] = margin(Gc*K2.it5.tf); % Get crossover frequency
% h4_distrej = stepinfo(feedback(Gc, K2.it5.tf)).PeakTime/20;
% h4_distrej = 1.1/wc;
% h4_distrej = 0.6/max(abs(pole(K2.it5.tf*Gc)));
h4_tracking = K1.it6.Td2*0.7/10;
h4_distrej = K2.it5.Td1*0.7/10;

figure('Name', 'Q4: DT Reference tracking controller', 'NumberTitle', 'on')
K_tracking = c2d(K1.it6.tf, h4_tracking, 'tustin');
Gd_tracking = c2d(Gc, h4_tracking, 'zoh');
[~, tin] = specialstep(feedback(K_tracking*Gd_tracking, 1), ...
                                            'ShowGoodies', 'off'); hold on;
specialstep(feedback(K1.it6.tf*Gc, 1), linspace(0, max(tin)));
legend('Discretised', 'Continuous', 'Location', 'southeast')
title('\textbf{Reference-tracking controllers}')
exportgraphics(gcf, '../tex/media/q4/dt_tracking.eps');

figure('Name', 'Q4: DT Disturbance rejection controller', 'NumberTitle', 'on')
K_distrej = c2d(K2.it5.tf, h4_distrej, 'tustin');
Gd_distrej = c2d(Gc, h4_distrej, 'zoh');
[~, tin] = specialstep(feedback(Gd_distrej, K_distrej)); hold on;

K_distrej2 = c2d(K2.it5.tf, 10*h4_distrej, 'tustin');
Gd_distrej2 = c2d(Gc, 10*h4_distrej, 'tustin');
specialstep(feedback(Gd_distrej2, K_distrej2), ...
                1:10*h4_distrej:max(tin)); hold on;

specialstep(feedback(Gc, K2.it5.tf), linspace(0, max(tin)));

ax2 = copyobj(gca, gcf);
set(ax2, 'Position', [0.47, 0.28, 0.4, 0.4]);
xlabel(ax2, '');
set(ax2, 'XLim', [3e-3 16e-3]);
set(ax2, 'YLim', [1e-5 2e-5]);
set(ax2, 'XTickLabel', {});
set(ax2, 'YTickLabel', {});

title('\textbf{Disturbance-rejection controllers}')
legend('Discretised (ZOH)', 'Discretised (Tustin)', 'Continuous')
exportgraphics(gcf, '../tex/media/q4/dt_distrej.eps');

% figure
% hold on;
% specialbode(feedback(Gd_distrej, K_distrej));
% specialbode(feedback(Gd_distrej2, K_distrej2));
% specialbode(feedback(Gc, K2.it5.tf));

clearvars -except K1 K2 Gc s PIDDstruct N Gcss

%% Question 5: Discrete controller with pole-placement
OS_max = 1;
zeta_target = -log(OS_max/100)/sqrt(pi^2 + log(OS_max/100)^2);
w = 20;
target_pole = complex(-zeta_target*w, sqrt(1 - zeta_target)*w);

poles_cont = [-10, -1+4j, -1-4j; % Complex pole pair
              -10, -4+2j, -4-2j; % Complex pole pair
              target_pole, conj(target_pole), -1.3*w]; % Pure damping
h5 = 0.5/1.3/w;
poles_disc = exp(poles_cont*h5);
poles_disc = [poles_disc; [0 0.001 0.002]]; % Deadbeat

Gdss = canon(c2d(Gcss, h5, 'zoh'), 'canonical');
%  
figure('Name', 'Q5: Controller design with pole-placement', 'NumberTitle', 'on')
nfig_q5_step = get(gcf, 'Number'); hold on;
figure('Name', 'Q5: Controller design with pole-placement (Pzmap)', 'NumberTitle', 'on')
nfig_q5_pzmap = get(gcf, 'Number'); hold on; 

tin = 0:h5:1.5;
lgdtxt = {};

for i = 1:length(poles_disc)
    L = place(Gdss.A, Gdss.B, poles_disc(i,:));
    Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B); % FFW gain for correct ss value
    Hcl = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, Gdss.C, Gdss.D, h5);
    figure(nfig_q5_step);
    specialstep(Hcl, tin);
    figure(nfig_q5_pzmap);
    specialpzmap(Hcl);
    lgdtxt{end+1} = strrep(sprintf('%+7s  %+7s  %+7s', ...
                                   num2str(poles_disc(i,1), '%.2g'), ...
                                   num2str(poles_disc(i,2), '%.2g'), ...
                                   num2str(poles_disc(i,3), '%.2g')), ...
                           'i', 'j'); % Display with 'j' as imag unit
end
figure(nfig_q5_step);
title('\textbf{Comparison between various pole locations}');
ylabel('Amplitude');
legend(lgdtxt, 'Location', 'southeast');
exportgraphics(gcf, '../tex/media/q5/dt_step.eps');

figure(nfig_q5_pzmap);
rectangle('Position', 2*[-0.5 -0.5 1 1], 'Curvature', 1);
exportgraphics(gcf, '../tex/media/q5/dt_pzmap.eps');

clearvars -except K1 K2 Gc s PIDDstruct N Gcss target_pole w h5

%% Question 6: Output feedback control
% -------------------------- TRACKING ------------------------------
h6 = h5;
Gdss = canon(c2d(Gcss, h6, 'zoh'), 'canonical');

% Build the controller system system
% poles_fb = [-8 -12.01 -12];
poles_fb = [target_pole, conj(target_pole), -1.3*w];
L = place(Gdss.A, Gdss.B, exp(h6*poles_fb));
K = place(Gdss.A', Gdss.C', exp(h6*poles_fb.*1.3))';

Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B);

% Generalised plant
% IN: [r u] OUT: [y u y r]
Gdss_ext = ss(Gdss.A, ... % A
              [zeros(3, 1) Gdss.B], ... % B 
              [Gdss.C; [0 0 0]; Gdss.C; [0 0 0]], ... % C
              [[0 0]; [0 1]; [0 0]; [1 0]], h6, ... % D
              'InputName', {'r', 'u'}, 'OutputName', {'y', 'u', 'y', 'r'});

% Output feedback controller
% IN: [u y r] OUT: [u] 
K_obs = ss(Gdss.A - K*Gdss.C, ... % A
           [Gdss.B K zeros(3, 1)], ... % B
           -L, ... % C
           [0 0 Lc], ... % D
           h6, 'InputName', {'u', 'y', 'r'}, 'OutputName', 'u');

Hcl_out = lft(Gdss_ext, K_obs);
Hcl_full = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, Gdss.C, Gdss.D, h6);

tin = 0:h6:0.5;
x0 = [0 0 0 -2 3 5]*10;
[y, ~, x] = lsim(Hcl_out, ones(size(tin)), tin, x0);

% Plot the results
figure('Name', 'Q6: Ouput-feedback reference tracking', 'NumberTitle', 'on')
tile = tiledlayout(2, 1);
nexttile
stairs(tin, y); hold on;
specialstep(Hcl_full, tin); xlabel('');
ylabel('Amplitude')
legend({'Output feedback', 'Full-information feedback'}, ...
        'Location', 'southeast');
nexttile; hold on;
stairs(tin, x(:, 4:6) - x(:, 1:3));
ylabel('State error');
xlabel(tile, 'Time (s)', 'interpreter', 'latex', 'FontSize', 11);

title(tile, '\textbf{Output feedback - Reference tracking}', ...
        'Interpreter', 'latex', 'FontSize', 11.5);

exportgraphics(gcf, '../tex/media/q6/output_servo.eps');

clearvars -except K1 K2 Gc s PIDDstruct N Gcss Gdss h6 poles_fb

% ------------------------- DISTURBANCE ----------------------------

% poles_fb = [-8 -12.01 -12];
L = place(Gdss.A, Gdss.B, exp(h6*poles_fb));
% K = place(Gdss.A', Gdss.C', exp(h6*poles_fb*2))';

K = place([Gdss.A Gdss.B; [0 0 0 1]]', [Gdss.C 0]', ...
                                     exp(h6*[poles_fb, -7]*2))';
% Extended system
% IN: [d u] OUT: [y u y];
Gdss_ext = ss(Gdss.A, ...
              [Gdss.B Gdss.B], ...
              [Gdss.C; [0 0 0]; Gdss.C], ...
              [[0 0]; [0 1]; [0 0]], h6, ...
              'InputName', {'v', 'u'}, ...
              'OutputName', {'y', 'u', 'y'});

K_obs = ss([[Gdss.A; [0 0 0]] - K*Gdss.C, [Gdss.B; 1]], ... % A
           [[Gdss.B; 0], K], ... % B
           [-L -1], ... % C
           0, h6, ... % D
           'InputName', {'u', 'y'}, 'OutputName', 'u');

Hcl = lft(Gdss_ext, K_obs);

figure('Name', 'Q6: Ouput-feedback reference tracking', 'NumberTitle', 'on')
tin = 0:h6:0.8;
u = ones(size(tin));

tile = tiledlayout(2, 1);
nexttile
[y, ~, x] = lsim(Hcl, u, tin, [0 0 0 1 3 5 -1]);
stairs(tin, y);
ylabel('Amplitude')

nexttile
stairs(tin, x(:, 7));
ylabel('Disturbance estimate')

title(tile, '\textbf{Output feedback - disturbance rejection}', ...
        'interpreter', 'latex', 'FontSize', 11.5);

xlabel(tile, 'Time (s)', 'interpreter', 'latex', 'FontSize', 11);
exportgraphics(gcf, '../tex/media/q6/output_distrej.eps');

clearvars -except K1 K2 Gc s PIDDstruct N Gcss h6

%% Question 7: LQR controllers
h7 = h6;
Gdss = canon(c2d(Gcss, h7, 'zoh'), 'canonical');

Rrange = [1 10];
Q0 = Gdss.C'*Gdss.C;
Qrange = [10 1000 5e4];

figure; hold on;
tin = 0:h7:3;

lgdtxt = {};

for i = 1:numel(Rrange)
    for k = 1:numel(Qrange)
        L = dlqr(Gdss.A, Gdss.B, Qrange(k)*Q0, Rrange(i));
        Lc = 1/(Gdss.C/(eye(3) - Gdss.A + Gdss.B*L)*Gdss.B); % FFW gain for correct ss value
        Hcl = ss(Gdss.A - Gdss.B*L, Gdss.B*Lc, Gdss.C, Gdss.D, h7);
        specialstep(Hcl, tin);
        lgdtxt{end+1} = sprintf('$Q$ = %.3g$\\times Q_0$\n$R$ = %d', ...
                                Qrange(k), Rrange(i));
    end
end
title('\textbf{LQR weighting matrices comparison}');
legend(lgdtxt, 'Location', 'eastoutside');
ylabel('Amplitude')
exportgraphics(gcf, '../tex/media/q7/lqr_comp.eps');


%% Functions
