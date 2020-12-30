clear; clc;

% Body frame
syms Jx Jy Jz wx wy wz tau_x tau_y tau_z
wx_dot = 1/Jx*(-(Jz - Jy)*wy*wz + tau_x);
wy_dot = 1/Jy*(-(Jx - Jz)*wx*wz + tau_y);
wz_dot = 1/Jz*(-(Jy - Jx)*wx*wy + tau_z);

% --- WITH EULER ANGLES ---
% Inertial frame
% syms phi theta psi
% phi_dot = wx + (wy*sin(phi) + wz*cos(phi))*tan(theta);
% theta_dot = wy*cos(phi) - wz*sin(phi);
% psi_dot = (wx*sin(phi) + wy*cos(phi))/cos(theta);
% 
% f = [wx_dot; wy_dot; wz_dot; phi_dot; theta_dot; psi_dot];
% 
% A_general = jacobian(f, [wx, wy, wz, phi, theta, psi]);
% B_general = jacobian(f, [tau_x, tau_y, tau_z]);
% C = [zeros(3) eye(3)];
% 
% % Substitute fictitious numerical values
% A = subs(A_general, [wx, wy, wz, phi, theta, psi, Jx, Jy, Jz], [1 1 0 0 0 0 24866 24018 4621]);
% B = subs(B_general, [Jx, Jy, Jz], [24866 24018 4621]);

% --- NO EULER ANGLES ---
% f = [wx_dot, wy_dot, wz_dot];
% A_general = jacobian(f, [wx, wy, wz]);
% B_general = jacobian(f, [tau_x, tau_y, tau_z]);
% 
% A = subs(A_general, [wx wy wz Jx Jy Jz], [0 0.1 0 24866 24018 4621]);
% B = subs(B_general, [Jx Jy Jz], [24866 24018 4621]);

% --- FROM THE SPACECRAFT BOOK ---
J = [24851 21 525
     21 24014 321
     525 321 4640];
A = [zeros(3, 6); eye(3), zeros(3)];
B = [inv(J); zeros(3)];
C = [zeros(3) eye(3)];


G = tf(ss(A, B, C, zeros(3)));
% G = minreal(G, 1e-5);

for i = 1:3 
    G(i,i)
end