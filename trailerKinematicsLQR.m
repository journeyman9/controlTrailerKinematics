%% trailer Kinetmatic LQR 
% Journey McDowell (c) 2018

clear; close all; clc;

%% Parameters
L1 = 1.96; %[m] tractor wheelbase
L2 = 4; %[m] trailer wheelbase
Lh = 0.53; %[m] hitch length
v_x1 = 4.5; %[m/s] keep below 4.5 m/s
orientation = 'up'; % right for horizontal, up for vertical, left for pi, and down for 3pi/2

%% Linearized State Space
phi_eq = 0;
del_eq = 0;

A = [0       v_x1        0;
     0       0           0;
     0       0  -(v_x1 * cos(phi_eq) / L2)];

B = [0;
     v_x1 / (L1 * (cos(del_eq)).^2);
     v_x1 / (L1 * (cos(del_eq)).^2)];

C = [1     0     0;
     0     1     0;
     0     0     1;
     1 -(Lh+L2) L2];

D = zeros(4, 1);

sys = ss(A, B, C, D);

%% Transfer Function
[num, den] = ss2tf(A, B, C, D);
G1 = tf(num(1,:), den(1,:));
G2 = tf(num(2,:), den(1,:));

%% Controllability
controllability = rank(ctrb(A, B));

%% Observability
observability = rank(obsv(A, C));

%% LQR Gains
G = eye(4);
H = zeros(4, 1);
rho = 1;
R = 1;
Q = eye(4);

QQ = G'*Q*G;
RR = H'*Q*H + rho*R;
NN = G'*Q*H;
QN = eye(2); %Match dimension of state
RN = eye(2); %Match dimension of output
Bbar = B;

[K S e] = lqr(sys, QQ, RR, NN);
% [est, L, P] = kalman(ss(A, [B Bbar], C, zeros(2,2)), QN, RN);

%% Set Point Control
% Q_sp = [A, B; G, H];
% [n, n] = size(A);
% % [l, p] = size(C); %Number of controlled outputs
% l = 2;
% m = 1; %Number of process inputs, or just inputs
% M = pinv(Q_sp); %psuedo inverse because matrix is not square
% F = M(1:n, end-l+1:end);
% N = M(end-m+1:end, end-l+1:end);

%% Feedforward
track_vector = csvread('t_circle.txt');
s = track_vector(:, 5);
t = abs(s / v_x1);
curv = [t track_vector(:, 3)];
hitch = [t track_vector(:, 4)];
yaw_tractor = [t track_vector(:, 4)];
y_r = [t track_vector(:, 2)];
x_r = [t track_vector(:, 1)];

sim_time = t(end, 1);

%% Simulink
y_IC = 0;

switch orientation
    case 'right'
        trailerIC = [track_vector(1,1)-y_IC*sin(0), track_vector(1, 2)+y_IC*cos(0), deg2rad(0)]; %x, y, yaw_trailer
        tractorIC = [trailerIC(1) + (L2+Lh), trailerIC(2)]; %x, y
        ICs = [y_IC; deg2rad(0); deg2rad(0)]; %y, yaw_tractor, hitch
    case 'up'
        trailerIC = [track_vector(1,1)-y_IC*sin(pi/2), track_vector(1, 2)+y_IC*cos(pi/2), deg2rad(90)];  %x, y, yaw_trailer
        tractorIC = [trailerIC(1), trailerIC(2) + (L2+Lh)]; %x, y
        ICs = [y_IC; deg2rad(90); deg2rad(0)]; %y, yaw_tractor, hitch
    case 'left'
        trailerIC = [track_vector(1,1)-y_IC*sin(pi), track_vector(1, 2)+y_IC*cos(pi), deg2rad(180)];  %x, y, yaw_trailer
        tractorIC = [trailerIC(1) - (L2+Lh), trailerIC(2)]; %x, y
        ICs = [y_IC; deg2rad(180); deg2rad(0)]; %y, yaw_tractor, hitch
    case 'down'
        trailerIC = [track_vector(1,1)-y_IC*sin(3*pi/2), track_vector(1, 2)+y_IC*cos(3*pi/2), deg2rad(270)];  %x, y, yaw_trailer
        tractorIC = [trailerIC(1), trailerIC(2) - (L2+Lh)]; %x, y
        ICs = [y_IC; deg2rad(270); deg2rad(0)]; %y, yaw_tractor, hitch
end

sim('LQRTrailerKinematics.slx')

%% Plots
figure
hold on
plot(track_vector(:, 1), track_vector(:, 2), '--r')
plot(odometry(:, 6), odometry(:, 5), 'b') % trailer
plot(odometry(:, 4), odometry(:, 3), 'g') % tractor

plot(odometry(1, 6), odometry(1, 5), 'ob')
plot(odometry(1, 4), odometry(1, 3), 'og')
plot(odometry(end, 6), odometry(end, 5), 'xb')
plot(odometry(end, 4), odometry(end, 3), 'xg')
axis square
axis equal
xlabel('Position in x [m]')
ylabel('Position in y [m]')
legend('desired path', 'trailer path', 'tractor path')
hold off

y_tractor_e = error(:,1);
psi_1_e = error(:,2);
phi_e = error(:, 3);
y_trailer_e = error(:, 4);

figure
ax1 = subplot(4, 1, 1);
plot(tout, y_tractor_e)
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('y_{tractor_e} [m]')

ax2 = subplot(4, 1, 2);
plot(tout, rad2deg(psi_1_e))
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('\psi_{1_e} [{\circ}]')

ax3 = subplot(4, 1, 3);
plot(tout, rad2deg(phi_e));
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('\phi_{e} [{\circ}]')

ax4 = subplot(4, 1, 4);
plot(tout, y_trailer_e)
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('y_{trailer_e} [m]')

xlabel('time [s]')
legend('response', 'desired')
movegui('west')
linkaxes([ax1 ax2], 'x')

