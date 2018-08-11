%% trailer Kinetmatic LQR 
% Journey McDowell (c) 2018

clear; close all; clc;

%% Parameters
lr = 1.96; %[m] tractor wheelbase
lt = 4; %[m] trailer wheelbase
lh = 0.53; %[m] hitch wheelbase
vr = 4.5; %[m/s] keep below 4.5 m/s
orientation = 'right'; % right for horizontal, up for vertical, left for pi, and down for 3pi/2

tractorParams = [lr lt lh vr];

%% Linearized State Space
A = [0       0         0;
     vr./lt  -vr./lt   0;
     0       vr        0];

B = [vr./lt;
     -lh*vr ./ (lr*lt);
     0];

C = eye(3);
D = zeros(3, 1);

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
G = eye(3);
H = zeros(3, 1);
rho = 1;
R = 1;
Q = eye(3);

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
track_vector = csvread('t_fortyfive.txt');
s = track_vector(:, 5);
t = abs(s / vr);
curv = [t track_vector(:, 3)];
yaw_trailer = [t track_vector(:, 4)];
yaw_tractor = yaw_trailer;
y_r = [t track_vector(:, 2)];
x_r = [t track_vector(:, 1)];

sim_time = t(end, 1);

%% Simulink
y_IC = 1;

switch orientation
    case 'right'
        trailerIC = [track_vector(1,1)-y_IC*sin(0), track_vector(1, 2)+y_IC*cos(0)]; %x_t y_t
        tractorIC = [trailerIC(1) + (lt+lh), trailerIC(2)]; 
        ICs = [deg2rad(0); deg2rad(0); y_IC]; %phi_r phi_t y_t
    case 'up'
        trailerIC = [track_vector(1,1)-y_IC*sin(pi/2), track_vector(1, 2)+y_IC*cos(pi/2)]; %x_t y_t
        tractorIC = [trailerIC(1), trailerIC(2) + (lt+lh)];
        ICs = [deg2rad(90); deg2rad(90); y_IC]; %phi_r phi_t y_t
    case 'left'
        trailerIC = [track_vector(1,1)-y_IC*sin(pi), track_vector(1, 2)+y_IC*cos(pi)]; %x_t y_t
        tractorIC = [trailerIC(1) - (lt+lh), trailerIC(2)]; 
        ICs = [deg2rad(180); deg2rad(180); y_IC]; %phi_r phi_t y_t
    case 'down'
        trailerIC = [track_vector(1,1)-y_IC*sin(pi/2), track_vector(1, 2)+y_IC*cos(pi/2)]; %x_t y_t
        tractorIC = [trailerIC(1), trailerIC(2) - (lt+lh)];
        ICs = [deg2rad(270); deg2rad(270); y_IC]; %phi_r phi_t y_t
end

sim('LQRTrailerKinematics.slx')

y_te = error(:,1);
phi_te = error(:,2);

%% Plots
figure
ax1 = subplot(2, 1, 1);
plot(tout, y_te)
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('y_{te} [m]')
ax2 = subplot(2, 1, 2);
plot(tout, rad2deg(phi_te))
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('\psi_{te} [{\circ}]')
xlabel('time [s]')
legend('response', 'desired')
movegui('west')
linkaxes([ax1 ax2], 'x')

figure
hold on
plot(track_vector(:, 1), track_vector(:, 2), '--r')
plot(odometry(:, 5), odometry(:, 4), 'b') % trailer
plot(odometry(:, 7), odometry(:, 6), 'g') % tractor

plot(odometry(1, 5), odometry(1, 4), 'ob')
plot(odometry(1, 7), odometry(1, 6), 'og')
plot(odometry(end, 5), odometry(end, 4), 'xb')
plot(odometry(end, 7), odometry(end, 6), 'xg')
axis square
axis equal
xlabel('Position in x [m]')
ylabel('Position in y [m]')
legend('desired path', 'trailer path', 'tractor path')
hold off