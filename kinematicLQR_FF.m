%% Kinematic LQR + feedforward
% Journey McDowell (c) 2018

clear; close all; clc;

%% Parameters
L = 1.524; %[m]
V = 20; %[m/s]

%% Linearized State Space
A = [0 V;
     0 0]; 

 B = [0;
      V/L];
 
C = eye(2);

D = zeros(2, 1);

sys = ss(A, B, C, D);

%% Check controllability
controllability  = rank(ctrb(A, B));

%% LQR gains
G = eye(2);
H = D;
Q = [1/(0.1^2) 0;
     0 1/(1.^2)];
p = 1;
R = 1/(deg2rad(30).^2);

QQ = G'*Q*G;
RR = H'*Q*H + p*R;
NN = G'*Q*H;

[K S e] = lqr(sys, QQ, RR, NN);

%% Set Point Control
Q_sp = [A, B; G, H];
[n, n] = size(A);
[l, p] = size(G); % number of controlled outputs
m = 1; % number of process inputs, or just inputs
M = pinv(Q_sp); % psuedo inverse if matrix not square
F = M(1:n, end-l+1:end);
N = M(end-m+1:end, end-l+1:end);

%% Feedforward
track_vector = csvread('t_lanechange.txt');
s = track_vector(:, 5);
t = s/V;
curv = [t track_vector(:, 3)];
psi_d = [t track_vector(:, 4)];
% y_d = [t track_vector(:, 2)];
y_d = [t zeros(length(psi_d), 1)];

sim_time = t(end, 1);

%% Simulink
y_IC = 1;
ICs = [y_IC deg2rad(0)];
vehicleIC = [track_vector(1,1)-y_IC*sin(ICs(2)) track_vector(1,2)+y_IC*cos(ICs(2))];

sim('LQRFF.slx')

y_e = deviation(:, 1);
psi_e = deviation(:, 2);

%% Plots

figure
subplot 211
plot(tout, y_e)
hold on
plot(tout, des(:, 1), '--r')
ylabel('y_{e} [m]')
hold off
subplot 212
plot(tout, rad2deg(psi_e))
hold on
plot(tout, rad2deg(des(:, 2)), '--r')
hold off
xlabel('time[s]')
ylabel('\psi [{\circ}]')
legend('response', 'desired')
movegui('west')

figure
plot(track_vector(:, 1), track_vector(:, 2), '--r')
hold on
plot(odometry(:, 1), odometry(:, 2), 'b')
plot(odometry(1, 1), odometry(1, 2), 'ob')
plot(odometry(end, 1), odometry(end, 2), 'xb')
axis square
axis equal
xlabel('Position in x [m]')
ylabel('Posiiton in y [m]')
legend('desired path', 'vehicle path')
hold off