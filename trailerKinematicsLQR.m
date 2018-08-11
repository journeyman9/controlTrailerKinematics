%% trailer Kinetmatic LQR 
% Journey McDowell (c) 2018

clear; close all; clc;

%% Parameters
lr = 1.96; %[m] tractor wheelbase
lt = 4; %[m] trailer wheelbase
lh = 0.53; %[m] hitch wheelbase
vr = 4.5; %[m/s] keep below 4.5 m/s
orientation = 1; % 0 for Horizontal, 1 for vertical

tractorParams = [lr lt lh vr];

%% At steady state, theta = phi_d_r - phi_d_t = 0, so vt_ss = wr*R
% Radius = 6; %[m] Radius of turn
% sigma = 1; % clock wise = -1, counter-clockwise = 1
% wss = sigma*abs(vr)*sqrt(lt.^2 + Radius.^2 - lh^2)/(lt.^2 + Radius.^2 - lh.^2);
% thss = -2*sigma*sign(vr)*atan((Radius - sqrt(lt.^2 + Radius.^2 + lh.^2))/(lt -lh));
% vt = vr*cos(thss) + lh*wss*sin(thss);

%% Linearized State Space
%Steering rate
% A = [0          1           0           0;
%      0          0           0           0;
%      vr./lt     -lh./lt     -vr./lt     0;
%      0          0           vr          0];
%  
% B = [0;
%      vr./lr;
%      0;
%      0];
 
% C = [0 0 0 1
%      0 0 1 0];
%  
% D = [0;
%      0];

% full state
% C = eye(4);
% D = zeros(4, 1);

% steering angle
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
% %steering rate
% G = [0 0 0 0;
%      0 0 0 0;
%      0 0 0 1;
%      0 0 1 0];
% H = zeros(4,1);
% rho = 1;
% R = 1;
% Q = [0 0 0 0;
%      0 0 0 0;
%      0 0 4 0;
%      0 0 0 3];

%steering angle
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
track_vector = csvread('t_circle.txt');
s = track_vector(:, 5);
t = abs(s / vr);
curv = [t track_vector(:, 3)];
yaw_r = [t track_vector(:, 4)];
y_r = [t track_vector(:, 2)];
x_r = [t track_vector(:, 1)];

sim_time = t(end, 1);

%% Simulink
y_IC = 1;

if orientation == 1
    trailerIC = [track_vector(1,1)-y_IC*sin(90), track_vector(1, 2)+y_IC*cos(90)]; %x_t y_t
    % steering rate
%     tractorIC = [trailerIC(1), trailerIC(2) + (lt+lh)];
%     ICs = [deg2rad(90); 0; deg2rad(90); y_IC]; %phi_r phi_d_r phi_t y_t
    % steering angle
    tractorIC = [trailerIC(1), trailerIC(2) + (lt+lh)];
    ICs = [deg2rad(90); deg2rad(90); y_IC]; %phi_r phi_t y_t
else
    % steering angle
    trailerIC = [track_vector(1,1)-y_IC*sin(0), track_vector(1, 2)+y_IC*cos(0)]; %x_t y_t
%     tractorIC = [trailerIC(1) + (lt+lh), trailerIC(2)]; 
%     ICs = [deg2rad(0); 0; deg2rad(0); y_IC]; %phi_r phi_d_r phi_t y_t
    tractorIC = [trailerIC(1) + (lt+lh), trailerIC(2)]; 
    ICs = [deg2rad(0); deg2rad(0); y_IC]; %phi_r phi_t y_t
end

sim('LQRTrailerKinematics.slx')

y_te = error(:,1);
phi_te = error(:,2);

%% Plots
figure
subplot 211
plot(tout, y_te)
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('y_{te} [m]')
subplot 212
plot(tout, rad2deg(phi_te))
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('\phi_{t} [{\circ}]')
xlabel('time [s]')
legend('response', 'desired')

% figure
% hold on
% plot(odometry(:, 5), odometry(:, 4), 'b') % trailer
% plot(odometry(:, 7), odometry(:, 6), 'g') % tractor
% 
% plot(odometry(1, 5), odometry(1, 4), 'ob')
% plot(odometry(1, 7), odometry(1, 6), 'og')
% plot(odometry(end, 5), odometry(end, 4), 'xb')
% plot(odometry(end, 7), odometry(end, 6), 'xg')
% axis square
% axis equal
% xlabel('Position in x [m]')
% ylabel('Position in y [m]')
% legend('trailer path', 'tractor path', 'desired path')
% hold off
