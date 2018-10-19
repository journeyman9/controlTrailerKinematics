%% trailer Kinetmatic LQR 
% Journey McDowell (c) 2018

clear; close all; clc;

%% Parameters
lr = 5.7336; %[m] tractor wheelbase
lt = 12.192; %[m] trailer wheelbase
lh = -0.2286; %[m] hitch wheelbase (e1 from Luijten)
vr = -4.5; %[m/s] keep below 4.5 m/s
orientation = 'up'; % right for horizontal, up for vertical, left for pi, and down for 3pi/2

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

% x = [yaw_tractor, yaw_trailer, y_r]
sys = ss(A, B, C, D);

%% Transfer Function
[num, den] = ss2tf(A, B, C, D);
G1 = tf(num(1,:), den(1,:));
G2 = tf(num(2,:), den(1,:));
G3 = tf(num(3,:), den(1,:));

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
track_vector = csvread('t_cw_circle.txt');
s = track_vector(:, 5);
t = abs(s / vr);
curv = [t track_vector(:, 3)];
if vr < 0
    yaw_trailer = [t track_vector(:, 4)-pi];
else
    yaw_trailer = [t track_vector(:, 4)];
end
yaw_tractor = yaw_trailer;
y_r = [t track_vector(:, 2)];
x_r = [t track_vector(:, 1)];

sim_time = t(end, 1);

%% Simulink
y_IC = 0;

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
        trailerIC = [track_vector(1,1)-y_IC*sin(3*pi/2), track_vector(1, 2)+y_IC*cos(3*pi/2)]; %x_t y_t
        tractorIC = [trailerIC(1), trailerIC(2) - (lt+lh)];
        ICs = [deg2rad(270); deg2rad(270); y_IC]; %phi_r phi_t y_t
end

sim('LQRTrailerKinematics.slx')

% x = [yaw_tractor, yaw_trailer, y_r]
psi_tractor_e = error(:, 1);
psi_te = error(:, 2);
y_te = error(:, 3);

%% Plots

figure
ax1 = subplot(3, 1, 1);
plot(tout, rad2deg(psi_tractor_e))
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('\psi_{tractor} [{\circ}]')
ax2 = subplot(3, 1, 2);
plot(tout, rad2deg(psi_te))
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('\psi_{te} [{\circ}]')
ax3 = subplot(3, 1, 3);
plot(tout, y_te)
hold on
plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
hold off
ylabel('y_{te} [m]')

xlabel('time [s]')
legend('response', 'desired')
movegui('west')
linkaxes([ax1 ax2, ax3], 'x')

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
movegui('east')
hold off

%% Animation
W_c = lr;
H_c = lt / 3;
W_t = lt;
H_t = lt / 3;

time = 0:.01:sim_time;
tractor_x = interp1(tout, odometry(:, 7), time);
tractor_y = interp1(tout, odometry(:, 6), time);
trailer_x = interp1(tout, odometry(:, 5), time);
trailer_y = interp1(tout, odometry(:, 4), time);
psi_tractor = interp1(tout, odometry(:, 1), time);
psi_trailer = interp1(tout, odometry(:, 3), time);

% tractor_x = odometry(:, 7);
% tractor_y = odometry(:, 6);
% trailer_x = odometry(:, 5);
% trailer_y = odometry(:, 4);
% psi_tractor = odometry(:, 1);
% psi_trailer = odometry(:, 3);

DCM = @(ang) [cos(ang) -sin(ang) 0;
              sin(ang)  cos(ang) 0;
                0         0      1];

% homogenous transformation
center = @(x, y) [1 0 x;
                  0 1 y;
                  0 0 1];
figure
for i = 1:length(time)
    plot(track_vector(:, 1), track_vector(:, 2), '--r')
    hold on
    
    ang0 = psi_trailer(i);
    ang1 = psi_tractor(i);
    
    % tractor ccw pts starting with top right -- rear axle
    x_trac = [tractor_x(i)+W_c tractor_x(i) tractor_x(i) tractor_x(i)+W_c tractor_x(i)+W_c]; 
    y_trac = [tractor_y(i)+H_c/2 tractor_y(i)+H_c/2 tractor_y(i)-H_c/2 tractor_y(i)-H_c/2 tractor_y(i)+H_c/2];
    corners_trac = zeros(5, 3);
    for j = 1:length(x_trac)
        corners_trac(j, 1:3) = center(tractor_x(i), tractor_y(i)) * DCM(ang1) * center(-tractor_x(i), -tractor_y(i)) * [x_trac(j); y_trac(j); 1];
    end
    plot(corners_trac(:, 1), corners_trac(:, 2), 'g-', 'LineWidth', 2)
    
    % trailer ccw pts starting with top right -- rear axle
    x_trail = [trailer_x(i)+W_t trailer_x(i) trailer_x(i) trailer_x(i)+W_t trailer_x(i)+W_t]; 
    y_trail = [trailer_y(i)+H_t/2 trailer_y(i)+H_t/2 trailer_y(i)-H_t/2 trailer_y(i)-H_t/2 trailer_y(i)+H_t/2];
    corners_trail = zeros(5, 3);
    for j = 1:length(x_trail)
        corners_trail(j, 1:3) = center(trailer_x(i), trailer_y(i)) * DCM(ang0) * center(-trailer_x(i), -trailer_y(i)) * [x_trail(j); y_trail(j); 1];
    end
    plot(corners_trail(:, 1), corners_trail(:, 2), 'b-', 'LineWidth', 2)

    xlim([trailer_x(i)-20 trailer_x(i)+20])
    ylim([ trailer_y(i)-20 trailer_y(i)+20])
    xlabel('Position in x [m]')
    ylabel('Position in y [m]')
    drawnow
    hold off
end

