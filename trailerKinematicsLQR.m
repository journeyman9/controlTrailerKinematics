%% trailer Kinetmatic LQR 
% Journey McDowell (c) 2018

clear; close all; clc;

%% Parameters
lr = 1.96; %[m] tractor wheelbase
lt = 4; %[m] trailer wheelbase
lh = 0.53; %[m] hitch wheelbase

%% Change 
vr = 1; %[m/s] keep below 4.5 m/s
ur = deg2rad(-0.5); %[deg/s] max 30

tractorParams = [lr lt lh vr];

%% At steady state, theta = phi_d_r - phi_d_t = 0, so vt_ss = wr*R
% Radius = 6; %[m] Radius of turn
% sigma = 1; % clock wise = -1, counter-clockwise = 1
% wss = sigma*abs(vr)*sqrt(lt.^2 + Radius.^2 - lh^2)/(lt.^2 + Radius.^2 - lh.^2);
% thss = -2*sigma*sign(vr)*atan((Radius - sqrt(lt.^2 + Radius.^2 + lh.^2))/(lt -lh));
% vt = vr*cos(thss) + lh*wss*sin(thss);

%% Linearized State Space
A = [0          1           0           0;
     0          0           0           0;
     vr./lt     -lh./lt     -vr./lt     0;
     0          0           vr          0];
 
B = [0;
     vr./lr;
     0;
     0];
 
C = [0 0 0 1
     0 0 1 0]; 
 
D = [0;
     0];

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
G = [0 0 0 0;
     0 0 0 0;
     0 0 4 0;
     0 0 0 3];
H = zeros(4,1);
rho = 1;
R = 1;
Q = [0 0 0 0;
     0 0 0 0;
     0 0 4 0;
     0 0 0 3];
QQ = G'*Q*G;
RR = H'*Q*H + rho*R;
NN = G'*Q*H;
QN = eye(2); %Match dimension of state
RN = eye(2); %Match dimension of output
Bbar = B;

[K S e] = lqr(sys, QQ, RR, NN);
[est, L, P] = kalman(ss(A, [B Bbar], C, zeros(2,2)), QN, RN);

%% Simulink
trailerIC = [3, 6]; %x_t y_t
tractorIC = [trailerIC(1), trailerIC(2) + (lt+lh)]; 
ICs = [deg2rad(0); 0; deg2rad(0); trailerIC(1,2)]; % phi_r phi_d_r phi_t y_t

sim('LQRTrailerKinematics.slx')

% y_te = OptimalControl(:,1);
% phi_te = OptimalControl(:,2);

y_te = physics(:,1);
phi_te = physics(:,2);

%% Plots
figure
subplot 211
plot(tout, y_te)
hold on
plot(tout, xform_set(:,1), '--r')
hold off
ylabel('y_{t} [m]')
subplot 212
plot(tout, rad2deg(phi_te))
ylabel('\phi_{t} [{\circ}]')
xlabel('time [s]')
movegui('west')

figure
hold on
plot(trailer_xy(:,1), trailer_xy(:,2), 'b')
plot(tractor_xy(:,1), tractor_xy(:,2), 'g')
% x_center = 6;
% y_center = -6;
% t = 0:.01:2*pi;
% x_desired = Radius*cos(t) + x_center;
% y_desired = Radius*sin(t) + y_center;
% plot(x_desired, y_desired, '--r')

x_desired_line = linspace(trailerIC(1), tractor_xy(end,1), 100);
y_desired_line = 0*ones(length(x_desired_line));
plot(x_desired_line, y_desired_line, '--r')
plot(trailer_xy(1,1), trailer_xy(1,2), 'ob')
plot(tractor_xy(1,1), tractor_xy(1,2), 'og')
plot(trailer_xy(end,1), trailer_xy(end,2), 'xb')
plot(tractor_xy(end,1), tractor_xy(end,2), 'xg')
axis square
axis equal
% ylim([-15 3])
% xlim([-3 15])
xlabel('Position in x [m]')
ylabel('Position in y [m]')
legend('trailer path', 'tractor path', 'desired path')
hold off
movegui('east')