%% trailer Kinetmatic LQR 
% Journey McDowell (c) 2018

clear; close all; clc;

%% Parameters
L1 = 5.74; %[m] tractor wheelbase
L2 = 10.192; %[m] trailer wheelbase
h = -0.29; %[m] hitch wheelbase (e1 from Luijten)
v1x = -2.012; %[m/s] keep below 4.5 m/s

%% Linearized State Space
A = [0       0         0;
     v1x./L2  -v1x./L2   0;
     0       v1x        0];

B = [v1x./L1;
     -h*v1x ./ (L1*L2);
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
steer_max = 45; %[degrees]

% G = eye(3);
G = [1 0 0;
     0 1 0;
     0 0 1];
H = zeros(3, 1);
rho = 1;
% R = 1;
% Q = eye(3);
R = 1 / (deg2rad(steer_max).^2);
Q = [1/(deg2rad(2).^2)       0                       0;
     0                   1/(deg2rad(2).^2)           0;
     0                        0                1/(0.1.^2)];

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

%% Trajectory Generation and Feedforward
rms_psi_1_log = [];
rms_psi_2_log = [];
rms_y2_log = [];
max_psi_1_log = [];
max_psi_2_log = [];
max_y2_log = [];
goal_log = [];

for i = 0:99
    clearvars -except A B Bbar C controllability D den e G G1 G2 G3 h H ...
        i K L1 L2 NN num observability Q QN QQ R rho RN RR S steer_max ... 
        sys v1x rms_psi_1_log rms_psi_2_log rms_y2_log max_psi_1_log ...
        max_psi_2_log max_y2_log goal_log 
    track_vector = csvread(sprintf('./dubins_path/dubins_path_%d.txt', i));
    if v1x < 0
        track_vector(:, 4) = track_vector(:, 4) + pi;
    end

    hitch_max = 90; %[degrees]

    %% Simulink
    y_IC = 0;
    psi_2_IC = deg2rad(0) + track_vector(1, 4);
    hitch_IC = deg2rad(0);

    look_ahead = 0; %indices

    psi_1_IC = hitch_IC + psi_2_IC;

    trailerIC = [track_vector(1, 1)-y_IC*sin(track_vector(1, 4)), track_vector(1, 2)+y_IC*cos(track_vector(1, 4))]; %x2, y2
    tractorIC = [trailerIC(1)+L2*cos(psi_2_IC)+h*cos(psi_1_IC), trailerIC(2)+L2*sin(psi_2_IC)+h*sin(psi_1_IC)]; %x1, y1
    ICs = [psi_1_IC; psi_2_IC; y_IC];

    sim('LQRTrailerKinematics.slx')

    % x = yaw_tractor, yaw_trailer, y_r
    psi_1_e = error(:, 1);
    psi_2_e = error(:, 2);
    y_2_e = error(:, 3);

    %% Jack-knife check 
    hitch_angle = odometry(:, 8);

    for terminal_index = 1:length(hitch_angle)
        if hitch_angle(terminal_index) > deg2rad(hitch_max)
            fprintf('Jackknifed! theta = %4.2f \n', rad2deg(hitch_angle(terminal_index)))
            break
        elseif hitch_angle(terminal_index) < deg2rad(-hitch_max)
            fprintf('Jackknifed! theta = %4.2f \n', rad2deg(hitch_angle(terminal_index)))
            break
        else
            continue
        end
    end

    %% Goal check
    if goal(end) == 1
        fprintf('GOAL with d = %4.2f m and psi = %4.2f degrees\n', d_goal(end), rad2deg(psi_goal(end)))
    else
        [minimum, best_index] = min(d_goal(1500:terminal_index));
        fprintf('TIMES UP. Closest: d = %4.2f m and psi = %4.2f degrees\n', minimum, rad2deg(psi_goal(best_index)))
    end

    tractor_x = odometry(1:terminal_index, 7);
    tractor_y = odometry(1:terminal_index, 6);
    trailer_x = odometry(1:terminal_index, 5);
    trailer_y = odometry(1:terminal_index, 4);
    psi_tractor = odometry(1:terminal_index, 1);
    psi_trailer = odometry(1:terminal_index, 3);
    
    %% summary append
    rms_psi_1_log(end+1) = rms(psi_1_e);
    rms_psi_2_log(end+1) = rms(psi_2_e);
    rms_y2_log(end+1) = rms(y_2_e);
    max_psi_1_log(end+1) = max(psi_1_e);
    max_psi_2_log(end+1) = max(psi_2_e);
    max_y2_log(end+1) = max(y_2_e);
    goal_log(end+1) = goal(end);

    %% Plots
%     figure
%     ax1 = subplot(3, 1, 1);
%     plot(tout, rad2deg(psi_1_e))
%     hold on
%     plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
%     line([tout(terminal_index) tout(terminal_index)], [max(rad2deg(psi_1_e)) min(rad2deg(psi_1_e))],'Color','red')
%     hold off
%     ylabel('\psi_{1_e} [{\circ}]')
%     ax2 = subplot(3, 1, 2);
%     plot(tout, rad2deg(psi_2_e))
%     hold on
%     plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
%     line([tout(terminal_index) tout(terminal_index)], [max(rad2deg(psi_2_e)) min(rad2deg(psi_2_e))],'Color','red')
%     hold off
%     ylabel('\psi_{2_e} [{\circ}]')
%     ax3 = subplot(3, 1, 3);
%     plot(tout, y_2_e)
%     hold on
%     plot(tout, 0*linspace(0, length(tout), length(tout))', '--r')
%     line([tout(terminal_index) tout(terminal_index)], [max(y_2_e) min(y_2_e)],'Color','red')
%     hold off
%     ylabel('y_{2_e} [m]')
% 
%     xlabel('time [s]')
%     legend('response', 'desired')
%     movegui('west')
%     linkaxes([ax1 ax2, ax3], 'x')
% 
%     figure
%     hold on
%     plot(track_vector(:, 1), track_vector(:, 2), '--r')
%     plot(trailer_x, trailer_y, 'b') % trailer
%     plot(tractor_x, tractor_y, 'g') % tractor
% 
%     plot(trailer_x(1), trailer_y(1), 'ob')
%     plot(tractor_x(1), tractor_y(1), 'og')
%     plot(trailer_x(end), trailer_y(end), 'xb')
%     plot(tractor_x(end), tractor_y(end), 'xg')
%     axis square
%     axis equal
%     xlabel('Position in x [m]')
%     ylabel('Position in y [m]')
%     legend('desired path', 'trailer path', 'tractor path')
%     movegui('east')
%     hold off
% 
%     %% Animation
%     H_c = L2 / 3;
%     H_t = L2 / 3;
% 
%     DCM = @(ang) [cos(ang) -sin(ang) 0;
%                   sin(ang)  cos(ang) 0;
%                     0         0      1];
% 
%     % homogenous transformation
%     center = @(x, y) [1 0 x;
%                       0 1 y;
%                       0 0 1];
%     figure
%     for i = 1:length(tout(1:terminal_index))
%         plot(track_vector(:, 1), track_vector(:, 2), '--r')
%         hold on
% 
%         ang0 = psi_trailer(i);
%         ang1 = psi_tractor(i);
% 
%         % trailer ccw pts starting with top right -- rear axle
%         x_trail = [trailer_x(i)+L2 trailer_x(i) trailer_x(i) trailer_x(i)+L2 trailer_x(i)+L2]; 
%         y_trail = [trailer_y(i)+H_t/2 trailer_y(i)+H_t/2 trailer_y(i)-H_t/2 trailer_y(i)-H_t/2 trailer_y(i)+H_t/2];
%         corners_trail = zeros(5, 3);
%         for j = 1:length(x_trail)
%             corners_trail(j, 1:3) = center(trailer_x(i), trailer_y(i)) * DCM(ang0) * center(-trailer_x(i), -trailer_y(i)) * [x_trail(j); y_trail(j); 1];
%         end
%         plot(corners_trail(:, 1), corners_trail(:, 2), 'b-', 'LineWidth', 2)
% 
%         % tractor ccw pts starting with top right -- rear axle
%         x_trac = [tractor_x(i)+L1 tractor_x(i) tractor_x(i) tractor_x(i)+L1 tractor_x(i)+L1]; 
%         y_trac = [tractor_y(i)+H_c/2 tractor_y(i)+H_c/2 tractor_y(i)-H_c/2 tractor_y(i)-H_c/2 tractor_y(i)+H_c/2];
%         corners_trac = zeros(5, 3);
%         for j = 1:length(x_trac)
%             corners_trac(j, 1:3) = center(tractor_x(i), tractor_y(i)) * DCM(ang1) * center(-tractor_x(i), -tractor_y(i)) * [x_trac(j); y_trac(j); 1];
%         end
%         plot(corners_trac(:, 1), corners_trac(:, 2), 'g-', 'LineWidth', 2)
% 
%         % rear axle
%         plot(trailer_x(i), trailer_y(i), 'b+')
%         plot(tractor_x(i), tractor_y(i), 'g+')
% 
%         % hitch point (should be the same for both)
%         hitch_trail = center(trailer_x(i), trailer_y(i)) * DCM(ang0) * center(-trailer_x(i), -trailer_y(i)) * [trailer_x(i)+L2; trailer_y(i); 1];
%         plot(hitch_trail(1), hitch_trail(2), 'b*')
% 
%         hitch_trac = center(tractor_x(i), tractor_y(i)) * DCM(ang1) * center(-tractor_x(i), -tractor_y(i)) * [tractor_x(i)-h; tractor_y(i); 1];
%         plot(hitch_trac(1), hitch_trac(2), 'g*')
% 
%         xlim([trailer_x(i)-25 trailer_x(i)+25])
%         ylim([ trailer_y(i)-25 trailer_y(i)+25])
%         xlabel('Position in x [m]')
%         ylabel('Position in y [m]')
%         drawnow
%         hold off
%     %     frames(i) = getframe(gcf);
%     end
% 
%     % video = VideoWriter('Single_dubins.avi', 'Motion JPEG AVI');
%     % video.Quality = 50;
%     % open(video)
%     % writeVideo(video, frames);
%     % close(video)
    
end

%% print out summary
fprintf('\n')
fprintf('rms_psi_1: %4.4f +- %4.4f \n', mean(rms_psi_1_log), std(rms_psi_1_log));
fprintf('rms_psi_2: %4.4f +- %4.4f \n', mean(rms_psi_2_log), std(rms_psi_2_log));
fprintf('rms_y_2: %4.4f +- %4.4f \n\n', mean(rms_y2_log), std(rms_y2_log));

fprintf('max_psi_1: %4.4f +- %4.4f \n', mean(max_psi_1_log), std(max_psi_1_log));
fprintf('max_psi_2: %4.4f +- %4.4f \n', mean(max_psi_2_log), std(max_psi_2_log));
fprintf('max_y_2: %4.4f +- %4.4f \n\n', mean(max_y2_log), std(max_y2_log));

fprintf('goal : %d \n', sum(goal_log));