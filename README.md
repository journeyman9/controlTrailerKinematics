# controlTrailerKinematics

This repo designs an LQR controller for a tractor-trailer system driving in reverse and evaluates it over 100 paths pre-generated from Dubins Curves imported as `.txt` files. An animation will render for each of the paths and the code will report the results to the console.

## Dependencies
```
MATLAB 2018a
Simulink
```

## How to run

Simply hit the `Run` button or hit `F5`. 

## Change Driving direction
Simply make positive if you want to drive forward.
```
v1x = -2.012; %[m/s]
```

## Change the `Q` and `R` matrices to design different LQR controllers

```
% R = 1;
% Q = eye(3);
R = 1 / (deg2rad(steer_max).^2);
Q = [1/(deg2rad(2).^2)       0                       0;
     0                   1/(deg2rad(2).^2)           0;
     0                        0                1/(0.1.^2)];
```

Typically one starts with the identity matrices for `Q` and `R`, but one can increase the value along the diagonal to increase the penalization in the cost function.

## Change path

```
for i = 0:99
    clearvars -except A B Bbar C controllability D den e G G1 G2 G3 h H ...
        i K L1 L2 NN num observability Q QN QQ R rho RN RR S steer_max ... 
        sys v1x rms_psi_1_log rms_psi_2_log rms_y2_log max_psi_1_log ...
        max_psi_2_log max_y2_log goal_log 
    track_vector = csvread(sprintf('./dubins_path/dubins_path_%d.txt', i));
```

This is looping through 100 paths in the `/dubins_path` directory. One can simply change the length of the for loop and change the filename to be read.

```
for i = 0:1
    clearvars -except A B Bbar C controllability D den e G G1 G2 G3 h H ...
        i K L1 L2 NN num observability Q QN QQ R rho RN RR S steer_max ... 
        sys v1x rms_psi_1_log rms_psi_2_log rms_y2_log max_psi_1_log ...
        max_psi_2_log max_y2_log goal_log 
    track_vector = csvread('t_lanechange.txt');
```

## path array headers
```
track_vector = [x, y, curvature, \psi, distance]
```
