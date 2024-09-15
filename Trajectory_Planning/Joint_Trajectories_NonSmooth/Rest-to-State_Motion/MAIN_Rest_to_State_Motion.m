% NameFile: MAIN_Rest_to_State_Motion.m
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 08-06-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - qi: List of Joint Initial Joint Position.
%    - qf: List of Joint Final Joint Position.
%    - ti: Initial Time.
%    - tf: Final Time.
%    - vf: List of Final Velocities, one for each Joint.

%   Find: Simulation of the motion profile of a robot going from
%   Rest-to-State.
%   It outputs 7 main plots being: 
%   1. Plot of position wrt time.
%   2. Plot of speed wrt time.
%   3. Plot of acceleration wrt time.

%%%%%% END TASK %%%%%%

clc

%%%%%% PARAMETERS TO SET %%%%%%

% Example
% qi = [0, pi/2, 0]; % Initial positions [rad]
% qf = [pi, pi, pi/2]; % Final positions [rad]
% ti = 1.5; % Initial time [s]
% tf = 2; % Final time [s]
% vf = [-4, -4, -2]; % Final velocities [rad/s]

% Exam 24-01-2024 N6
qi = [pi/2]; % Initial positions [rad]
qf = [pi]; % Final positions [rad]
ti = 1.5; % Initial time [s]
tf = 2; % Final time [s]
vf = [-4]; % Final velocities [rad/s]

%%%%%% END PARAMETERS %%%%%%

%%%%%% START PROGRAM %%%%%%

% Calculate the motion trajectories, velocities, and accelerations
Rest_to_State(qi, qf, ti, tf, vf);

%%%%%% END PROGRAM %%%%%%


