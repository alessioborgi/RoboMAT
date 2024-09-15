% NameFile: MAIN_State_to_State_Motion.m
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
%    - vi: List of Initial Joint Velocities.
%    - vf: List of Final Joint Velocities.
%    - ti: Initial Time.
%    - tf: Final Time.

%   Find: Simulation of the motion profile of a robot going from
%   State-to-State.
%   It outputs 7 main plots being: 
%   1. Plot of position wrt time.
%   2. Plot of speed wrt time.
%   3. Plot of acceleration wrt time.

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Example
qi = [0, pi/2, 0]; % Initial positions [rad]
qf = [pi, pi, pi/2]; % Final positions [rad]
vi = [1, -2, 1]; % Initial velocities [rad/s]
vf = [-4.5, 3.5, -3.5]; % Final velocities [rad/s]
ti = 1.5; % Initial time [s]
tf = 2.5; % Final time [s]

%%%%%% END PARAMETERS %%%%%%

%%%%%% START PROGRAM %%%%%%

% Calculate the motion trajectories, velocities, and accelerations
State_to_State(qi, qf, vi, vf, ti, tf);

%%%%%% END PROGRAM %%%%%%