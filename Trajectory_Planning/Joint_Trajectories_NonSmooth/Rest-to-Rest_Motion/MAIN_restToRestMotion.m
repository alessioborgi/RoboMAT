% NameFile: MAIN_restToRestMotion
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - totalTime: Total time of the motion profile. (T)
%    - accelerationValue: Acceleration magnitude during the acceleration
%    phase. (a_max)
%    - speedValue: Constant speed during the coast phase. (v_max)
%    - accelerationTime: Time duration of the acceleration phase. (T_s)

%   Find: Simulation of the motion profile of a robot going from
%   Rest-to-Rest.
%   It outputs 7 main plots being: 
%   1. Plot of position wrt time.
%   2. Plot of speed wrt time.
%   3. Plot of acceleration wrt time.
%   4. Plot of jerk wrt time.
%   5. Plot of snap (jounce) wrt time.
%   6. Plot of crackle wrt time.
%   7. Plot of pop wrt time.

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Given parameters

% Exam 21-10-2022 N2
L = [2; 1]; % Link lengths [m]
Vmax = [2; 1.5]; % Max joint velocities [rad/s]
%%% INVERSE KINEMATICS %%%
% Define initial and final positions in Joint coordinates. 
% If we are given Cartesian Coordinates, please first compute the Joint one
% through IK!!!
q_i = [0; pi/4]; % Initial position [m]
q_f = [-pi/4; pi/2]; % Final position [m]


%%%%%% END PARAMETERS %%%%%%
clc

%%%%%% START PROGRAM %%%%%%
% Call the restToRestMotion function with the given parameters.
% This function computes various motion profiles (position, velocity,
% acceleration, jerk, snap, crackle, pop).

% Compute the minimum time rest-to-rest motion and plot the profiles
restToRestMotion(L, Vmax, q_i, q_f);


