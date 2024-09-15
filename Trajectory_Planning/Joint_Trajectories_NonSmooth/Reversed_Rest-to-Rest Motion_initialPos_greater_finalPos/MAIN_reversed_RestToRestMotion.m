% NameFile: Main_reversed_RestToRestMotion
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%
% This function simulates the motion profile of an object going from rest to rest in the reverse direction
% Use: when initialPosition > finalPosition

%   Given:
%    - totalTime: Total time of the motion profile.
%    - accelerationValue: Acceleration magnitude during the acceleration phase.
%    - speedValue: Constant speed during the coast phase.
%    - accelerationTime: Time duration of the acceleration phase.
%    - initialPosition: Initial position of the object.
%    - finalPosition: Final position of the object.

%   Find: Simulation of the motion profile of a robot going from initial to
%         final position (Rest-to-Rest), when the initial position is greater 
%         than the final position. 
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

% Exam 09-09-2022 N4
% totalTime = 1.2855;             % Total time of the motion profile (s).
% accelerationValue = 4;    % Acceleration magnitude during the acceleration phase (m/s^2).
% speedValue = 2;              % Constant speed during the coast phase (m/s).
% accelerationTime = 0.5;      % Time duration of the acceleration phase (s).
% initialPosition = pi/2;
% finalPosition = 0;

totalTime = 1.801;             % Total time of the motion profile (s).
accelerationValue = 4;    % Acceleration magnitude during the acceleration phase (m/s^2).
speedValue = 2;              % Constant speed during the coast phase (m/s).
accelerationTime = 0.375;      % Time duration of the acceleration phase (s).
initialPosition = pi/2;
finalPosition = 0;

%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%
clc
% Call the restToRestMotion function with the given parameters.
% This function computes various motion profiles (position, velocity,
% acceleration, jerk, snap, crackle, pop).
[time, position, velocity, acceleration, jerk, snap, crackle, pop] = reverse_restToRestMotion(totalTime, accelerationValue, speedValue, accelerationTime, initialPosition, finalPosition)



