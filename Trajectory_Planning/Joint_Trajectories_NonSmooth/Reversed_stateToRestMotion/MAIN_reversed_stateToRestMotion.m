% NameFile: MAIN_reversed_stateToRestMotion
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

% Given:
    % - totalTime: Total time of the motion profile.
    % - accelerationValue: Acceleration magnitude during the acceleration phase.
    % - initialVelocity: Initial velocity.
    % - initialPosition: Initial position.
    
    % - constantVelocity: Constant velocity during the coast phase.
    % - accelerationTime: Time duration of the acceleration phase.
    % - coastingTime: Time duration of the coasting phase.

% Find: Simulation of the motion profile of a robot going from
    %   State-to-Rest(i.e., starting from initial velocity != 0).
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

% NOTICE: If motion is Clockwise -> give Negative values for constantVelocity and AccelerationValue

% Given parameters

% Exam 09-09-2022 N4
totalTime = 1.801;             % Total time of the motion profile (s).
accelerationValue = -4;          % Acceleration magnitude during the acceleration phase (m/s^2).
initialVelocity = 1.5;            % Initial Velocity != 0.
initialPosition = pi/2;            % Initial Position != 0.
constantVelocity = -2;          % Constant speed during the coast phase (m/s).
accelerationTime = 0.5;      % Time duration of the acceleration phase (s).
coastingTime = 0.426;               % Time duration of the coasting phase. (s)
% If we want the Reversed State_To_Rest_Motion
stopTime=0.375;
intermediatePosition = 1.852;


%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%
clc
% Call the stateToRestMotion function with the given parameters.
% This function computes various motion profiles (position, velocity, acceleration, jerk, snap, crackle, pop).

%%%%% REVERSED STATE-To-REST %%%%%
% To use when we have to invert the first phase and go back to 0. Stick together an initial phase to get back to 0, and the State to Rest.
[time, position, velocity, acceleration, jerk, snap, crackle, pop] = Reversed_stateToRestMotion(totalTime, stopTime, accelerationValue, initialVelocity, initialPosition, intermediatePosition, constantVelocity, accelerationTime, coastingTime)




