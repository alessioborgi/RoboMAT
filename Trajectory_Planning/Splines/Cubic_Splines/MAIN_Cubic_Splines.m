% NameFile: MAIN_Cubic Splines
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 15-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - tvals: A series of time intervals.
% - qvals: Joint Positions.
% - v1: Initial Velocity.
% - vn: Final Velocity.
% - norm: T/F indicating whether we want to normalize or not the spline.

% Find:
% The Position, Velocity and Acceleration Plot, after having found the
% associated coefficients, for each Joint Variable.

% NOTICE THAT YOU WILL HAVE TO REPEAT THIS FOR EVERY JOINT !!!

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

clc;
clear;

% Define T time values 
tvals = [0, 2, 4];  

% Define Joint Position values (for each joint, you will need to do a
% different plot !)
% qvals = [0, 0.5318, 3];  % Joint 1
qvals = [pi/2, 0.2527, pi/2];  % Joint 2

% Define initial and final velocity you want to obtain in the cubic spline.
v1 = 0;
vn = 0;
norm = true;

%%%%%% END PARAMETERS %%%%%%

%%%%%% START PROGRAM %%%%%%

% Call the splineplot function.
splineplot(tvals, qvals, v1, vn, norm);

% Display the generated plots.
disp('Splines, velocities, and accelerations have been plotted.');

