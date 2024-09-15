% NameFile: Main_Direct_Problem_R_theta_r
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 22-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given two data, namely:
%   - theta: An angle(in degrees).
%   - r: A vector in 3 dimensions ([rx, ry, rz]).
%
%   Compute the Rotation Matrix R(theta, r).

%%%%%% END TASK %%%%%%

% addpath("../Check_Rotation_Matrix_Control_Conditions/")


clear all;
clear clc;

%%%%%% PARAMETERS TO SET %%%%%%
% theta: An angle(in degrees).
theta_value = pi/3;
r = [1/sqrt(2);  -1/sqrt(2) ;   0];

%%%%%% END PARAMETERS %%%%%%



%%%%%% START PROGRAM %%%%%%

% Compute the Rotation Matrix.
R = calculateRotationMatrix(theta_value, r);

% Check whether R is a Rotation Matrix.
a = isRotationMatrix(R);

%%%%%% END PROGRAM %%%%%%



