% NameFile: Main_RPY_Angles
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - sequence_rotation: A string describing the three rotations over fixed 
%                        axes we are interested in.
%   - phi_value: The angle of the first rotation. It can be either symbolic
%                or normal numbers.
%   - theta_value: The angle of the second rotation. It can be either symbolic
%                  or normal numbers.
%   - psi_value: The angle of the third rotation. It can be either symbolic
%                or normal numbers.
%   Find: 
%   - R: The Rotation Matrix.

%%%%%% END TASK %%%%%%

clc
%%%%%% PARAMETERS TO SET %%%%%%

% Set the sequence of rotation you want. 
sequence_rotation = "zxy";

% Set the sequence of angles you want.

% SYMBOLIC:
syms alpha beta gamma real
sequence_angles = [alpha, beta, gamma];


% REAL NUMBERS: Passed in Degrees.

% Exam 24-01-2024
phi_value = 45;
theta_value = -60;
psi_value = -60;
sequence_angles = [deg2rad(phi_value), deg2rad(theta_value), deg2rad(psi_value)];


%%%%%% END PARAMETERS %%%%%%




%%%%%% START PROGRAM %%%%%%


% Compute the Rotation Matrix.
R = RPY_Rotation(sequence_rotation, sequence_angles);
disp("The Rotation Matrix R in RPY Representation is: ");
disp(simplify(R));

disp(isRotationMatrix(R));

% Compute eigenvalues and eigenvectors
%[V, D] = eig(R);
%disp(V);

%%%%%% END PROGRAM %%%%%%
