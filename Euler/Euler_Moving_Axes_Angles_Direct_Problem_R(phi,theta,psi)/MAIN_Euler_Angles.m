% NameFile: Main_Euler_Angles
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - sequence_rotation: A string describing the three rotations over moving 
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


%%%%%% PARAMETERS TO SET %%%%%%

% Set the sequence of rotation you want. 
sequence_rotation = "zxz";

% Set the sequence of angles you want.

% SYMBOLIC:
syms phi_value theta_value psi_value real
sequence_angles = [phi_value, theta_value, psi_value];


% REAL NUMBERS: Passed in Degrees.
% phi_value = -90;
% theta_value = 60;
% psi_value = 60;
% sequence_angles = [deg2rad(phi_value), deg2rad(theta_value), deg2rad(psi_value)];


%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%

% Compute the Rotation Matrix.
R = euler_rotation(sequence_rotation, sequence_angles);
disp(" ");
disp("Given the sequence of angles (phi, theta, psi): ");
disp([phi_value, theta_value, psi_value]);
disp("And with Sequence of Rotation: ");
disp(sequence_rotation);
disp(" ");
disp("The Rotation Matrix R in Euler Representation is: ");
disp(" ");
disp(R);

%%%%%% END PROGRAM %%%%%%
