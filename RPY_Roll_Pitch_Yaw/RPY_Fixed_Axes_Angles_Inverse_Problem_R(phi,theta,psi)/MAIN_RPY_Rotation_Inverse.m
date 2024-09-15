% NameFile: Main_RPY_Rotation_Inverse
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
%   - R: The Rotation Matrix.

%   Find: 
%   - phi_value: The angle of the first rotation. It can be either symbolic
%                or normal numbers.
%   - theta_value: The angle of the second rotation. It can be either symbolic
%                or normal numbers.
%   - psi_value: The angle of the third rotation. It can be either symbolic
%                or normal numbers.

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Set the sequence of rotation you want. 
sequence_rotation = "zxy";

% Set the Rotation Matrix:
% R = [-0.3536,   -0.6124,    0.7071;
%       0.8660,   -0.5000,   -0.0000;
%       0.3536,    0.6124,    0.7071];

% Exam 24-01-2024
R = [(5*sqrt(2))/8 ,         sqrt(2)/8,    -sqrt(3)/4;
          sqrt(2)/4,         sqrt(2)/4,     sqrt(3)/2;
          sqrt(6)/8,    (-3*sqrt(6))/8,          1/4];


%%%%%% END PARAMETERS %%%%%%



%%%%%% START PROGRAM %%%%%%

% Compute the Rotation Matrix.
[psi, theta, phi] = rpy_rotation_inverse(sequence_rotation, R);
disp(" ");
disp("The Rotation Matrix R in RPY Representation is: ");
disp("phi, theta, psi  = ");
disp([rad2deg(phi), rad2deg(theta), rad2deg(psi)]);
disp("or equivalently: ");
disp([phi, theta, psi]);

%%%%%% END PROGRAM %%%%%%
