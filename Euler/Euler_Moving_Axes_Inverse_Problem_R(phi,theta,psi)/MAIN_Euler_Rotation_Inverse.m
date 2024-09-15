% NameFile: Main_Euler_Rotation_Inverse
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
sequence_rotation = "zyz";

% Set the Rotation Matrix:

% STANDARD CASE: Examples
% R = [-0.3536,   -0.6124,    0.7071;
%       0.8660,   -0.5000,   -0.0000;
%       0.3536,    0.6124,    0.7071];

% R = [0.4330,   -0.5000,   -0.7500;
%      0.2500,    0.8660,   -0.4330;
%      0.8660,    0.0000,    0.5000];

% SINGULAR CASE: Examples
% R = [-0.7071,   -0.7071,         0;
%       0.7071,   -0.7071,         0;
%            0,         0,    1.0000];

% MIDTERM 18-11-2022:
% R = [         0,         0,   -1.0000;
%     0.5000,   -0.8660,         0;
%    -0.8660,   -0.5000,         0]; 

% MIDTERM 19-11-2021:
% R = [    0   -1.0000         0;
%     0.5000         0    0.8660;
%    -0.8660         0    0.5000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Exam 13-02-2023
% Rc
% R = [sqrt(2)/2, sqrt(2)/2, 0; 
%      -0.5, 0.5,  sqrt(2)/2;
%      0.5,-0.5, sqrt(2)/2 ];

% Rd
R = [sqrt(2)/2, 0.5, 0.5; 
     -sqrt(2)/2, 0.5,  0.5;
     0,-sqrt(2)/2, sqrt(2)/2 ];

%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%
clc
% Compute the Rotation Matrix.
[phi, theta, psi] = Euler_Rotation_Inverse(sequence_rotation, R);
disp(" ");
disp("The Rotation Matrix R in RPY Representation is: ");
disp("phi, theta, psi  = ");
disp([rad2deg(phi), rad2deg(theta), rad2deg(psi)]);

%%%%%% END PROGRAM %%%%%%
