% NameFile: Main_Inverse_Problem
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 22-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given the Rotation Matrix R, find:
%   - theta: An angle.
%   - r: A unit vector in 3 dimensions ([rx, ry, rz]).
%   Such that: 
%   𝑅 = 𝒓𝒓𝑇 + 𝐼 − 𝒓𝒓𝑇 cos 𝜃 + 𝑆(𝒓) sin 𝜃 = 𝑅(𝜃, 𝒓)

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Define your rotation matrix R

% Example of STANDARD CASE.
% R = [1/sqrt(3), 1/sqrt(3), 1/sqrt(3);
%     1/sqrt(6), -2/sqrt(6), 1/sqrt(6);
%     1/sqrt(2), 0, -1/sqrt(2)];


% Example of SINGULAR CASE with 2 Solutions (theta=+-pi).
% R = [-1,    0,              0;
%       0,    -1/sqrt(2),     -1/sqrt(2);
%       0,    -1/sqrt(2),     1/sqrt(2)];


% R = [-2, 2, -1;
%     2, 1, -2;
%     -1, -2, -2];
% R = (1/3)*R;


% 
% R = [-0.7071,   -0.7071,         0;
%       0.7071,   -0.7071,         0;
%            0,         0,    1.0000];




% Midterm: 18-11-2022
% R = (1/3)*[-2, 2, -1;
%             2, 1, -2;
%            -1, -2, -2];

% Midterm: 20-11-2020
% R = [0   -0.5000   -0.8660;
%     1.0000         0         0;
%          0   -0.8660    0.5000];

% R = [0.9999   -0.0001   -0.0129;
%    -0.0129   -0.0129   -0.9998;
%    -0.0001    0.9999   -0.0129];

% Exam 03-02-2022 N2 
R = [0.0795   -0.9874   0.1370;
   0.5   -0.0795   -0.8624;
   0.8624    0.1370   0.4874];
%%%%%% END PARAMETERS %%%%%%

clc

%%%%%% START PROGRAM %%%%%%

% First check that the Rotation Matrix you have inserted is a Rotation
% Matrix.
% addpath("../Check_Rotation_Matrix_Control_Conditions/");
is_rotation = isRotationMatrix(R);
disp(" ");

if is_rotation == true
    % Call the function to calculate theta and r values
    [theta_value_1, r1, theta_value_2, r2] = calculateThetaAndR(R);
    
    % Display or use the calculated values as needed
    disp('Theta 1: ');
    disp(theta_value_1);
    disp('r1: ');
    disp(r1);
    
    if ~isempty(theta_value_2)
        disp('Theta 2: ');
        disp(theta_value_2);
        disp('r2: ');
        disp(r2);
    end
end

%%%%%% END PROGRAM %%%%%%

