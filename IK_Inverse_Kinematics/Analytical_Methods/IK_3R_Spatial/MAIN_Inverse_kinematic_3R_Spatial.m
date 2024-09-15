% NameFile: MAIN_Inverse_Kinematic_3R_Spatial
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
% - l1: length of link 1.
% - l2: length of link 2.
% - l3: length of link 3.
% - p: Value of the E-E Position, containing px, py and pz.
% - pd: This represents the desired E-E Position.

%   Find: 
%   - The IK Solutions of the 3R Spatial Arm.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

clc;
clear;

% Link lengths and desired end-effector position
L = 0.5; % [m]
M = 0.5; % [m]
N = 0.5; % [m]
pd = [0.3; -0.3; 0.7]; % [m]

%%%%%% END PARAMETERS %%%%%%

% Call the inverse kinematics solver function
solutions = IK_3R_Spatial(L, M, N, pd);

%%%%%% CHECK SOLUTIONS %%%%%%

% Define the direct kinematics function
direct_kinematics = @(q) [L*cos(q(1)) + N*cos(q(1) + q(2))*cos(q(3));
                          L*sin(q(1)) + N*sin(q(1) + q(2))*cos(q(3));
                          M + N*sin(q(3))];

% Check each solution
disp('Inverse Kinematics Solutions and Validation:');
for i = 1:size(solutions, 1)
    q = solutions(i, :);
    p_check = direct_kinematics(q');
    q_deg = rad2deg(q); % Convert solution to degrees
    disp(['Solution ', num2str(i), ': q (rad) = [', num2str(q), '], q (deg) = [', num2str(q_deg), ']']);
    disp(['Computed end-effector position: p = [', num2str(p_check'), ']']);
    disp(['Error: ', num2str(norm(p_check - pd)), ' m']);
    disp(' ');
end

%%%%%% END CHECK SOLUTIONS %%%%%%
