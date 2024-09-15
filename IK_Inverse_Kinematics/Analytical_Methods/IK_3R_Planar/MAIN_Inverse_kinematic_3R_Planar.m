% NameFile: MAIN_IK_3R_Planar
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 25-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - l1: length of link 1.
% - l2: length of link 2.
% - l3: length of link 3.
% - px: x coordinate of the cartesian final end effector position.
% - py: y coordinate of the cartesian final end effector position.
% - phi: orientation of the E-E (angle).
% Find:
% - The angle configurations q1, q2, q3 of 3R Planar Arm.
%%%%%% END TASK %%%%%%

syms l1 l2 l3 px py phi real

%%%%%% PARAMETERS TO SET %%%%%%

% Define link lengths
l1 = 0.5; % Link length 1
l2 = 0.5; % Link length 2
l3 = 0.2; % Link length 3

% Define desired end-effector position and orientation
px = 0.3;
py = 0.3;
phi = pi/4; % 45 degrees

%%%%%% MAIN %%%%%%
clc; clear;

% Call the inverse kinematics function
[solutions] = IK_3R_Planar(l1, l2, l3, px, py, phi);

% Print the solutions
fprintf('The solutions for the joint angles are:\n');
fprintf('Solution 1: [%6.4f, %6.4f, %6.4f]\n', solutions(:,1));
fprintf('Solution 2: [%6.4f, %6.4f, %6.4f]\n', solutions(:,2));
