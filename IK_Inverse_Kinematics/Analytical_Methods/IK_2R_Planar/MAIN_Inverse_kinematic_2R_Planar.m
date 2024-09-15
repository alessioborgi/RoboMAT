% NameFile: MAIN_Inverse_Kinematics_2R_Planar
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 18-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - l1: This indicates the link 1's length.
%   - l2: This indicates the link 2's length.
%   - px: This indicates the x coordinate of the cartesian end effector position.
%   - py: This indicates the y coordinate of the cartesian end effector position.
%   - pos_neg: This indicates a string that specifies wheter the solution comes
%              from the positive root of the sine or the negative one
%              ("pos" or "neg").

%   Find: 
%   - The angle configurations q1, q2 of 2R Planar Arm.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%
clc 

%syms l1 l2 theta1 theta2 real 

% Example 
% l1 = 1;
% l2 = 1;
% px = l1*cos(theta1) + l2*cos(theta1+theta2);
% py = l1*sin(theta1) + l2*sin(theta1+theta2);
% pos_neg = "pos";

% Exam 21-10-2022
% l1 = 2;
% l2 = 1;
% % Starting Position
% % px = 2+(1/sqrt(2));
% % py = 1/sqrt(2);
% % Final Position
% px = 3/sqrt(2);
% py = -1/sqrt(2);
% pos_neg = "pos";

% Exam 10-06-2022
% l1 = 1;
% l2 = 1;
% % Starting Position
% px = 0.5;
% py = 0.5;
% % Final Position
% % px = 3/sqrt(2);
% % py = -1/sqrt(2);
% pos_neg = "pos";

% Exam 12-06-2024
l1 = 1;
l2 = 1;
% Starting Position
% px = 0.6;
% py = -0.4;
% Final Position
px = 1;
py = 1;
pos_neg = "pos";




%%%%%% MAIN %%%%%%
q = IK_2R_Planar(l1, l2, px, py, pos_neg);
% Extract joint angles q1 and q2
q1 = q(1);
q2 = q(2);
disp('Theta 1 (q1): ');
disp(q1);

disp('Theta 2 (q2): ');
disp(q2);

