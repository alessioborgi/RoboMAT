% NameFile: MAIN_Differential_Kinematics
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - J: The (Geometric) Jacobian.
%   - joint_vel: A Vector containing the Joint Velocities 
%     (in the Joint Space).
% 
%   Find: 
%   - Computes the Differential Kinematics Problem, by returning the E-E 
%     (Cartesian Space) Velocities. 

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%
syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 l M N l1 l2 l3 l4 l5 l6 real 

% Exam 12-06-2023
J = [       0,       0,       0,    0
 a1 + a3 + a4, a3 + a4,       0,    0;
            0,       0, a3 + a4,   a4;
            0,       0,       0,    0;
            0,       0,      -1,   -1;
            1,       1,       0,    0];

% joint_vel = [-(a3 + a4 - 3)/a1;
%          (a1 + a3 + a4 - 3)/a1;
%                          -3/a3;
%                          3/a3];

% joint_vel = [ -(a3 + a4)/a1;
%           (a1 + a3 + a4)/a1;
%                        1/a3;
%                       -1/a3;];


%%%%%% END PARAMETERS TO SET %%%%%%


% Start Direct Differential Kinematics
clc
disp("Given the Jacobian: ");
disp(J);
disp("And given the Joint Velocity: ");
disp(joint_vel);

ee_vel = J*joint_vel;
disp("The E-E Velocity (Differential Kinematics Solution) is: ");
disp(simplify(ee_vel));













