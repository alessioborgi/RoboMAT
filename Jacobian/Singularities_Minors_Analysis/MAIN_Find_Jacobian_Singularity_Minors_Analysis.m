% NameFile: MAIN_Find_Jacobian_Singularity
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 27-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - J: The Jacobian Matrix.
%   - variables: A vector of variables of the Jacobian (e.g., [q1, q2, q3, q4, q5]).

%   Find:
%   - All the Singularities for the Jacobian Matrix, using Minors_Analysis.

%%%%%% END TASK %%%%%%

clc

% Define symbolic variables
syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 L l1 l2 l3 l4 l5 l6 real 

% Uncomment the following lines to define specific variables and Jacobian for a different problem
% variables = [q1, q2, q3];
% J = [- L*sin(q1 + q3) - q2*sin(q1), cos(q1), -L*sin(q1 + q3);
%      L*cos(q1 + q3) + q2*cos(q1), sin(q1),  L*cos(q1 + q3);
%                               1,       0,               1];

% Example problem: Exam 23-01-2023 N4
J = [- q4*sin(q1 + q3) - q2*sin(q1), cos(q1), -q4*sin(q1 + q3), cos(q1 + q3);
       q4*cos(q1 + q3) + q2*cos(q1), sin(q1),  q4*cos(q1 + q3), sin(q1 + q3);
                                  1,       0,                1,            0];

% Simplify the Jacobian matrix and find its singularities
result = Find_Jacobian_Singularity_Minors_Analysis(simplify(J), variables);

% Display the result
disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ");