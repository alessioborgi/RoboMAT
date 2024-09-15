% NameFile: Main_DH
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - alpha_vector: This is a vector of alphas, representing the twist angle 
%               between joint axes, obtained by projecting on a plane O 
%               orthogonal to the link axis. (ALWAYS CONSTANT)
%   - a_vector: This is a vector of a's, representing the displacement AB 
%               between the joint axes. (ALWAYS CONSTANT)
%   - d_vector: This is a vector of d's, representing displacement CD, i.e.,
%               the displacement between axis of joint i-1 and axis of joint i.
%               This is:
%               - VARIABLE: If the joint i is PRISMATIC.
%               - CONSTANT: If the joint i is REVOLUT.
%   - theta_vector: This is a vector of thetas, representing the angle between 
%               link axes (i-1, and i), obtained by projecting these two axes on a
%               orthogonal plane. 
%               This is:
%               - VARIABLE: If the joint i is REVOLUT.
%               - CONSTANT: If the joint i is PRISMATIC.

%   Find: 
%   - T: This is the product of all the matrices corresponding to each
%        vector of arrays.

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%
syms q1 q2 q3 q4 q5 q6 q7 q8 d1 d2 d3 d4 d5 d6 d7 d8 a1 a2 a3 a4 a5 a6 a7 a8 L N M l1 l2 l3 l4 l5 l6 l7 l8 real 

%%%%%%%%%% MIDTERMS %%%%%%%%%%

% Example: 2R Planar Robot
% syms q1 q2 real
% syms l1 l2 real 
% alpha_vector = [0, 0];
% a_vector = [l1, l2];
% d_vector = [0, 0];
% theta_vector = [q1, q2];

% Example: SCARA Robot
% syms a1 a2 real
% syms d1 d4 real
% syms q1 q2 q3 q4 real 
% alpha_vector = [0, 0, 0, pi];
% a_vector = [a1, a2, 0, 0];
% d_vector = [d1, 0, q3, d4];
% theta_vector = [q1, q2, 0, q4];

% Example: Stanford Manipulator
% syms alpha1 alpha2 alpha4 alpha5 real
% syms d1 d2 q3 d6 real
% syms q1 q2 q4 q5 real 
% alpha_vector = [alpha1, alpha2, 0, alpha4, alpha5, 0];
% a_vector = [0, 0, 0, 0, 0, 0];
% d_vector = [d1, d2, q3, 0, 0, d6];
% theta_vector = [q1, q2, -pi/2, q4, q5, 0];

% Midterm 18-11-2022
% syms d1 q1 q2 q3 a3 real
% alpha_vector = [pi/2, pi/2, 0];clc
% a_vector = [0, 0, a3];
% d_vector = [d1, q2, 0];
% theta_vector = [q1, pi/2, q3];

% Midterm 19-11-2021
% syms q1 q2 q3 q4 N real
% alpha_vector = [0, -pi/2, 0, 0];
% a_vector = [0, N, 0, 0];
% d_vector = [q1, 0, q3, 0];
% theta_vector = [0, q2, 0, q4];

%%%%%%%%%% EXAMS %%%%%%%%%%

% Exam 10-06-2022
% alpha_vector = [0, pi/2, 0];
% a_vector = [L, 0, N];
% d_vector = [0, M, 0];
% theta_vector = [q1, q2, q3];

% Exam 09-09-2022
% alpha_vector = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];
% a_vector = [a1, a2, a3, 0, 0, 0];
% d_vector = [d1, 0, 0, d4, 0, d6];
% theta_vector = [q1, q2, q3, q4, q5, q6];

% Exam 21-10-2022
% alpha_vector = [pi/2, pi/2, 0];
% a_vector = [0, 0, a3];
% d_vector = [d1, q2, 0];
% theta_vector = [q1, pi/2, q3];

% Exam 23-01-2023
% alpha_vector = [0, -pi/2, -pi/2, -pi/2, pi/2, -pi/2, -pi/2, 0];
% a_vector = [a1, a2, 0, 0, 0, 0, 0, 0];
% d_vector = [q1, 0, 0, d4, 0, d6, 0, 0];
% theta_vector = [0, q2, q3, q4, q5, q6, q7, q8];

% Exam 10-07-2023
% alpha_vector = [-pi/2, pi/2, 0];
% a_vector = [0, 0, l3];
% d_vector = [0, q2, 0];
% theta_vector = [q1, 0, q3];

% Exam 11-09-2023
% alpha_vector = [pi/2, -pi/2, pi/2, pi/2];
% a_vector = [0, 0, 0, 0];
% d_vector = [q1, q2, q3, q4];
% theta_vector = [0, 0, 0, pi/2];

% Exam 24-01-2024
% syms q1 q2 q3 q4 q5 q6 a2 a3 d1 d4 d6 real
% alpha_vector = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
% a_vector = [40, 445, 40, 0, 0, 0];
% d_vector = [330, 0, 0, 440, 0, 80];
% theta_vector = [0, 0, 0, 0, 0, 0];

% Exam 24-01-2024
% alpha_vector = [pi/2, 0 pi/2];
% a_vector = [a1, a2, a3];
% d_vector = [d1, 0, 0];
% theta_vector = [q1, q2, q3];
% alpha_vector = [pi/2, 0 pi/2];
% a_vector = [0.04, 0.445, 0.04];
% d_vector = [0.33, 0, 0];
% theta_vector = [q1, q2, q3];

% Exam 16-02-2024
% alpha_vector = [0, pi/2, 0];
% a_vector = [0, 0, 0];
% d_vector = [0, q2, q3];
% theta_vector = [q1, pi/2, 0];

% Exam 12-06-2024
alpha_vector = [pi/2, 0, 0];
a_vector = [0, a2, a3];
d_vector = [0, 0, 0];
theta_vector = [q1, q2, q3];

%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%

%%% Denavit-Hartenberg Direct Kinematics %%%
clc
arrays = [alpha_vector; a_vector; d_vector; theta_vector];
[T, A] = DHMatrix(arrays);

% Printing out the Transformation Matrix.
disp('T: ');
disp(T);

% Printing out the xn axis.
n=T(1:3,1);
disp("n");
disp(n);

% Printing out the yn axis.
s=T(1:3,2);
disp("s");
disp(s);

% Printing out the zn axis.
a=T(1:3,3);
disp("a");
disp(a);

% Printing out the position axis.
p = T(1:3,4);
disp("p");
disp(p);

%%%%%% END PROGRAM %%%%%%




