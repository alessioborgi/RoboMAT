% NameFile: Main_Find_Range_Numerical
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - J: A Jacobian Matrix.
%   Find: 
%   - Range: the Range of the Jacobian returned using SVD Decomposition.

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

J = [q2*cos(q1),  sin(q1),  a3*sin(q1);
     q2*sin(q1), -cos(q1), -a3*cos(q1);
              0,        0,           0];

%%%%%% END PARAMETERS TO SET %%%%%%

syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 l1 l2 l3 l4 l5 l6 real

disp("The Jacobian Matrix is: ");
disp(J);
a = Find_Range_Numerical(J);
disp("The Range Space of the Jacobian Matrix is: ");
disp(a);