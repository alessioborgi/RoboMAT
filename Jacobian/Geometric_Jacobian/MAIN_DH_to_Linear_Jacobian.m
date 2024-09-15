% NameFile: MAIN_DH_to_Linear_Jacobian
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - DHTABLE: Denavit-Hartenmberg(DH) matrix written in the column order: alpha, a, d, theta.
%              Ex.
%                   DHTABLE = [pi/2            0            sym('d1')           q1;
%                              pi/2            0                   0            q2;
%                              pi/2            0                  q3             0;
%                                 0     sym('a4')                  0            q4];
%   - variables: Array containing the joint variables (e.g., [q1, q2, .., qn]).
% 
%   Find: 
%   - The Linear Jacobian for a Robot.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 l1 l2 l3 l4 l5 l6

% Set the DH Table with columns being: 
DHTABLE = [pi/2            a1                  d1            q1;
              0            a2                   0            q2;
           pi/2            a3                   0            q3;];

variables = [q1, q2, q3];



%%%%%% END PARAMETERS TO SET %%%%%%
%%%%%% MAIN %%%%%

clc
result = simplify(DH_to_Linear_Jacobian(DHTABLE, variables));

%%%%%% END MAIN %%%%%
