% NameFile: MAIN_DH_to_Angular_Jacobian
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
%   - prismatic_indices: A list of indices of Prismatic Joints (e.g., [1, 4, 5], corresponding to 
%                        the fact that joints 1,4 and 5 are prismatic. Look where the qs is in the 
%                        third column (the one referring to d).
%                        (Remember to start index from 1)). 
%                        If None, put [].
% 
%   Find: 
%   - The Angular Jacobian for a Robot.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

% Set the DH Table with columns being: 
syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 l1 l2 l3 l4 l5 l6

DHTABLE = [pi/2            a1                  d1            q1;
              0            a2                   0            q2;
           pi/2            a3                   0            q3;];

prismatic_indices = [];

%%%%%% END PARAMETERS TO SET %%%%%%
%%%%%% MAIN %%%%%

clc
result = DH_to_Angular_Jacobian(DHTABLE, prismatic_indices);

%%%%%% END MAIN %%%%%
