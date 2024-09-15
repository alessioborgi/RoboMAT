% NameFile: MAIN_DH_to_Rotation
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
%   - rot_to_idx: Index of the frame to which we want to rotate. (e.g., 3).
%                 (Notice that this must be 0 < i <= number of rows of the DH Table )
% 
%   Find: 
%   - The Rotation Matrix from the base up to the i-th frame.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

% Set the DH Table with columns being: 
DHTABLE = [pi/2            0            sym('d1')           q1;
           pi/2            0                   0            q2;
           pi/2            0                  q3             0;
              0     sym('a4')                  0            q4];

rot_to_idx = 4;

%%%%%% END PARAMETERS TO SET %%%%%%
%%%%%% MAIN %%%%%

clc
result = DH_to_Rotation(DHTABLE,rot_to_idx);


%%%%%% END MAIN %%%%%
