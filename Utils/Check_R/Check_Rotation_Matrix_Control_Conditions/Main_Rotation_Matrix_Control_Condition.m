% NameFile: Main_Rotation_Matrix_Control_Conditions
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 22-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given a Matrix A, check whether it is a Rotation Matrix:
%   - Square Matrix Check.
%   - Orthogonality Check.
%   - Norm = 1 Check.
%   - Determinant = 1 Check.

%%%%%% END TASK %%%%%%




%%%%%% PARAMETERS TO SET %%%%%%

% Define your matrix A
A = [
    1/sqrt(3), 1/sqrt(3), 1/sqrt(3);
    1/sqrt(6), -2/sqrt(6), 1/sqrt(6);
    1/sqrt(2), 0, -1/sqrt(2)
];
%%%%%% END PARAMETERS %%%%%%





%%%%%% START PROGRAM %%%%%%

isRotation = isRotationMatrix(A);

%%%%%% END PROGRAM %%%%%%








































































