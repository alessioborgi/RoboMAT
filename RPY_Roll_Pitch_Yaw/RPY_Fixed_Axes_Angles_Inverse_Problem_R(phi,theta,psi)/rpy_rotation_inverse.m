% NameFile: rpy_rotation_inverse
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - sequence_rotation: A string describing the three rotations over fixed 
%                        axes we are interested in.
%   - R: The Rotation Matrix.

%   Find: 
%   - phi_value: The angle of the first rotation. It can be either symbolic
%                or normal numbers.
%   - theta_value: The angle of the second rotation. It can be either symbolic
%                or normal numbers.
%   - psi_value: The angle of the third rotation. It can be either symbolic
%                or normal numbers.

%%%%%% END TASK %%%%%%


function [phi, theta, psi] = rpy_rotation_inverse(sequence, R)
    
    % Obtain the sequence and pass it to the Euler Function flipped.
    sequence = char(sequence);
    disp(flip(sequence));
    [phi, theta, psi] = Euler_Rotation_Inverse(flip(sequence), R);

















































































