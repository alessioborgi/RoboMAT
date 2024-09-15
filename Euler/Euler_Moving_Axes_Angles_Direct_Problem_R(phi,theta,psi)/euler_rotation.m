% NameFile: euler_rotation
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - sequence_rotation: A string describing the three rotations over moving 
%                        axes we are interested in. (Ex. "zxz").
%   - angles: A sequence of three angles. (Ex. [90, 40, 60], or [phi,
%             theta, psi].
%   Find: 
%   - R: The Rotation Matrix.

%%%%%% END TASK %%%%%%
    



%%%%%% START PROGRAM %%%%%%

function R = euler_rotation(sequence, angles)
    
    % Check the length of the sequence.
    if strlength(sequence) ~= 3
        disp("The Sequence you are providing is NOT VALID. It must be 3 characters long.")
        return;
    end
    
    % Check whether no two Consecutive Rotations on the same axis..
    sequence = lower(char(sequence));
    if (sequence(2) == sequence(1) || sequence(2) == sequence(3))
        disp("The Sequence you are providing is NOT VALID. You cannot perform Two Consecutive Rotations along the Same Axis (They will sum up simply :) ).")
        return;
    end
    
    % Compute the Rotation Matrix.
    R = elem_rot_mat(sequence(1), angles(1)) * elem_rot_mat(sequence(2), angles(2)) * elem_rot_mat(sequence(3), angles(3));

end

%%%%%% END PROGRAM %%%%%%
