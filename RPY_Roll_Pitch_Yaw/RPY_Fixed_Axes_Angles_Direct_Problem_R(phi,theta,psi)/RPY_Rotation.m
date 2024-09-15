% NameFile: RPY_Rotation
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - sequence: A string describing the three rotations over fixed axes we 
%               are interested in.
%   - angles: The sequence of three rotation angles.

%   Find: 
%   - R: The Rotation Matrix.

%%%%%% END TASK %%%%%%


function R = RPY_Rotation(sequence, angles)

    % RPY rotations work about fixed-axes
    sequence = char(sequence);

    % Pass the sequence Flipped to the Euler Rotation.
    % Indeed, we know that Euler Rotation = RPY Rotation with rotation
    % inverted.
    R = euler_rotation(flip(sequence), flip(angles));
    
end