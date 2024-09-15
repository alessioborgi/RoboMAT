% NameFile: elem_rot_mat
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - axis: This represents the axis over which we should perform the
%           rotation. ("x", "y", "z").
%   - s: This is the angle taken in RADIANTS (or SYMBOLIC).

%   Find: 
%   - R: The Rotation Matrix.

%%%%%% END TASK %%%%%%
    



%%%%%% START PROGRAM %%%%%%

function R = elem_rot_mat(axis, s)
    
    % Analyse all the possible cases: 
    switch axis

        % X-axis Rotation.
        case {"x", "X"}
            R = [1       0        0;
                 0    cos(s)   -sin(s);
                 0    sin(s)    cos(s)];

        % Y-axis Rotation.
        case {"y", "Y"}
            R = [cos(s)     0   sin(s);
                   0	    1     0;
                -sin(s)     0   cos(s)];

        % Z-axis Rotation.
        case {"z", "Z"}
            R = [cos(s)  -sin(s)    0;
                 sin(s)   cos(s)    0;
                   0        0       1];
        
        % You have inserted a letter that is not one of the three aexes.
        otherwise
            disp("The axes you inserted does not exists. It should be either 'x', 'y', 'z' or any of those capitalized")
    end
    
end