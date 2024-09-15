% NameFile: IK_2R_Planar
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 18-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - l1: This indicates the link 1's length.
%   - l2: This indicates the link 2's length.
%   - px: This indicates the x coordinate of the cartesian end effector position.
%   - py: This indicates the y coordinate of the cartesian end effector position.
%   - pos_neg: This indicates a string that specifies wheter the solution comes
%              from the positive root of the sine or the negative one
%              ("pos" or "neg").

%   Find: 
%   - The angle configurations q1, q2 of 2R Planar Arm.

%%%%%% END TASK %%%%%%


function [angles] = IK_2R_Planar(l1,l2,px,py,pos_neg)
    
    %Find the cosine of the second angle
    c2 = (px^2 + py^2 - (l1^2 + l2^2)) / (2 * l1 * l2);
    fprintf("cos(q2)=%");
    disp(c2);

    %Find the sin of the second angle depending on the sign chosen
    if strcmp(pos_neg, 'pos')
        s2 = sqrt(1 - c2^2);
    elseif strcmp(pos_neg, 'neg')
        s2 = -sqrt(1 - c2^2);
    else
        error('Invalid pos_neg argument. Use "pos" or "neg".');
    end

    fprintf("sin(q2)=%");
    disp(s2)
    
    %compute the value of q2
    q2 = atan2(s2, c2);
    
    %compute the value of q2
    q1 = atan2(py, px) - atan2(l2 * s2, l1 + l2 * c2);
    angles=[q1,q2];
end