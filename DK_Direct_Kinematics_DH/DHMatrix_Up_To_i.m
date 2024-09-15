% NameFile: DHMatrix_Up_To_i
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - alpha_vector: This is a vector of alphas, representing the twist angle 
%               between joint axes, obtained by projecting on a plane O 
%               orthogonal to the link axis. (ALWAYS CONSTANT)
%   - a_vector: This is a vector of a's, representing the displacement AB 
%               between the joint axes. (ALWAYS CONSTANT)
%   - d_vector: This is a vector of d's, representing displacement CD, i.e.,
%               the displacement between axis of joint i-1 and axis of joint i.
%               This is:
%               - VARIABLE: If the joint i is PRISMATIC.
%               - CONSTANT: If the joint i is REVOLUT.
%   - theta_vector: This is a vector of thetas, representing the angle between 
%               link axes (i-1, and i), obtained by projecting these two axes on a
%               orthogonal plane. 
%               This is:
%               - VARIABLE: If the joint i is REVOLUT.
%               - CONSTANT: If the joint i is PRISMATIC.
%   - last_index: The index over which we want to stop the DH Matrix
%               computation. This is done for giving back intermediate results 
%               in the Dh Process.  

%   Find: 
%   - T: This is the product of all the matrices corresponding to each
%        vector of arrays.

%%%%%% END TASK %%%%%%
    

% Remember that:
% cos(q1 + q2) = cos(q1)*cos(q2) - sin(q1)*sin(q2)
% sin(q1 + q2) = cos(q1)*sin(q2) + cos(q2)*sin(q1)
% making use of the simplify function these are converted automatically


%%%%%% START PROGRAM %%%%%%


function [T, A] = DHMatrix_Up_To_i(arrays, last_index)
    
    % Define an Identity Matrix 4*4.
    T = eye(4);

    % Take the number of joints.
    nums = length(arrays(1, :));
    A = cell(1,nums);       
    
    % Loop over the vectors passed.
    for i = 1:last_index

        % Take the i-th row of values from the DH Parameters Matrix.
        values_vector = arrays(:,i);
        alpha_value = values_vector(1);
        a_value = values_vector(2);
        d_value = values_vector(3);
        theta_value = values_vector(4);
        
        % Build the DH Matrix.
        R = [cos(theta_value)   -cos(alpha_value)*sin(theta_value)       sin(alpha_value)*sin(theta_value)          a_value*cos(theta_value);
             sin(theta_value)    cos(alpha_value)*cos(theta_value)      -sin(alpha_value)*cos(theta_value)          a_value*sin(theta_value);
             0                   sin(alpha_value)                        cos(alpha_value)                           d_value;
             0                   0                                       0                                          1;
             ];

        % Save the value of the Transformation Matrix.
        A{i} = R;
        disp(append(int2str(i-1),"A",int2str(i)));
        %disp(A{i});
        T = T * R;   
        disp(['0T', num2str(i)]);
        disp(simplify(T));

    end
    
    % Simplify if we are passed Symbolic Variables.
    if isa(T, 'sym')
        T = simplify(T);
    end
    
end