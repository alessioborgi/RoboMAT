% NameFile: Task_Vector_to_Angular_Jacobian
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - variables: List of symbolic variables representing the joint variables.
%              Ex.
%                   variables = [q1, q2, q3, q4];
%   - prismatic_indices: A list of indices of Prismatic Joints. If None, put [].
% 
%   Find: 
%   - The Angular Jacobian for a Robot.

%%%%%% END TASK %%%%%%


function [Ja] = Task_Vector_to_Angular_Jacobian(variables, prismatic_indices)
    % Function that finds the Angular Jacobian for a robot based on a task vector.
    
    % Define the number of joints.
    N = length(variables);

    % Initialize the Angular Jacobian matrix.
    Ja = sym(zeros(3, N));

    % Compute transformation matrices for each joint based on the task vector.
    % Assuming identity matrices for rotations initially.
    T = eye(4);
    rotation_0_to_i = cell(1, N);
    for i = 1:N
        % Calculate the transformation matrix for each joint
        T_i = eye(4); % Modify this to the correct transformation for your robot
        T = T * T_i;
        rotation_0_to_i{i} = T(1:3, 1:3);
    end

    % Unit vector along the z-axis.
    z = [0; 0; 1];
    Ja(:, 1) = z;

    % Compute the column vectors for the Angular Jacobian matrix.
    for i = 1:N-1
        R = rotation_0_to_i{i};
        zi = R * z;
        Ja(:, i+1) = zi;
    end

    % Substitute columns with zeros for prismatic joints.
    for prismatic_index = prismatic_indices
        if prismatic_index <= N
            Ja(:, prismatic_index) = zeros(3, 1);
        end
    end

    % Display or return the Jacobian matrix as needed.
    disp('The Angular Jacobian Matrix is:');
    disp(Ja);
end