% NameFile: Task_Vector_to_Angular_Jacobian
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - TASK_VECTOR: Task vector describing the end-effector position and orientation.
%              Ex.
%                   TASK_VECTOR = [q2*cos(q1)+q4*cos(q1+q3);
%                                  q2*sin(q1)+q4*sin(q1+q3);
%                                  q1 + q3];
%   - VARIABLES: List of symbolic variables representing the joint variables.
%              Ex.
%                   VARIABLES = [q1, q2, q3, q4];
%   - prismatic_indices: A list of indices of Prismatic Joints. If None, put [].
% 
%   Find: 
%   - The Angular Jacobian for a Robot.

%%%%%% END TASK %%%%%%


function [Ja] = Task_Vector_to_Angular_Jacobian(VARIABLES, prismatic_indices)
    % Function that finds the Angular Jacobian for a robot based on a task vector.
    
    % Define the number of joints.
    N = length(VARIABLES);

    % Initialize the Angular Jacobian matrix.
    Ja = sym(zeros(3, N));

    % Unit vector along the z-axis.
    z = [0; 0; 1];
    Ja(:, 1) = z;

    % Compute the column vectors for the Angular Jacobian matrix.
    for i = 1:N-1
        % Here, we assume an identity rotation matrix for simplicity.
        % In a real scenario, replace with appropriate rotation matrices.
        R = eye(3);
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