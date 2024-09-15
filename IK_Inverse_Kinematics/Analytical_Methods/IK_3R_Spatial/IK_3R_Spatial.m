% NameFile: solve_inverse_kinematics_3R
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

% Description:
% This function computes the inverse kinematics solutions for a 3R spatial
% arm given the lengths of the links and the desired end-effector position.

% Inputs:
% - L: length of link 1.
% - M: length of link 2.
% - N: length of link 3.
% - pd: Desired end-effector position as a 3x1 vector [px; py; pz].

% Output:
% - solutions: A matrix containing all the possible IK solutions, where each
%   row represents a solution in the form [q1, q2, q3].


function solutions = IK_3R_Spatial(L, M, N, pd)

% Solve for q3 using the third component of pd
    s3 = (pd(3) - M) / N;
    if abs(s3) > 1
        error('Desired position is outside the reachable workspace.');
    end
    c3 = sqrt(1 - s3^2);

    % Two possible solutions for q3
    q3_solutions = [atan2(s3, c3), atan2(s3, -c3)];

    solutions = [];

    for q3 = q3_solutions
        % Calculate l2 for q2 solutions
        l2 = N * cos(q3);
        l2_pos = l2;
        l2_neg = -l2;

        % Calculate c2 for q2 solutions
        c2 = (pd(1)^2 + pd(2)^2 - (L^2 + l2_pos^2)) / (2 * L * l2_pos);
        if abs(c2) > 1
            continue;
        end
        s2_pos = sqrt(1 - c2^2);
        s2_neg = -s2_pos;

        % Calculate q2 solutions
        q2_solutions = [atan2(s2_pos, c2), atan2(s2_neg, c2)];

        for q2 = q2_solutions
            % Calculate q1 solutions
            detA = L^2 + l2^2 + 2 * L * l2 * cos(q2);
            s1_pos = (pd(2) * (L + l2 * cos(q2)) - pd(1) * l2 * sin(q2)) / detA;
            c1_pos = (pd(1) * (L + l2 * cos(q2)) + pd(2) * l2 * sin(q2)) / detA;
            q1 = atan2(s1_pos, c1_pos);

            % Collect the solution
            solutions = [solutions; [q1, q2, q3]];
        end
    end
end
