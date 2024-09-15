% NameFile: DLS
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 01-06-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
%   J       - Jacobian matrix of the robot at the current configuration.
%   v       - Desired end-effector velocity.
%   lambda  - Damping factor.
%   tol     - Tolerance for convergence.
%   max_iter- Maximum number of iterations.
%
% Outputs:
%   q_dot     - Joint velocities.
%   iterations- Number of iterations used.
%   norm_diffs- Array of norm differences at each iteration.


% Find:
% - Solves the inverse differential kinematics problem using DLS.
%%%%%% END TASK %%%%%%


function [q_dot, iterations, norm_diffs] = DLS(J, v, lambda, tol, max_iter)
% DAMPED_LEAST_SQUARES_TRAJECTORY Solves the inverse differential kinematics problem using DLS.
% 
% Inputs:
%   J       - Jacobian matrix of the robot at the current configuration
%   v       - Desired end-effector velocity
%   lambda  - Damping factor
%   tol     - Tolerance for convergence
%   max_iter- Maximum number of iterations
%
% Outputs:
%   q_dot     - Joint velocities
%   iterations- Number of iterations used
%   norm_diffs- Array of norm differences at each iteration

    % Initialize variables
    q_dot = zeros(size(J, 2), 1);
    iterations = 0;
    norm_diff = tol + 1;
    norm_diffs = []; % Store norm differences for plotting

    % Iteration loop for DLS
    while norm_diff > tol && iterations < max_iter
        iterations = iterations + 1;

        % Compute the DLS solution
        JtJ = J' * J;
        I = eye(size(JtJ));
        q_dot_new = (JtJ + lambda^2 * I) \ (J' * v);

        % Compute the norm of the difference
        norm_diff = norm(q_dot_new - q_dot);
        norm_diffs = [norm_diffs; norm_diff]; % Append norm difference

        % Update the joint velocities
        q_dot = q_dot_new;
    end

    % Warning if maximum iterations reached
    if iterations >= max_iter
        warning('Maximum number of iterations reached without convergence.');
    end
end