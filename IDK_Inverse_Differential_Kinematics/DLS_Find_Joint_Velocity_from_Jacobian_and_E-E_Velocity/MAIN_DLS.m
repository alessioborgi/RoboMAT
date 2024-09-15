% NameFile: MAIN_DLS
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

% Define the Jacobian matrix at the current configuration
J = [0.5, 0.2, 0; 0.1, 0.7, 0.3; 0.3, 0.4, 0.8];

% Desired end-effector velocity
v = [0.1; 0.2; 0.3];

% Damping factor
lambda = 0.1;

% Tolerance for convergence
tol = 1e-6;

% Maximum number of iterations
max_iter = 100;

% Solve using Damped Least Squares
[q_dot, iterations, norm_diffs] = DLS(J, v, lambda, tol, max_iter);

% Display results
disp('Joint Velocities:');
disp(q_dot);

disp('Iterations:');
disp(iterations);

% Plotting
figure;
subplot(2, 1, 1);
plot(1:iterations, norm_diffs, '-o');
xlabel('Iteration');
ylabel('Norm Difference');
title('Convergence of DLS');

subplot(2, 1, 2);
plot(repmat((1:iterations)', 1, length(q_dot)), repmat(q_dot', iterations, 1));
xlabel('Iteration');
ylabel('Joint Velocities');
title('Joint Velocities over Iterations');
legend(arrayfun(@(i) sprintf('q\\_dot_%d', i), 1:length(q_dot), 'UniformOutput', false));