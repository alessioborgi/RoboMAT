% NameFile: Main_Determine_Forces_Polytope
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 05-06-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - tau_max1: This is the first joint torque Maximum Value.
%   - tau_max2: This is the secnd joint torque Maximum Value.               
%   - variables: This is a series of joint variables.
%   - p: These is the Robot Position.
%   - q1_v, q2_v: These are the Joint's Configuration's Values.

%   Find: 
%   - T: This is the product of all the matrices corresponding to each
%        vector of arrays.

%%%%%% END TASK %%%%%%

clc

%%%%%% PARAMETERS TO SET %%%%%%
syms q1 q2 

% Given parameters
tau_max1 = 10; % [Nm]
tau_max2 = 5;  % [N]
variables = [q1, q2];
p = [q2*cos(q1);
     q2*sin(q1)];

% Define the Jacobian matrix at the given configuration
J = jacobian (p, variables);
disp("The Jacobian is equal to: ");
disp(J);

q1_v = pi/3; % [rad]
q2_v = 1.5;  % [m]

%%%%%% END PARAMETERS %%%%%%

%%%%%% START PROGRAM %%%%%%

J = subs(J, [q1,q2], [q1_v, q2_v]);
% Compute the inverse of the transpose of the Jacobian
J_inv_transpose = inv(J');

% Define the four combinations of joint torques
tau_combinations = [-tau_max1, -tau_max2; ... 
                    tau_max1, -tau_max2; ...
                    tau_max1, tau_max2; ...
                    -tau_max1, tau_max2];
                    

% Compute the feasible Cartesian forces
F_feasible = zeros(4, 2);
for i = 1:4
    F_feasible(i, :) = -J_inv_transpose * tau_combinations(i, :)';
end

% Display the results
disp('The Feasible Cartesian Forces Polytope Vertices (Fx, Fy) are:');
disp(F_feasible);

% Plotting the feasible region.
figure;
hold on;

% Plot the torque limits as a rectangle.
rectangle('Position', [-tau_max1, -tau_max2, 2*tau_max1, 2*tau_max2], 'EdgeColor', 'r');

% Plot the feasible forces as a skewed rectangle.
plot([F_feasible(:, 1); F_feasible(1, 1)], [F_feasible(:, 2); F_feasible(1, 2)], 'b-o');

xlabel('Fx [N]');
ylabel('Fy [N]');
title('Feasible Cartesian Forces Region');
grid on;
axis equal;
hold off;














