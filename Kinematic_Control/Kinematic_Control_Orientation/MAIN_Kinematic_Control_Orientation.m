% NameFile: MAIN_Kinematic_Control_Orientation
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - l1, l2,...: A of lengths of the robot planar.
%   - v: Target Velocity. 
%   - beta: Target Angle.
%   - pd_dot: Desired Velocity Vector. 
%   - K_task: Desired Gain Matrix.
%   - f_q: (Task) DK of the Robot.
%   - lambda: Damping Factor used for the Pseudo-Inverse.

%   Find: 
%   The Orientation Trajectories employing Quintic Splines.
%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Define symbolic variables
syms q1 q2 real

% Define the robot parameters
l1 = 0.5; % Length of the first link
l2 = 0.4; % Length of the second link

% Target velocity and angle
v = 0.3; % Target speed [m/s]
beta = -20 * pi / 180; % Target angle with the x-axis in radians

% Desired velocity vector in Cartesian coordinates
pd_dot = v * [cos(beta); sin(beta)];

% Gain matrices for the task
K_task = diag([3, 10]); % Tangential and normal gains

% Direct kinematics
p_x = l1 * cos(q1) + l2 * cos(q1 + q2);
p_y = l1 * sin(q1) + l2 * sin(q1 + q2);
p = [p_x; p_y];

% Damping factor for the pseudoinverse
lambda = 0.01; 

%%%%%% END PARAMETERS %%%%%%

clc

%%%%%% START PROGRAM %%%%% %

% Compute the Jacobian symbolically
J_q_sym = jacobian(p, [q1, q2]);

% Convert symbolic Jacobian to function handle
J_q = matlabFunction(J_q_sym, 'Vars', {q1, q2});

% Rotation matrix for the given beta
R_beta = [cos(beta), sin(beta);
         -sin(beta), cos(beta)];

% Control law using damped least squares pseudoinverse
control_law = @(q, ep) (J_q(q(1), q(2))' * J_q(q(1), q(2)) + lambda * eye(2)) \ (J_q(q(1), q(2))' * (pd_dot + R_beta * K_task * (R_beta' * ep)));

% Simulation parameters
dt = 0.01; % Time step
T = 10; % Total time
q = [pi; 0]; % Initial configuration [rad]
ep = [0; 0]; % Initial position error

% Storage for plotting
Q = zeros(2, T/dt);
E = zeros(2, T/dt);

% Simulation loop
for i = 1:T/dt
    % Calculate current position
    p = double(subs(p, {q1, q2}, {q(1), q(2)}));
    
    % Update position error
    ep = pd_dot * dt * i - p;
    
    % Calculate control input
    q_dot = control_law(q, ep);
    
    % Update joint configuration
    q = q + q_dot * dt;
    
    % Store data for plotting
    Q(:, i) = q;
    E(:, i) = ep;
end

% Plot results
time = 0:dt:T-dt;
figure;
subplot(2, 1, 1);
plot(time, Q(1, :), 'r', time, Q(2, :), 'b');
title('Joint Angles');
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('q1', 'q2');

subplot(2, 1, 2);
plot(time, E(1, :), 'r', time, E(2, :), 'b');
title('Position Errors');
xlabel('Time [s]');
ylabel('Error [m]');
legend('et', 'en');

%%%%%% END PROGRAM %%%%% %

