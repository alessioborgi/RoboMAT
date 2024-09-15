% NameFile: State_to_State.m
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 08-06-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%
%   Given:
%    - qi: List of Initial Joint Positions.
%    - qf: List of Final Joint Positions.
%    - vi: List of Initial Joint Velocities.
%    - vf: List of Final Joint Velocities.
%    - ti: Initial Time.
%    - tf: Final Time.

%   Find: Simulation of the motion profile of a robot going from
%   State-to-State using a Cubic Polynomial.
%   It outputs 7 main plots being: 
%   1. Plot of position wrt time.
%   2. Plot of speed wrt time.
%   3. Plot of acceleration wrt time.

%%%%%% END TASK %%%%%%

function State_to_State(qi, qf, vi, vf, ti, tf)

    % Number of joints
    num_joints = length(qi);
    
    % Define the time vector
    t = linspace(ti, tf, 100);
    T = tf - ti;
    
    % Initialize arrays for storing results
    q = zeros(num_joints, length(t));
    q_dot = zeros(num_joints, length(t));
    q_ddot = zeros(num_joints, length(t));
    
    % Loop over each joint
    for i = 1:num_joints
        % Calculate coefficients for cubic polynomial
        a0 = qi(i);
        a1 = vi(i);
        a2 = (3 * (qf(i) - qi(i)) - (2 * vi(i) + vf(i)) * T) / T^2;
        a3 = (2 * (qi(i) - qf(i)) + (vi(i) + vf(i)) * T) / T^3;
        
        % Compute the position, velocity, and acceleration for current joint
        q(i, :) = a0 + a1 * (t - ti) + a2 * (t - ti).^2 + a3 * (t - ti).^3;
        q_dot(i, :) = a1 + 2 * a2 * (t - ti) + 3 * a3 * (t - ti).^2;
        q_ddot(i, :) = 2 * a2 + 6 * a3 * (t - ti);
    end
    
    % Plot positions
    figure;
    subplot(3, 1, 1);
    plot(t, q, 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Position [rad]');
    title('Joint Positions vs Time');
    legend(arrayfun(@(i) sprintf('Joint %d', i), 1:num_joints, 'UniformOutput', false));
    grid on;

    % Plot velocities
    subplot(3, 1, 2);
    plot(t, q_dot, 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Velocity [rad/s]');
    title('Joint Velocities vs Time');
    legend(arrayfun(@(i) sprintf('Joint %d', i), 1:num_joints, 'UniformOutput', false));
    grid on;

    % Plot accelerations
    subplot(3, 1, 3);
    plot(t, q_ddot, 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Acceleration [rad/s^2]');
    title('Joint Accelerations vs Time');
    legend(arrayfun(@(i) sprintf('Joint %d', i), 1:num_joints, 'UniformOutput', false));
    grid on;
end