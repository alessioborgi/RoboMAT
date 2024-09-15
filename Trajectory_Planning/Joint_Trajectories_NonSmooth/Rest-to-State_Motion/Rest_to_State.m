% NameFile: Rest_to_State.m
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 08-06-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - qi: List of Joint Initial Joint Position.
%    - qf: List of Joint Final Joint Position.
%    - ti: Initial Time.
%    - tf: Final Time.
%    - vf: List of Final Velocities, one for each Joint.

%   Find: Simulation of the motion profile of a robot going from
%   Rest-to-State using a Cubic Polynomial Doubly Normalized.
%   It outputs 7 main plots being: 
%   1. Plot of position wrt time.
%   2. Plot of speed wrt time.
%   3. Plot of acceleration wrt time.

%%%%%% END TASK %%%%%%

function Rest_to_State(qi, qf, ti, tf, vf)

    % Number of joints
    num_joints = length(qi);
    
    % Define the time vector
    t = linspace(ti, tf, 100);
    T = tf - ti;
    
    % Initialize arrays for storing results
    q = zeros(num_joints, length(t));
    q_dot = zeros(num_joints, length(t));
    q_ddot = zeros(num_joints, length(t));
    t_star = zeros(1, num_joints);
    vmax = zeros(1, num_joints);
    
    % Loop over each joint
    for i = 1:num_joints

        % Calculate delta_q for current joint
        delta_q = qf(i) - qi(i);
        
        % Calculate coefficients
        a = 3 - (vf(i) * T / delta_q);
        b = -2 + (vf(i) * T / delta_q);
        
        % Compute the position, velocity, and acceleration for current joint
        q(i, :) = qi(i) + delta_q * (a * ((t - ti) / T).^2 + b * ((t - ti) / T).^3);
        q_dot(i, :) = delta_q / T * (2 * a * ((t - ti) / T) + 3 * b * ((t - ti) / T).^2);
        q_ddot(i, :) = delta_q / (T^2) * (2 * a + 6 * b * ((t - ti) / T));
        
        % Find maximum velocity and the time it occurs for current joint
        if vf(i) / delta_q <= 0
            tau_star = (3 - vf(i) * T / delta_q) / (6 - 3 * vf(i) * T / delta_q);
            t_star(i) = ti + tau_star * T;
            v_star = delta_q / T * (2 * a * tau_star + 3 * b * tau_star^2);
        else
            t_star(i) = tf;
            v_star = vf(i);
        end
        
        vmax(i) = max(abs([v_star, vf(i)]));
    end
    
    % Plot the results
    figure;

    % Plot positions
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

    % Display vmax and t_star for each joint
    for i = 1:num_joints
        fprintf('Joint %d:\n', i);
        fprintf('  Instant tau*: %.4f rad/s\n', tau_star(i));
        fprintf('  Maximum Velocity vmax: %.4f rad/s\n', vmax(i));
        fprintf('  Time of Maximum Velocity t*: %.4f s\n', t_star(i));
    end
end