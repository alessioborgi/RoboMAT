% NameFile: plot_trajectory
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - R1, R...: A set of Rotation Matrices.
%   - T1, T2: A set of Time Intervals.

%   Find: 
%   The Orientation Trajectories employing Cubic Splines.
%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

function plot_trajectory(time_vec, s_t, s_dot_t, s_ddot_t, p_t, v_t, a_t, theta_AB, omega_t, alpha_t, r)
    % Plot the timing law s(t), s_dot(t), s_ddot(t)
    figure;
    subplot(3, 1, 1);
    plot(time_vec, s_t, 'LineWidth', 1.5);
    title('Timing law $s(t)$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('$s(t)$', 'Interpreter', 'latex');
    grid on;

    subplot(3, 1, 2);
    plot(time_vec, s_dot_t, 'LineWidth', 1.5);
    title('Pseudo-velocity $\dot{s}(t)$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('$\dot{s}(t)$', 'Interpreter', 'latex');
    grid on;

    subplot(3, 1, 3);
    plot(time_vec, s_ddot_t, 'LineWidth', 1.5);
    title('Pseudo-acceleration $\ddot{s}(t)$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('$\ddot{s}(t)$', 'Interpreter', 'latex');
    grid on;

    % Plot position, velocity, and acceleration
    figure;
    subplot(3, 1, 1);
    plot(time_vec, p_t(1, :), 'b', time_vec, p_t(2, :), 'y', time_vec, p_t(3, :), 'r', 'LineWidth', 1.5);
    title('Position $p(t)$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('Position [m]');
    legend('x', 'y', 'z');
    grid on;

    subplot(3, 1, 2);
    plot(time_vec, v_t(1, :), 'b', time_vec, v_t(2, :), 'y', time_vec, v_t(3, :), 'r', 'LineWidth', 1.5);
    title('Velocity $v(t)$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    legend('x', 'y', 'z');
    grid on;

    subplot(3, 1, 3);
    plot(time_vec, a_t(1, :), 'b', time_vec, a_t(2, :), 'y', time_vec, a_t(3, :), 'r', 'LineWidth', 1.5);
    title('Acceleration $a(t)$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('Acceleration [m/s^2]');
    legend('x', 'y', 'z');
    grid on;

    % Plot angle, angular velocity, and angular acceleration
    figure;
    subplot(3, 1, 1);
    plot(time_vec, theta_AB * s_t, 'LineWidth', 1.5);
    title('Angle around unit axis $r$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('Angle [rad]', 'Interpreter', 'latex');
    grid on;

    subplot(3, 1, 2);
    plot(time_vec, omega_t(1, :), 'b', time_vec, omega_t(2, :), 'y', time_vec, omega_t(3, :), 'r', 'LineWidth', 1.5);
    title('Angular velocity $\omega(t)$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('Angular velocity [rad/s]', 'Interpreter', 'latex');
    legend('x', 'y', 'z');
    grid on;

    subplot(3, 1, 3);
    plot(time_vec, alpha_t(1, :), 'b', time_vec, alpha_t(2, :), 'y', time_vec, alpha_t(3, :), 'r', 'LineWidth', 1.5);
    title('Angular acceleration $\alpha(t)$', 'Interpreter', 'latex');
    xlabel('Time [s]');
    ylabel('Angular acceleration [rad/s^2]', 'Interpreter', 'latex');
    legend('x', 'y', 'z');
    grid on;
end

function [s_dot_max, s_ddot_max] = compute_max_values(s_dot, s_ddot, T)
    % Use fminbnd to find the minimum value of negative s_dot and s_ddot in the interval [0, T]
    % because fminbnd finds the minimum, we look for the negative to find the maximum
    s_dot_func = matlabFunction(s_dot);
    s_ddot_func = matlabFunction(s_ddot);

    [s_dot_max_time, s_dot_max_neg] = fminbnd(@(t) -s_dot_func(t), 0, T);
    s_dot_max = -s_dot_max_neg;

    [s_ddot_max_time, s_ddot_max_neg] = fminbnd(@(t) -s_ddot_func(t), 0, T);
    s_ddot_max = -s_ddot_max_neg;
end


