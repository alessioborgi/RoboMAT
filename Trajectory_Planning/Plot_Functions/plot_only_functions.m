% NameFile: plot_only_functions
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 15-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - funcs: A cell array of function handles representing joint trajectories.
% Ex: {@sin, @(x) x.^2}
% - q_in: A vector of initial position values for each function.
% - q_out: A vector of final position values for each function.
% - v_in: A vector of initial velocity values for each function.
% - v_out: A vector of final velocity values for each function.
% - t_start, t_end: Time interval over which to plot the functions.
% Find:
% - Plot the given functions (position) over the specified time interval with the given initial and final position and velocity values.
% - Plot the velocity and acceleration profiles for each function.
%%%%%% END TASK %%%%%%

function plot_only_functions(funcs, q_in, q_out, v_in, v_out, t_i, t_f)
    t = linspace(t_i, t_f, 100);
    n_funcs = length(funcs);
    dt = t(2) - t(1);

    % Plot the position functions
    figure;
    legend_entries = cell(1, n_funcs);
    for i = 1:n_funcs
        q = q_in(i) + (q_out(i) - q_in(i)) * funcs{i}(t) + v_in(i) * t + (v_out(i) - v_in(i)) * (t.^2) / (2 * (t_f - t_i));
        subplot(3, 1, 1); hold on;
        plot(t, q, 'LineWidth', 2);
        legend_entries{i} = sprintf('f%d', i);
    end
    title('Position');
    xlabel('Time');
    ylabel('q(t)');
    legend(legend_entries);
    hold off;

    % Plot the velocity profiles
    for i = 1:n_funcs
        q_vel = (q_out(i) - q_in(i)) * gradient(funcs{i}(t), dt) + v_in(i) + (v_out(i) - v_in(i)) * t / (t_f - t_i);
        subplot(3, 1, 2); hold on;
        plot(t, q_vel, 'LineWidth', 2);
        title('Velocity');
        xlabel('Time');
        ylabel('v(t)');
    end
    hold off;

    % Plot the acceleration profiles
    for i = 1:n_funcs
        q_acc = (q_out(i) - q_in(i)) * gradient(gradient(funcs{i}(t), dt), dt) + (v_out(i) - v_in(i)) / (t_f - t_i);
        subplot(3, 1, 3); hold on;
        plot(t, q_acc, 'LineWidth', 2);
        title('Acceleration');
        xlabel('Time');
        ylabel('a(t)');
    end
    hold off;
end