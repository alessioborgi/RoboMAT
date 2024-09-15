% NameFile: plot_multiple_functions
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 15-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - funcs: A set of polynomial functions representing joint trajectories.
%      Ex: {@sin, @cos, @(x) x.^2}
% - interval_start, interval_end: An interval over which to plot the functions.

% Find:
% - Plot the given functions over the specified interval.
% - Additionally, plot the velocity, acceleration, jerk, snap, crackle, and pop profiles for each function.

%%%%%% END TASK %%%%%%

function plot_multiple_functions(functions, interval_start, interval_end)

x = linspace(interval_start, interval_end, 1000);

figure('Position', [100, 100, 1200, 800]);

legendEntries = cell(1, length(functions) * 7);
count = 1;

for i = 1:length(functions)
    y = arrayfun(functions{i}, x);
    vel = gradient(y, x(2) - x(1)); % Calculate velocity
    acc = gradient(vel, x(2) - x(1)); % Calculate acceleration
    jerk = gradient(acc, x(2) - x(1)); % Calculate jerk
    snap = gradient(jerk, x(2) - x(1)); % Calculate snap
    crackle = gradient(snap, x(2) - x(1)); % Calculate crackle
    pop = gradient(crackle, x(2) - x(1)); % Calculate pop

    subplot(7, 1, 1);
    p1 = plot(x, y);
    hold on;
    title('Joint Position');
    legendEntries{count} = func2str(functions{i});
    count = count + 1;

    subplot(7, 1, 2);
    p2 = plot(x, vel);
    hold on;
    title('Joint Velocity');
    legendEntries{count} = ['Velocity of ', func2str(functions{i})];
    count = count + 1;

    subplot(7, 1, 3);
    p3 = plot(x, acc);
    hold on;
    title('Joint Acceleration');
    legendEntries{count} = ['Acceleration of ', func2str(functions{i})];
    count = count + 1;

    subplot(7, 1, 4);
    p4 = plot(x, jerk);
    hold on;
    title('Joint Jerk');
    legendEntries{count} = ['Jerk of ', func2str(functions{i})];
    count = count + 1;

    subplot(7, 1, 5);
    p5 = plot(x, snap);
    hold on;
    title('Joint Snap');
    legendEntries{count} = ['Snap of ', func2str(functions{i})];
    count = count + 1;

    subplot(7, 1, 6);
    p6 = plot(x, crackle);
    hold on;
    title('Joint Crackle');
    legendEntries{count} = ['Crackle of ', func2str(functions{i})];
    count = count + 1;

    subplot(7, 1, 7);
    p7 = plot(x, pop);
    hold on;
    title('Joint Pop');
    legendEntries{count} = ['Pop of ', func2str(functions{i})];
    count = count + 1;
end

legend([p1, p2, p3, p4, p5, p6, p7], legendEntries, 'Location', 'eastoutside');
xlabel('Time');
ylabel('Value');
grid on;
hold off;

end