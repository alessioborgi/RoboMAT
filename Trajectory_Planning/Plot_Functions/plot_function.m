% NameFile: plot_function
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 15-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - funcs: A function representing joint trajectories.
%      Ex: @sin, or @(x) x.^2
% - interval_start, interval_end: An interval over which to plot the functions.

% Find:
% - Plot the given functions over the specified interval.
% - Additionally, plot the velocity, acceleration, jerk, snap, crackle, and pop profiles for each function.

%%%%%% END TASK %%%%%%

function[] = plot_function(f,x_i,x_f)  
   
    % Define the function in the interval.
    x=linspace(x_i,x_f,100);

    % Compute the function values.
    y = f(x);

    % Plot the function.
    figure;
    plot(x, y, 'LineWidth', 2);
    title('Plot of f');
    xlabel('x');
    ylabel('f(x)');

    % Modify the Plot Dims.
    %ylim([-1, 2]);
    grid on;



    