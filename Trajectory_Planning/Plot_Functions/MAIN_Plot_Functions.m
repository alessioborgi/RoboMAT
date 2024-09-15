% NameFile: MAIN_Plot_Functions
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 15-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - funcs: A set of quintic polynomial functions representing joint trajectories.
%      Ex: {@sin, @cos, @(x) x.^2}
% - interval_start, interval_end: An interval over which to plot the functions.

% Find:
% - Plot the given functions over the specified interval.
% - Additionally, plot the velocity, acceleration, jerk, snap, crackle, and pop profiles for each function.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%
% Define the functions to plot
joint1 = @(tau) -1.2876*tau.^5 + 4.219*tau.^4 - 4.146*tau.^3 + 2.0*tau - 0.7854;
joint2 = @(tau) 1.2876*tau.^5 - 4.219*tau.^4 + 4.146*tau.^3 - 2.0*tau + 0.7854;
joint3 = @(tau) 0.7854;

% Store the function handles in a cell array
funcs = {joint1, joint2, joint3};

% Define the single function to plot.
f = @(tau) -1.2876*tau.^5 + 4.219*tau.^4 - 4.146*tau.^3 + 2.0*tau - 0.7854;
% Define the interval
interval_start = 0;
interval_end = 2;

%%%%%% END PARAMETERS %%%%%%

%%%%%% START PROGRAM %%%%%%

% Call the plot_multiple_functions function
plot_only_functions(funcs, interval_start, interval_end);

% Call to plot a single function.
%plot_function(f,interval_start,interval_end)  


