% NameFile: MAIN_Cubic_Polynomial
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - qin: Initial position.
%    - qfin: Final position.
%    - vin: Initial velocity.
%    - vfin: Final velocity.
%    - T: Total time duration.
%    - print_info: Flag to print symbolic and numerical solutions.

%   Find: 
%   - Symbolic & Numerical Solutions for a,b,c,d coefficients

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Define input parameters
% qin = 0;
% qfin = 10;
% vin = 0;
% vfin = 0;
% T = 5;
% print_info = true;

% Exam 16-02-2024 exercise N3
% Joint 1
qin = pi/2;
qfin = 0;
vin = -2.5;
vfin = -0.3;
T = 2;
print_info = true;

% Joint 2
% qin = 0;
% qfin = 1;
% vin = 0;
% vfin = 0;
% T = 2;
% print_info = true;

%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%

clc

%%%%% CUBIC TRAJECTORY %%%%%
% 1) To call when I have all the Input Parameters: qin, qfin, vin, vfin, T.
[qn] = cubic_poly_double_norm_compute_coeff(qin, qfin, vin, vfin, T, print_info)

% 2) To call when I have a Rest-to-Rest Case, i.e., I know that only q_in=q_out=0
% syms tau qin delta T
% q_tau = delta*(3*tau^2 - 2*tau^3);
% disp("The Cubic Rest-to-Rest Timing Law is equal to: ");
% disp(q_tau);
% q_tau = subs(q_tau, delta, qfin-qin);
% disp(q_tau);

%Compute the first and second derivative of the Cubic-Polynomial Timing Law.
s_prime = (6*delta)/T * (tau-tau^2);
disp("The First Derivative of the Cubic Timing Law is equal to: ");
disp(s_prime);
s_second = (6*delta^2)/T * (1-2*tau);
disp("The Second Derivative of the Cubic Timing Law is equal to: ");
disp(s_second);

%%%%% PLOTTING CUBIC TRAJECTORY %%%%%
interval_start = 0;
interval_end = 1;
q_in = [0, 0];
q_fin = [1, 1];
v_in = [0, 0]; 
v_fin = [0, 0];

joint1 = @(tau) - (2*(pi - 28/5)*tau.^3)/pi + (2*((3*pi)/2 - 53/5)*tau.^2)/pi + (10*tau)/pi;
% joint2 = @(tau) - ((2*pi + 24/5)*tau.^3)/pi + ((3*pi + 49/5)*tau.^2)/pi - (5*tau)/pi;
% funcs = {joint1, joint2};


% Call the plot_multiple_functions function
addpath('/Users/alessioborgi/Library/Mobile Documents/com~apple~CloudDocs/University/Robotics_1/Code/MatLab/Code/Trajectory_Planning/Plot_Functions');
% plot_only_functions(funcs, q_in, q_fin, v_in, v_fin, interval_start, interval_end);

% Call to plot a single function.
q_tau = @(tau) delta*(3*tau.^2 - 2*tau.^3);
plot_function(q_tau,interval_start,interval_end)

