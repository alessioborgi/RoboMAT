% NameFile: MAIN_Orientation_Trajectories_Axis_Angle
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
%   The Orientation Trajectories employing Quintic Splines.
%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Exam 11-06-2021 N3

% Define points A and B
A = [1, 1, 1]';
B = [-1, 5, 0]';

% Define rotation matrices RA and RB
RA = [0 1 0; 1 0 0; 0 0 -1];
RB = [-1/sqrt(2) 0 1/sqrt(2); 0 -1 0; 1/sqrt(2) 0 1/sqrt(2)];

% Total motion time
T = 2.5;


%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%% %

% Compute linear trajectory
p_diff = B - A;
L = norm(p_diff);

% Quintic polynomial timing law
syms t;
s = 6*(t/T)^5 - 15*(t/T)^4 + 10*(t/T)^3;
s_dot = diff(s, t);
s_ddot = diff(s_dot, t);

% Define time vector
time_vec = linspace(0, T, 100);

% Compute position, velocity, and acceleration
s_t = double(subs(s, t, time_vec));
s_dot_t = double(subs(s_dot, t, time_vec));
s_ddot_t = double(subs(s_ddot, t, time_vec));

p_t = A + (B - A) .* s_t;
v_t = (B - A) .* s_dot_t;
a_t = (B - A) .* s_ddot_t;

% Compute maximum pseudo-velocity and pseudo-acceleration from the document
[s_dot_max, s_ddot_max] = compute_max_values(s_dot, s_ddot, T);

% Compute maximum values for velocity and acceleration
v_max = norm(p_diff) * s_dot_max;
a_max = norm(p_diff) * s_ddot_max;

% Compute the relative orientation matrix
R_AB = RA' * RB;

% Compute axis-angle representation
theta_AB = acos((trace(R_AB) - 1) / 2);
r = 1/(2*sin(theta_AB)) * [R_AB(3,2) - R_AB(2,3); R_AB(1,3) - R_AB(3,1); R_AB(2,1) - R_AB(1,2)];

% Compute angular velocity and acceleration
omega_t = theta_AB * s_dot_t .* r;
alpha_t = theta_AB * s_ddot_t .* r;

% Maximum angular velocity and acceleration
omega_max = theta_AB * s_dot_max * norm(r);
alpha_max = theta_AB * s_ddot_max * norm(r);

% Compute midpoint orientation
theta_mid = theta_AB / 2;
R_mid = RA * (cos(theta_mid) * eye(3) + sin(theta_mid) * skew_symmetric(r) + (1 - cos(theta_mid)) * (r * r'));

% Display results
fprintf('Maximum linear velocity: %.4f m/s\n', v_max);
fprintf('Maximum linear acceleration: %.4f m/s^2\n', a_max);
fprintf('Maximum angular velocity: %.4f rad/s\n', omega_max);
fprintf('Maximum angular acceleration: %.4f rad/s^2\n', alpha_max);
disp('Orientation at midpoint (R_mid):');
disp(R_mid);

% Plotting the profiles
plot_trajectory(time_vec, s_t, s_dot_t, s_ddot_t, p_t, v_t, a_t, theta_AB, omega_t, alpha_t, r);

% Helper functions
addpath('functions_and_plots'); % Add the directory containing the helper functions to the path

function S = skew_symmetric(v)
    S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end


%%%%%% END PROGRAM %%%%% %