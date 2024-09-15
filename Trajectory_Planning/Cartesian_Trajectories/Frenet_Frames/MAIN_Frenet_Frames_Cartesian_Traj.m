% NameFile: MAIN_Frenet_Frames_Cartesian_Traj
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%
%   Given:
%    - V: Maximum Velocity.
%    - A: Maximum Acceleration.
%    - path_parametrization_type: The typology of path.
%    - Additional Parameters depending on the Path Typology.
%   * For 'circular':
%       - R: Radius of the circular path.
%   * For 'linear':
%       - p_i: Initial position [x; y; z].
%       - p_f: Final position [x; y; z].
%   * For 'ellipse':
%       - a: Semi-major axis.
%       - b: Semi-minor axis.
%       - phi: Phase shift.
%   * For 'helix':
%       - C: Starting point of the helix [x; y; z].
%       - r: Radius of the helix.
%       - axis: Helix axis direction ('x', 'y', or 'z').
%       - h_s: Helix step.

%   Find: 
%   - p, p_prime, p_second, p_prime_norm, p_second_norm. 
%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%
% Define input parameters

%%%%% PATH PARAMETRIZATION %%%%%

path_parametrization_type = 'helix_z'; % Options: 'linear', 'circle', 'ellipse', 'helix_x', 'helix_y', 'helix_z'
V = 2; % max velocity
A = 4.5; % max acceleration

%%%%% Line %%%%%
% p_i = [1; 0; 0];
% p_f = [1.5; 1.5; 1.5];

%%%%% Circle %%%%%
% Exam 12-06-2023
% R = 1.5; % Radius of the circular path

%%%%% Ellipse %%%%%
% a = 1.5;
% b = 0.75;
% phi = 0; % Phase shift

%%%%% Helix %%%%%
% Exam 24-02-2024
C = [0; 0; r]; % Starting point of the helix (= [0; 0; 0], diverse from [0; 0; 0]).
r = 0.5;
axis = 'z';
h_s = 0.2+0.4; % Helix step

%%%%%%%%%% END PARAMETRIZATION %%%%%%%%%%

clc

%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Additional parameters based on the path type
switch path_parametrization_type
    case 'linear'
        additional_params = {p_i, p_f};
        L = norm(p_f - p_i);

    case 'circle'
        additional_params = {R};
        L = 2 * pi * R;

    case 'ellipse'
        additional_params = {a, b, phi};
        L = 2 * pi * max(a, b); % Approximation for full ellipse

    case {'helix_x', 'helix_y', 'helix_z'}
        additional_params = {C, r, axis, h_s};
        L = 4 * pi; % length of the helix (2 turns)

    otherwise
        error('Unknown path parametrization type');
end



% Calculating vmax and amax.
vmax = min(V / sqrt(r^2 + h^2), sqrt(A / r));
amax = A / sqrt(r^2 + h^2);
disp("The Maximum Velocity V_max is:");
disp(vmax);
disp("The Maximum Acceleration A_max is:");
disp(amax);

% Determining the minimum time T.
if L > (vmax^2 / amax)
    T_star = (L * amax + vmax^2) / (amax * vmax);
else
    T_star = 2 * sqrt(L / amax);
end
disp("The Minimum Time T_min is: ");
disp(T_star);

% Calculating time intervals for the acceleration phases.
Ts = vmax / amax;
disp("The Acceleration/Deceleration Phases' Time T_s is:");
disp(Ts);

%Defining time vector.
t = linspace(0, T_star, 1000);

%Calculating s(t), s_dot(t), and s_ddot(t) using Bang_Coast_Bang profile.
s = zeros(size(t));
s_dot = zeros(size(t));
s_ddot = zeros(size(t));

for i = 1:length(t)
    if t(i) < Ts
        s(i) = 0.5 * amax * t(i)^2;
        s_dot(i) = amax * t(i);
        s_ddot(i) = amax;
    elseif t(i) < (T_star - Ts)
        s(i) = vmax * (t(i) - 0.5 * Ts);
        s_dot(i) = vmax;
        s_ddot(i) = 0;
    else
        s(i) = L - 0.5 * amax * (T_star - t(i))^2;
        s_dot(i) = amax * (T_star - t(i));
        s_ddot(i) = -amax;
    end
end

%Plotting the profiles of s(t), s_dot(t), and s_ddot(t).
figure;
subplot(3, 1, 1);
plot(t, s, 'LineWidth', 2);
title('Position s(t)');
xlabel('Time [s]');
ylabel('s [m]');
grid on;

subplot(3, 1, 2);
plot(t, s_dot, 'LineWidth', 2);
title('Speed s\_dot(t)');
xlabel('Time [s]');
ylabel('s\_dot [m/s]');
grid on;

subplot(3, 1, 3);
plot(t, s_ddot, 'LineWidth', 2);
title('Acceleration s\_ddot(t)');
xlabel('Time [s]');
ylabel('s\_ddot [m/s^2]');
grid on;

% Defining the path p(s) based on the parametrization type.
[p, p_prime, p_second, p_prime_norm, p_second_norm] = Path_Parametrization_Cartesian_Traj(path_parametrization_type, additional_params{:});

% Generating the path for plotting.
s_values = linspace(0, L, 1000);
p_values = arrayfun(p, s_values, 'UniformOutput', false);
p_values = cell2mat(p_values);

% Plotting the path.
figure;
plot3(p_values(1, :), p_values(2, :), p_values(3, :), 'LineWidth', 2);
title('Path of the End-Effector');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;
axis equal;

% Determining the robot base placement.
xb = r;
yb = 2 * pi * h_s;
zb = 0;

fprintf('The robot base should be placed at (xb, yb) = (%.2f, %.2f) [m]\n', xb, yb);