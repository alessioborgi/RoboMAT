% NameFile: MAIN_compute_Min_Time
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 02-06-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - A: Maximum allowed acceleration [m/s^2].
% - timing_law: Choose the type of Smooth Polynomial you want to use:
%               'cubic', 'quintic', or 'seventic'.
% - path_type: Type of path ('circular', 'linear', 'ellipse', 'helix').
% - Additional parameters based on the path type:
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

% Find:
% - Symbolic & Numerical Solutions for the coefficients of the chosen polynomial trajectory.
% - Plot the norm of the Cartesian acceleration and its components for the given path.
% - Check if the acceleration bounds are satisfied and adjust the time duration accordingly.

%%%%%% END TASK %%%%%%


%%%%%%%%%% Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% [m/s^2] Maximum allowed acceleration
% Exam 12-06-2023
A = 3;    % [m/s^2] Maximum allowed acceleration

% Timing law and path type
timing_law = 'cubic'; % Options: 'cubic', 'quintic', 'seventic'
path_type = 'helix'; % Options: 'circular', 'linear', 'ellipse', 'helix'

%%%%% PATH PARAMETRIZATION %%%%%

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
C = [0; 0; r]; % Starting point of the helix (= [0; 0; 0], diverse from [0; 0; 0]).
r = 0.4;
axis = 'y';
h_s = 0.3; % Helix step

%%%%%%%%%% END PARAMETRIZATION %%%%%%%%%%

clc

%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Additional parameters based on the path type
switch path_type
    case 'circular'
        additional_params = {R};
    case 'linear'
        additional_params = {p_i, p_f};
    case 'ellipse'
        additional_params = {a, b, phi};
    case 'helix'
        additional_params = {C, r, axis, h_s};
    otherwise
        error('Unknown path type');
end

% Print the chosen parameters for clarity
disp('Starting computation with the following parameters:');
disp(['Maximum acceleration (A): ', num2str(A)]);
disp(['Timing law: ', timing_law]);
disp(['Path type: ', path_type]);

if strcmp(path_type, 'circular')
    disp(['Radius (R): ', num2str(R)]);
elseif strcmp(path_type, 'linear')
    disp(['Start point (p_i): ', mat2str(p_i)]);
    disp(['End point (p_f): ', mat2str(p_f)]);
elseif strcmp(path_type, 'ellipse')
    disp(['Semi-major axis (a): ', num2str(a)]);
    disp(['Semi-minor axis (b): ', num2str(b)]);
    disp(['Phase shift (phi): ', num2str(phi)]);
elseif strcmp(path_type, 'helix')
    disp(['Starting point (C): ', mat2str(C)]);
    disp(['Radius (r): ', num2str(r)]);
    disp(['Axis: ', axis]);
    disp(['Helix step (h_s): ', num2str(h_s)]);
end

% Call the compute_Min_Time function and get the result
T_min = compute_Min_Time_Rest_to_Rest(A, timing_law, path_type, additional_params{:});








