% NameFile: compute_Min_Time
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

function T_min = compute_Min_Time_Rest_to_Rest(A, timing_law, path_type, varargin)
    
    delta = pi; % Change this for different path lengths if needed
    
    %%% TIMING LAW %%%
    % Define the timing law functions and their derivatives based on input
    disp(" ");
    disp("The Computations for the Timing Law are: ");
    switch timing_law
        case 'cubic'
            s = @(t, T) delta * (3 * (t/T).^2 - 2 * (t/T).^3);
            s_dot = @(t, T) 6 * delta / T * (t/T - (t/T).^2);
            s_ddot = @(t, T) 6 * delta / T^2 * (1 - 2 * (t/T));
            disp("s: ");
            disp(s);
            disp("s_dot: ");
            disp(s_dot);
            disp("s_ddot ");
            disp(s_ddot);
            
        case 'quintic'
            s = @(t, T) delta * (10 * (t/T).^3 - 15 * (t/T).^4 + 6 * (t/T).^5);
            s_dot = @(t, T) 30 * delta / T * (t/T).^2 .* (1 - 2 * (t/T) + (t/T).^2);
            s_ddot = @(t, T) 60 * delta / T^2 * (t/T) .* (1 - 3 * (t/T) + 2 * (t/T).^2);
            disp("s: ");
            disp(s);
            disp("s_dot: ");
            disp(s_dot);
            disp("s_ddot ");
            disp(s_ddot);

        case 'seventic'
            s = @(t, T) delta * (35 * (t/T).^4 - 84 * (t/T).^5 + 70 * (t/T).^6 - 20 * (t/T).^7);
            s_dot = @(t, T) 140 * delta / T * (t/T).^3 .* (1 - 3 * (t/T) + 3 * (t/T).^2 - (t/T).^3);
            s_ddot = @(t, T) 420 * delta / T^2 * (t/T).^2 .* (1 - 4 * (t/T) + 6 * (t/T).^2 - 3 * (t/T).^3);
            disp("s: ");
            disp(s);
            disp("s_dot: ");
            disp(s_dot);
            disp("s_ddot ");
            disp(s_ddot);

        otherwise
            error('Unknown timing law');
    end
    
    %%% PATH PARAMETRIZATION %%%
    % Define the path type and its derivatives based on input.
    function [p_ddot_x, p_ddot_y, p_ddot_z] = path_acceleration(T)

        time_instants = linspace(0, T, 1000);
        s_dot_val = s_dot(time_instants, T);
        s_ddot_val = s_ddot(time_instants, T);
        
        switch path_type

            case 'circular'
                R = varargin{1};
                p_ddot_x = -R * (s_dot_val.^2 .* cos(s(time_instants, T)) + s_ddot_val .* sin(s(time_instants, T)));
                p_ddot_y = -R * (s_dot_val.^2 .* sin(s(time_instants, T)) - s_ddot_val .* cos(s(time_instants, T)));
                p_ddot_z = [];

                % p_ddot = ["-R * (s_dot_val.^2 .* cos(s(time_instants, T)) + s_ddot_val .* sin(s(time_instants, T)));"; 
                %           "-R * (s_dot_val.^2 .* sin(s(time_instants, T)) - s_ddot_val .* cos(s(time_instants, T)))";
                %           []];
                % disp("p_ddot");
                % disp(p_ddot);
                
            case 'linear'
                p_i = varargin{1};
                p_f = varargin{2};
                direction = p_f - p_i;
                norm_direction = norm(direction);
                unit_direction = direction / norm_direction;
                p_ddot = s_ddot_val .* unit_direction;
                p_ddot_x = p_ddot(1,:);
                p_ddot_y = p_ddot(2,:);
                if length(p_ddot) == 3
                    p_ddot_z = p_ddot(3,:);
                else
                    p_ddot_z = [];
                end

                % p_ddot = ["s_ddot_val * p_f - p_i / norm(p_f - p_i)'[1]"; 
                %           "s_ddot_val * p_f - p_i / norm(p_f - p_i)'[2]"; 
                %           "[]"];
                % disp("p_ddot");
                % disp(p_ddot);

            case 'ellipse'
                a = varargin{1};
                b = varargin{2};
                phi = varargin{3};
                p_ddot_x = -a * (s_dot_val.^2 .* cos(s(time_instants, T) + phi) + s_ddot_val .* sin(s(time_instants, T) + phi));
                p_ddot_y = -b * (s_dot_val.^2 .* sin(s(time_instants, T) + phi) - s_ddot_val .* cos(s(time_instants, T) + phi));
                p_ddot_z = [];

                % p_ddot = ["-a * (s_dot_val.^2 .* cos(s(time_instants, T) + phi) + s_ddot_val .* sin(s(time_instants, T) + phi))"; 
                %           "-b * (s_dot_val.^2 .* sin(s(time_instants, T) + phi) - s_ddot_val .* cos(s(time_instants, T) + phi))"; 
                %           "[]"];
                % disp("p_ddot");
                % disp(p_ddot);

            case 'helix'
                C = varargin{1};
                r = varargin{2};
                axis = varargin{3};
                h_s = varargin{4};
                switch axis
                    case 'x'
                        p_ddot_x = h_s * s_ddot_val + C(1);
                        p_ddot_y = -r * (s_dot_val.^2 .* sin(s(time_instants, T)) + s_ddot_val .* cos(s(time_instants, T))) + C(2);
                        p_ddot_z = -r * (s_dot_val.^2 .* cos(s(time_instants, T)) - s_ddot_val .* sin(s(time_instants, T))) + C(3);
                        
                    case 'y'
                        p_ddot_x = r * (s_dot_val.^2 .* cos(s(time_instants, T)) - s_ddot_val .* sin(s(time_instants, T))) + C(1);
                        p_ddot_y = h_s * s_ddot_val + C(2);
                        p_ddot_z = -r * (s_dot_val.^2 .* sin(s(time_instants, T)) + s_ddot_val .* cos(s(time_instants, T))) + C(3);
                        
                    case 'z'
                        p_ddot_x = -r * (s_dot_val.^2 .* cos(s(time_instants, T)) - s_ddot_val .* sin(s(time_instants, T))) + C(1);
                        p_ddot_y = -r * (s_dot_val.^2 .* sin(s(time_instants, T)) + s_ddot_val .* cos(s(time_instants, T))) + C(2);
                        p_ddot_z = h_s * s_ddot_val + C(3);
                        
                    otherwise
                        error('Unknown helix axis');
                end

            otherwise
                error('Unknown path type');
        end
    end

    % Calculate the Norm of the Acceleration.
    function acc_norm = acceleration_norm(T)
        [p_ddot_x, p_ddot_y, p_ddot_z] = path_acceleration(T);
        if isempty(p_ddot_z)
            acc_norm = sqrt(p_ddot_x.^2 + p_ddot_y.^2);
            % disp("The Norm of the Acceleration is equal to: ");
            % disp(acc_norm);
        else
            acc_norm = sqrt(p_ddot_x.^2 + p_ddot_y.^2 + p_ddot_z.^2);
            % disp("The Norm of the Acceleration is equal to: ");
            % disp(acc_norm);
        end
    end

    % Calculate the Maximum of the Norm of the Acceleration.
    function max_acc = max_acceleration_norm(T)
        acc_norm = acceleration_norm(T);
        max_acc = max(acc_norm);
    end

    % Calculate the Theoretical Minimum Feasible Time T
    switch path_type

        case 'circular'
            R = varargin{1};
            T_theoretical = (3 * pi / 2) * sqrt(R / A);
            disp("T_min = ");
            disp("(3 * pi / 2) * sqrt(R / A)");

        case 'linear'
            p_i = varargin{1};
            p_f = varargin{2};
            distance = norm(p_f - p_i);
            T_theoretical = sqrt(2 * distance / A);
            disp("T_min = ");
            disp("sqrt(2 * norm(p_f - p_i) / A)");

        case 'ellipse'
            a = varargin{1};
            b = varargin{2};
            T_theoretical = sqrt(2 * max(a, b) / A);
            disp("T_min = ");
            disp("sqrt(2 * max(a, b) / A");

        case 'helix'
            r = varargin{2};
            h_s = varargin{4};
            helix_length = sqrt(h_s^2 + (2 * pi * r)^2);
            T_theoretical = sqrt(2 * helix_length / A);
            disp("T_min = ");
            disp("sqrt(2 * sqrt(h_s^2 + (2 * pi * r)^2) / A");

        otherwise
            error('Unknown path type');
    end

    max_acc = max_acceleration_norm(T_theoretical);
    % disp("The maximum of the Norm of the Acceleration is equal to: ");
    % disp(max_acc);

    % Refine T by adjusting it until the acceleration norm is within the bound A
    max_iterations = 1000;  % Maximum number of iterations to prevent infinite loop
    iteration_count = 0;

    while max_acc > A
        T_theoretical = T_theoretical * 1.01;  % Increase T by 1%
        max_acc = max_acceleration_norm(T_theoretical);
        iteration_count = iteration_count + 1;
        if iteration_count > max_iterations
            warning('Maximum number of iterations reached. The acceleration bound may not be satisfied.');
            break;
        end
    end

    T = T_theoretical;

    % Calculate the final acceleration components and norm for plotting
    time_instants = linspace(0, T, 1000);
    s_dot_val = s_dot(time_instants, T);
    s_ddot_val = s_ddot(time_instants, T);
    [p_ddot_x, p_ddot_y, p_ddot_z] = path_acceleration(T);
    
    if isempty(p_ddot_z)
        acceleration_norm_val =sqrt(p_ddot_x.^2 + p_ddot_y.^2);
    else
    acceleration_norm_val = sqrt(p_ddot_x.^2 + p_ddot_y.^2 + p_ddot_z.^2);
    end

    % Plotting the results
    figure;
    
    subplot(2, 1, 1);
    plot(time_instants, acceleration_norm_val);
    title('Norm of the Cartesian Acceleration |p''''(t)|');
    xlabel('Time [s]');
    ylabel('Acceleration [m/s^2]');
    
    subplot(2, 1, 2);
    plot(time_instants, p_ddot_x, 'r', 'DisplayName', 'p''''_x(t)');
    hold on;
    plot(time_instants, p_ddot_y, 'b', 'DisplayName', 'p''''_y(t)');
    if ~isempty(p_ddot_z)
        plot(time_instants, p_ddot_z, 'g', 'DisplayName', 'p''''_z(t)');
    end
    title('Components of the Cartesian Acceleration');
    xlabel('Time [s]');
    ylabel('Acceleration [m/s^2]');
    legend('show');
    hold off;
    
    T_min = T;
    fprintf('Selected timing law: %s\n', timing_law);
    fprintf('Selected path type: %s\n', path_type);
    fprintf('Theoretical minimum time T (initial guess): %.4f seconds\n', T_theoretical);
    fprintf('Final adjusted minimum time T: %.4f seconds\n', T_min);
end