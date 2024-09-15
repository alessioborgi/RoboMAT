% NameFile: splineplot
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 15-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - tvals: A series of time intervals.
% - qvals: Joint Positions.
% - v1: Initial Velocity.
% - vn: Final Velocity.
% - norm: T/F indicating whether we want to normalize or not the spline.

% Find:
% The Position, Velocity and Acceleration Plot, after having found the
% associated coefficients, for each Joint Variable.

% NOTICE THAT YOU WILL HAVE TO REPEAT THIS FOR EVERY JOINT !!!

%%%%%% END TASK %%%%%%


function splineplot(tvals, qvals, v1, vn, norm)
    % Function used to print the splines, their velocities, and accelerations
    % Example usage:
    % tvals = [1 2 2.5 4];
    % qvals = [45 90 -45 45]; % in degrees
    
    % Check if the number of time points matches the number of position values
    if length(tvals) ~= length(qvals)
        error('The number of time points must match the number of position values.');
    end
    
    % Compute spline coefficients
    coeffs = fliplr(splines(tvals, qvals, v1, vn, norm));
    disp(coeffs)

    
    syms tau t real;
    N = length(tvals);  % Number of time points
    pols = cell(1, N-1);
    
    % Prepare the figure with adjusted size
    figure('Position', [100, 100, 800, 600]);
    
    % Plot Joint Values
    subplot(2, 2, 1);
    hold on
    for i = 1:N-1
        taut = (t - tvals(i)) / (tvals(i+1) - tvals(i));
        pol = poly2sym(coeffs(i, :), tau);
        pol = subs(pol, tau, taut);
        pols{i} = pol;
        fplot(pol, [tvals(i), tvals(i+1)]);
    end
    hold off
    title('Joint Values');
    xlabel('Time');
    ylabel('Position (degrees)');
    legend(arrayfun(@(i) sprintf('Segment %d', i), 1:N-1, 'UniformOutput', false));
    grid on;
    
    % Plot Velocity
    subplot(2, 2, 2);
    hold on
    vels = cell(1, N-1);
    for i = 1:N-1
        vel = diff(pols{i}, t);
        vels{i} = vel;
        fplot(vel, [tvals(i), tvals(i+1)]);
    end
    hold off
    title('Velocities');
    xlabel('Time');
    ylabel('Velocity (degrees/sec)');
    legend(arrayfun(@(i) sprintf('Segment %d', i), 1:N-1, 'UniformOutput', false));
    grid on;
    
    % Plot Acceleration
    subplot(2, 2, [3 4]);
    hold on
    for i = 1:N-1
        acc = diff(vels{i}, t);
        fplot(acc, [tvals(i), tvals(i+1)]);
    end
    hold off
    title('Accelerations');
    xlabel('Time');
    ylabel('Acceleration (degrees/sec^2)');
    legend(arrayfun(@(i) sprintf('Segment %d', i), 1:N-1, 'UniformOutput', false));
    grid on;
end