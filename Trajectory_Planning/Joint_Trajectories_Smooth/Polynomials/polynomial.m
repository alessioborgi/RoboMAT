% NameFile: polynomial
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - qin: Initial position for each of the Joints.
% - qfin: Final position for each of the Joints.
% - vin: Initial velocity.
% - vfin: Final velocity.
% - T: Total time duration.
% - Vmax: Set of Velocity Bounds for each of the Joints.
% - Amax: Set of Acceleration Bounds for each of the Joints.
% - poly: Choose for the type of Smooth Polynomial you want to use:
%         Cubic,Quintic or Seventic. 
% - joints: Set of Joints.

% Find:
% - Symbolic & Numerical Solutions for a,b,c,d,e,f coefficients of the quintic polynomial trajectory.
% - Plot the position, velocity, and acceleration profiles for the given joint trajectories.
% - Check if the velocity and acceleration bounds are satisfied and adjust the time duration accordingly.
%%%%%% END TASK %%%%%%



function [T_min, T_max] = polynomial(qin, qfin, vin, vfin, T, joints, Vmax, Amax, poly)
    % Compute the difference between final and initial positions
    deltaq = qfin - qin;
    disp("The difference delta_q= qfin-qin is: ");
    disp(deltaq),
    
    % Get the number of joints and their names
    num_joints = length(joints);
    legnd = string(joints);

    %%%%% POLYNOMIAL %%%%%
    syms t  % Symbolic time variable

    for i = 1:num_joints
        poly(i) = qin(i) + deltaq(i) * (-2 * (t/T)^3 + 3 * (t/T)^2) + vin(i) * t + vfin(i) * (T - t);
    end
    
    % Compute derivatives of the polynomial
    polydot = diff(poly, t);  % Velocity
    polyddot = diff(polydot, t);  % Acceleration
    polydddot = diff(polyddot, t);  % Jerk
    polyddddot = diff(polydddot, t);  % Snap
    polydddddot = diff(polyddddot, t);  % Crackle
    polyddddddot = diff(polydddddot, t);  % Pop
   
    % Plot the profiles for each joint
    figure('Position', [100, 100, 900, 1200]);
    for num = 1:num_joints
        subplot(7, 1, 1)
        fplot(poly(num), 'LineWidth', 1.5)
        title('Position')
        xlabel('time [s]')
        ylabel('m')
        grid on
        xlim([0 T])
        hold on
        if num == num_joints
            legend(legnd)
            hold off
        end

        subplot(7, 1, 2)
        fplot(polydot(num), 'LineWidth', 1.5)
        title('Velocity')
        xlabel('time [s]')
        ylabel('m/s')
        grid on
        xlim([0 T])
        ylim([-max(Vmax + 2), max(Vmax + 2)])
        hold on
        if num == num_joints
            fplot([Vmax, -Vmax], "-.", 'LineWidth', 1)
            hold off
        end

        subplot(7, 1, 3)
        fplot(polyddot(num), 'LineWidth', 1.5)
        title('Acceleration')
        xlabel('time [s]')
        ylabel('m/s^2')
        grid on
        xlim([0 T])
        ylim([-max(Amax + 2), max(Amax + 2)])
        hold on
        if num == num_joints
            fplot([Amax, -Amax], "-.", 'LineWidth', 1)
            hold off
        end

        subplot(7, 1, 4)
        fplot(polydddot(num), 'LineWidth', 1.5)
        title('Jerk')
        xlabel('time [s]')
        ylabel('m/s^3')
        grid on
        xlim([0 T])
        hold on
        if num == num_joints
            hold off
        end

        subplot(7, 1, 5)
        fplot(polyddddot(num), 'LineWidth', 1.5)
        title('Snap')
        xlabel('time [s]')
        ylabel('m/s^4')
        grid on
        xlim([0 T])
        hold on
        if num == num_joints
            hold off
        end

        subplot(7, 1, 6)
        fplot(polydddddot(num), 'LineWidth', 1.5)
        title('Crackle')
        xlabel('time [s]')
        ylabel('m/s^5')
        grid on
        xlim([0 T])
        hold on
        if num == num_joints
            hold off
        end

        subplot(7, 1, 7)
        fplot(polyddddddot(num), 'LineWidth', 1.5)
        title('Pop')
        xlabel('time [s]')
        ylabel('m/s^6')
        grid on
        xlim([0 T])
        hold on
        if num == num_joints
            hold off
        end
    end

    %%%%% CHECK VELOCITY AND ACCELERATION BOUNDS %%%%%
    tvel = T / 2;  % Time for maximum velocity
    qdot = abs(double(subs(polydot, [t], [tvel])));  % Calculate maximum velocity

    tacc = (0.5 + sqrt(3) / 6) * T;  % Time for maximum acceleration
    qddot = abs(double(subs(polyddot, [t], [tacc])));  % Calculate maximum acceleration

    % Check if velocity and acceleration bounds are satisfied
    qdot <= Vmax.'
    qddot <= Amax.'

    % If bounds are satisfied, decrease time.
    k_min_vel = Vmax.' ./ qdot;  % Scaling factor for velocity
    k_min_acc = sqrt(Amax.' ./ qddot);  % Scaling factor for acceleration
    k_min = min(min(k_min_vel), min(k_min_acc));  % Minimum scaling factor
    T_min = T / k_min;  % Minimum time duration

    % If bounds are not satisfied, increase time.
    k_max_vel = qdot ./ Vmax.';  % Scaling factor for velocity
    k_max_acc = sqrt(qddot ./ Amax.');  % Scaling factor for acceleration
    k_max = max(max(k_max_vel), max(k_max_acc));  % Maximum scaling factor
    T_max = k_max * T;  % Maximum time duration
end









