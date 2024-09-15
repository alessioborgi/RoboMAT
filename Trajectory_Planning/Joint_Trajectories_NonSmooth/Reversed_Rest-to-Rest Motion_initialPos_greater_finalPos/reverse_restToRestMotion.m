% NameFile: reverse_restToRestMotion
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - totalTime: Total time of the motion profile.
%    - accelerationValue: Acceleration magnitude during the acceleration phase.
%    - speedValue: Constant speed during the coast phase.
%    - accelerationTime: Time duration of the acceleration phase.

%   Find: Simulation of the motion profile of a robot going from
%   Rest-to-Rest.
%   It outputs 7 main plots being: 
%   1. Plot of position wrt time.
%   2. Plot of speed wrt time.
%   3. Plot of acceleration wrt time.
%   4. Plot of jerk wrt time.
%   5. Plot of snap (jounce) wrt time.
%   6. Plot of crackle wrt time.
%   7. Plot of pop wrt time.

%%%%%% END TASK %%%%%%

function [time, position, velocity, acceleration, jerk, snap, crackle, pop] = reverse_restToRestMotion(totalTime, accelerationValue, speedValue, accelerationTime, initialPosition, finalPosition)
    % This function simulates the motion profile of an object going from rest to rest in the reverse direction
    % Use: when initialPosition > finalPosition

    % Parameters:
    % - totalTime: Total time of the motion profile.
    % - accelerationValue: Magnitude of acceleration during the acceleration and deceleration phases.
    % - speedValue: Constant speed during the coasting phase.
    % - accelerationTime: Time duration of the acceleration and deceleration phases.
    % - initialPosition: Initial position of the object.
    % - finalPosition: Final position of the object.

    % Outputs:
    %  1. Plot of position wrt time
    %  2. Plot of speed wrt time
    %  3. Plot of acceleration wrt time
    %  4. Plot of jerk wrt time
    %  5. Plot of snap (jounce) wrt time
    %  6. Plot of crackle (snap) wrt time
    %  7. Plot of pop (crackle) wrt time
    
    % First get the confirmation that inital position > final position.
    if initialPosition <= finalPosition
        error('Initial position must be greater than final position. Stopping execution.');
    else

        % Reverse acceleration and speed values for motion in the opposite direction
        accelerationValue = -accelerationValue;
        speedValue = -speedValue;
    
        % Calculate coast time
        coastTime = totalTime - 2 * accelerationTime;
    
        % Time vector
        t1 = linspace(0, accelerationTime, 100);
        t2 = linspace(accelerationTime, accelerationTime + coastTime, 100);
        t3 = linspace(accelerationTime + coastTime, totalTime, 100);
        time = [t1, t2, t3];
    
        % Acceleration profile
        a1 = ones(1, 100) * accelerationValue;
        a2 = zeros(1, 100);
        a3 = -ones(1, 100) * accelerationValue;
        acceleration = [a1, a2, a3];
    
        % Velocity profile
        v1 = cumtrapz(t1, a1);
        v2 = ones(1, 100) * speedValue;
        v3 = cumtrapz(t3, a3) + v2(end);
        velocity = [v1, v2, v3];
    
        % Position profile (updated integration with initial and final positions)
        x1 = initialPosition + cumtrapz(t1, v1);
        x2 = cumtrapz(t2, v2) + x1(end);
        x3 = cumtrapz(t3, v3) + x2(end) + finalPosition;
        position = [x1, x2, x3];
    
        % Jerk profile
        jerk = [diff(acceleration)./diff(time), 0];
    
        % Snap (Jounce) profile
        snap = [diff(jerk)./diff(time), 0];
    
        % Crackle (Snap) profile
        crackle = [diff(snap)./diff(time), 0];
    
        % Pop (Crackle) profile
        pop = [diff(crackle)./diff(time), 0];
    
        % Plotting all profiles in a single figure
        figure('Position', [100, 100, 1500, 1200]);
    
        % Plot of position
        subplot(7, 1, 1);
        plot(time, position, 'b.-');
        title('Position vs Time');
        xlabel('Time (s)');
        ylabel('Position');
    
        % Plot of velocity
        subplot(7, 1, 2);
        plot(time, velocity, 'g.-');
        title('Velocity vs Time');
        xlabel('Time (s)');
        ylabel('Velocity');
    
        % Plot of acceleration
        subplot(7, 1, 3);
        plot(time, acceleration, 'r.-');
        title('Acceleration vs Time');
        xlabel('Time (s)');
        ylabel('Acceleration');
    
        % Plot of jerk
        subplot(7, 1, 4);
        plot(time, jerk, 'm.-');
        title('Jerk vs Time');
        xlabel('Time (s)');
        ylabel('Jerk');
    
        % Plot of snap
        subplot(7, 1, 5);
        plot(time, snap, 'c.-');
        title('Snap (Jounce) vs Time');
        xlabel('Time (s)');
        ylabel('Snap (Jounce)');
    
        % Plot of crackle
        subplot(7, 1, 6);
        plot(time, crackle, 'y.-');
        title('Crackle (Snap) vs Time');
        xlabel('Time (s)');
        ylabel('Crackle (Snap)');
    
        % Plot of pop
        subplot(7, 1, 7);
        plot(time, pop, 'k.-');
        title('Pop (Crackle) vs Time');
        xlabel('Time (s)');
        ylabel('Pop (Crackle)');
    end
end
