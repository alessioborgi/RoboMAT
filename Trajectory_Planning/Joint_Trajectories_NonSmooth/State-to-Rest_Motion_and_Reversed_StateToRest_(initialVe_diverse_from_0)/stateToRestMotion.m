% NameFile: state_to_RestMotion
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

% Given:
    % - totalTime: Total time of the motion profile.
    % - accelerationValue: Acceleration magnitude during the acceleration phase.
    % - initialVelocity: Initial velocity.
    % - initialPosition: Initial position.
    
    % - constantVelocity: Constant velocity during the coast phase.
    % - accelerationTime: Time duration of the acceleration phase.
    % - coastingTime: Time duration of the coasting phase.

% Find: Simulation of the motion profile of a robot going from
    %   State-to-Rest(i.e., starting from initial velocity != 0).
    %   It outputs 7 main plots being: 

    %   1. Plot of position wrt time.
    %   2. Plot of speed wrt time.
    %   3. Plot of acceleration wrt time.
    %   4. Plot of jerk wrt time.
    %   5. Plot of snap (jounce) wrt time.
    %   6. Plot of crackle wrt time.
    %   7. Plot of pop wrt time.

%%%%%% END TASK %%%%%%

function [time, position, velocity, acceleration, jerk, snap, crackle, pop] = stateToRestMotion(totalTime, accelerationValue, initialVelocity, initialPosition, constantVelocity, accelerationTime, coastingTime)
    
    % Calculate coast time
    coastTime = coastingTime;

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
    v1 = initialVelocity + cumtrapz(t1, a1);
    
    % Ensure that the final velocity during coasting is zero
    v2 = ones(1, 100) * max(0, constantVelocity) + v1(end);
    
    v3 = cumtrapz(t3, a3) + v2(end);
    velocity = [v1, v2, v3];

    % Position profile (updated integration)
    x1 = initialPosition + cumtrapz(t1, v1);
    x2 = cumtrapz(t2, v2) + x1(end);
    x3 = x2(end) + cumtrapz(t3, v3);
    position = [x1, x2, x3];

    % Computation of the Jerk Profile
    jerk = [diff(acceleration)./diff(time), 0];

    % Computation of the Snap (Jounce) Profile
    snap = [diff(jerk)./diff(time), 0];
    
    % Computation of the Crackle (Snap) Profile
    crackle = [diff(snap)./diff(time), 0];

    % Computation of the Pop (Crackle) Profile
    pop = [diff(crackle)./diff(time), 0];

    % Plotting the profiles:
    % 1) Position
    figure('Position', [100, 100, 1500, 1200]);
    subplot(7, 1, 1);
    plot(time, position, 'b.-');
    title('Position vs Time');
    xlabel('Time (s)');
    ylabel('Position');
    
    % 2) Velocity 
    subplot(7, 1, 2);
    plot(time, velocity, 'g.-');
    title('Velocity vs Time');
    xlabel('Time (s)');
    ylabel('Velocity');
    
    % 3) Acceleration
    subplot(7, 1, 3);
    plot(time, acceleration, 'r.-');
    title('Acceleration vs Time');
    xlabel('Time (s)');
    ylabel('Acceleration');
    
    % 4) Jerk
    subplot(7, 1, 4);
    plot(time, jerk, 'm.-');
    title('Jerk vs Time');
    xlabel('Time (s)');
    ylabel('Jerk');
    
    % 5) Snap (Jounce)
    subplot(7, 1, 5);
    plot(time, snap, 'c.-');
    title('Snap (Jounce) vs Time');
    xlabel('Time (s)');
    ylabel('Snap (Jounce)');
    
    % 6) Crackle (Snap)
    subplot(7, 1, 6);
    plot(time, crackle, 'y.-');
    title('Crackle vs Time');
    xlabel('Time (s)');
    ylabel('Crackle');
    
    % 7) Pop (Crackle)
    subplot(7, 1, 7);
    plot(time, pop, 'k.-');
    title('Pop vs Time');
    xlabel('Time (s)');
    ylabel('Pop');
end
