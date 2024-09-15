% NameFile: rest_to_RestMotion
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


function restToRestMotion(L, Vmax, q_i, q_f)
    % Inverse kinematics to find initial and final joint angles
    qin = q_i;
    qfin = q_f;
    
    % Displacement in joint space
    delta_q = qfin - qin;
    disp("The Displacement in the Joint Space delta_q is: ");
    disp(delta_q);

    % Calculate maximum accelerations for each joint.
    Amax = Vmax.^2 ./ delta_q;
    disp("The Maximum Acceleration for each Joint is: ");
    disp(Amax);

    % Calculate the minimum times for each joint.
    T = sqrt(4 * delta_q ./ Amax);
    disp("The Minumum Time for each Joint is: ");
    disp(T);

    % Total coordinated motion time dictated by slowest joint
    T = max(T); 
    disp("The % Total coordinated motion time dictated by slowest joint, being: ");
    disp(T);
    
    %%% SLOWING DOWN PEAK VELOCITY AND CONSTANT ACCELERATION OF FASTEST JOINTS
    % Determine number of joints.
    num_joints = length(delta_q);
    V = zeros(1, num_joints);
    A = zeros(1, num_joints);

    for i = 1:num_joints
        V(i) = (2 * delta_q(i)) / T;
        A(i) = V(i)^2 / delta_q(i);
    end
    disp("The new Peak Velocities and Constant Accelerations are: ");
    disp("V = ");
    disp(V);
    disp("A = ");
    disp(A);



    %%% PLOTTING %%%
    % Time vector
    time = linspace(0, T, 100);

    % Profiles
    q = zeros(length(L), length(time));
    qdot = zeros(length(L), length(time));
    qddot = zeros(length(L), length(time));
    qdddot = zeros(length(L), length(time));
    qddddot = zeros(length(L), length(time));
    qdddddot = zeros(length(L), length(time));
    qddddddot = zeros(length(L), length(time));

    half_T = T / 2;
    disp("Half_t = ");
    disp(half_T);

    for i = 1:length(time)
        for j = 1:length(L)
            if time(i) < half_T
                q(j, i) = (A(j) * time(i)^2) / 2;
                qdot(j, i) = A(j) * time(i);
                qddot(j, i) = A(j);
                % Compute higher order derivatives (jerk, snap, crackle, pop)
                qdddot = [diff(qddot, 1, 2) zeros(num_joints, 1)] ;
                qddddot = [diff(qdddot, 1, 2) zeros(num_joints, 1)] ;
                qdddddot = [diff(qddddot, 1, 2) zeros(num_joints, 1)] ;
                qddddddot = [diff(qdddddot, 1, 2) zeros(num_joints, 1)] ;
            else
                t_dec = time(i) - half_T;
                q(j, i) = V(j) * T - (V(j).^2 / A(j)) - 0.5 * A(j) * t_dec^2;
                qdot(j, i) = V(j) - A(j) * t_dec;
                qddot(j, i) = -A(j);
                % Compute higher order derivatives (jerk, snap, crackle, pop)
                qdddot = [diff(qddot, 1, 2) zeros(num_joints, 1)] ;
                qddddot = [diff(qdddot, 1, 2) zeros(num_joints, 1)] ;
                qdddddot = [diff(qddddot, 1, 2) zeros(num_joints, 1)] ;
                qddddddot = [diff(qdddddot, 1, 2) zeros(num_joints, 1)] ;
            end
        end
    end

    % Plotting the profiles
    fig = figure;
    set(fig, 'Position', [100, 100, 1500, 1200]);
    subplot(7, 1, 1);
    plot(time, q, 'LineWidth', 2);
    xlabel('time [s]');
    ylabel('q [rad]');
    title('Position profile');
    grid on;
    legend('q_1', 'q_2');

    subplot(7, 1, 2);
    plot(time, qdot, 'LineWidth', 2);
    xlabel('time [s]');
    ylabel('dq/dt [rad/s]');
    title('Velocity profile');
    grid on;
    legend('dq_1/dt', 'dq_2/dt');

    subplot(7, 1, 3);
    plot(time, qddot, 'LineWidth', 2);
    xlabel('time [s]');
    ylabel('d^2q/dt^2 [rad/s^2]');
    title('Acceleration profile');
    grid on;
    legend('d^2q_1/dt^2', 'd^2q_2/dt^2');

    subplot(7, 1, 4);
    plot(time, qdddot, 'LineWidth', 2);
    xlabel('time [s]');
    ylabel('d^3q/dt^3 [rad/s^3]');
    title('Jerk profile');
    grid on;
    legend('d^3q_1/dt^3', 'd^3q_2/dt^3');

    subplot(7, 1, 5);
    plot(time, qddddot, 'LineWidth', 2);
    xlabel('time [s]');
    ylabel('d^4q/dt^4 [rad/s^4]');
    title('Snap profile');
    grid on;
    legend('d^4q_1/dt^4', 'd^4q_2/dt^4');

    subplot(7, 1, 6);
    plot(time, qdddddot, 'LineWidth', 2);
    xlabel('time [s]');
    ylabel('d^5q/dt^5 [rad/s^5]');
    title('Crackle profile');
    grid on;
    legend('d^5q_1/dt^5', 'd^5q_2/dt^5');

    subplot(7, 1, 7);
    plot(time, qddddddot, 'LineWidth', 2);
    xlabel('time [s]');
    ylabel('d^6q/dt^6 [rad/s^6]');
    title('Pop profile');
    grid on;
    legend('d^6q_1/dt^6', 'd^6q_2/dt^6');
end