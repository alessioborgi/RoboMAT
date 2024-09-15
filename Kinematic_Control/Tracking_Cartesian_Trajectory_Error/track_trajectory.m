function track_trajectory(params)
    % Extract parameters from the input structure
    end_effector_pos = params.end_effector_pos;
    P1 = params.P1;
    P2 = params.P2;
    vd = params.vd;
    Vmax1 = params.Vmax1;
    Vmax2 = params.Vmax2;
    kP = params.kP;
    forward_kinematics = params.forward_kinematics;
    q = params.q0;

    % Define time step and total simulation time
    dt = 0.01; % time step
    T = norm(P2 - P1) / vd; % total time to travel from P1 to P2

    % Initialize variables
    time = 0:dt:T;
    n = length(time);
    trajectory = zeros(2, n);
    error = zeros(2, n);

    for i = 1:n
        t = time(i);

        % Desired position and velocity
        pd = P1 + vd * t * (P2 - P1) / norm(P2 - P1);
        pd_dot = vd * (P2 - P1) / norm(P2 - P1);

        % Calculate the current end-effector position
        p = forward_kinematics(q);

        % Cartesian error
        ep = pd - p;

        % Jacobian
        J = compute_jacobian(q, forward_kinematics);

        % Control law
        q_dot = J \ (pd_dot + kP * ep);

        % Apply velocity limits
        q_dot(1) = max(min(q_dot(1), Vmax1), -Vmax1);
        q_dot(2) = max(min(q_dot(2), Vmax2), -Vmax2);

        % Update joint angles
        q = q + q_dot * dt;

        % Store trajectory and error
        trajectory(:, i) = forward_kinematics(q);
        error(:, i) = ep;
    end

    % Plot the results
    figure;
    subplot(2, 1, 1);
    plot(trajectory(1, :), trajectory(2, :), 'b', 'LineWidth', 2);
    hold on;
    plot(P1(1), P1(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(P2(1), P2(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('X [m]');
    ylabel('Y [m]');
    title('End-Effector Trajectory');
    legend('Trajectory', 'Start Point', 'End Point');
    grid on;

    subplot(2, 1, 2);
    plot(time, error(1, :), 'r', 'LineWidth', 2);
    hold on;
    plot(time, error(2, :), 'b', 'LineWidth', 2);
    xlabel('Time [s]');
    ylabel('Error [m]');
    title('Tracking Error');
    legend('Error X', 'Error Y');
    grid on;
end

% Function to compute the Jacobian of a 2R robot
function J = compute_jacobian(q, forward_kinematics)
    % Numerical approximation of the Jacobian
    delta = 1e-6;
    q1 = q(1);
    q2 = q(2);

    p0 = forward_kinematics(q);

    q1_delta = [q1 + delta; q2];
    q2_delta = [q1; q2 + delta];

    p1 = forward_kinematics(q1_delta);
    p2 = forward_kinematics(q2_delta);

    J = [(p1 - p0) / delta, (p2 - p0) / delta];
end