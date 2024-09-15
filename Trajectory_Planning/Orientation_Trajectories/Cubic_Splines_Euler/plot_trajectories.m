function plot_trajectories(alpha1, alphavia, alpha2, beta1, betavia, beta2, gamma1, gammavia, gamma2, T1, T2)
    % Define the cubic spline coefficients
    [a1_alpha, b1_alpha, c1_alpha, d1_alpha, a2_alpha, b2_alpha, c2_alpha, d2_alpha] = cubic_spline(alpha1, alphavia, alpha2, T1, T2);
    [a1_beta, b1_beta, c1_beta, d1_beta, a2_beta, b2_beta, c2_beta, d2_beta] = cubic_spline(beta1, betavia, beta2, T1, T2);
    [a1_gamma, b1_gamma, c1_gamma, d1_gamma, a2_gamma, b2_gamma, c2_gamma, d2_gamma] = cubic_spline(gamma1, gammavia, gamma2, T1, T2);

    % Time vector for plotting
    T = T1 + T2;
    t = linspace(0, T, 1000);

    % Compute the Euler angles trajectories
    alpha = trajectory(t, T1, T2, a1_alpha, b1_alpha, c1_alpha, d1_alpha, a2_alpha, b2_alpha, c2_alpha, d2_alpha);
    beta = trajectory(t, T1, T2, a1_beta, b1_beta, c1_beta, d1_beta, a2_beta, b2_beta, c2_beta, d2_beta);
    gamma = trajectory(t, T1, T2, a1_gamma, b1_gamma, c1_gamma, d1_gamma, a2_gamma, b2_gamma, c2_gamma, d2_gamma);

    % Plot the results
    figure;
    subplot(3,1,1);
    plot(t, alpha, 'b', 'LineWidth', 1.5);
    title('\alpha(t)');
    xlabel('Time [s]');
    ylabel('\alpha [rad]');

    subplot(3,1,2);
    plot(t, beta, 'r', 'LineWidth', 1.5);
    title('\beta(t)');
    xlabel('Time [s]');
    ylabel('\beta [rad]');

    subplot(3,1,3);
    plot(t, gamma, 'g', 'LineWidth', 1.5);
    title('\gamma(t)');
    xlabel('Time [s]');
    ylabel('\gamma [rad]');

    % Check for singularities
    if any(abs(beta) >= pi/2)
        disp('Singularity encountered in the planned motion.');
    else
        disp('No singularities encountered in the planned motion.');
    end
end

function [a1, b1, c1, d1, a2, b2, c2, d2] = cubic_spline(theta1, thetavia, theta2, T1, T2)
    % First interval coefficients
    A1 = [1, 0, 0, 0;
          1, T1, T1^2, T1^3;
          0, 1, 0, 0;
          0, 1, 2*T1, 3*T1^2];
    b1 = [theta1; thetavia; 0; 0];
    coeffs1 = A1\b1;
    d1 = coeffs1(1); c1 = coeffs1(2); b1 = coeffs1(3); a1 = coeffs1(4);
    
    % Second interval coefficients
    A2 = [1, 0, 0, 0;
          1, T2, T2^2, T2^3;
          0, 1, 0, 0;
          0, 1, 2*T2, 3*T2^2];
    b2 = [thetavia; theta2; 0; 0];
    coeffs2 = A2\b2;
    d2 = coeffs2(1); c2 = coeffs2(2); b2 = coeffs2(3); a2 = coeffs2(4);
end

function theta = trajectory(t, T1, T2, a1, b1, c1, d1, a2, b2, c2, d2)
    theta = zeros(size(t));
    for i = 1:length(t)
        if t(i) <= T1
            tau = t(i);
            theta(i) = a1 * tau^3 + b1 * tau^2 + c1 * tau + d1;
        else
            tau = t(i) - T1;
            theta(i) = a2 * tau^3 + b2 * tau^2 + c2 * tau + d2;
        end
    end
end