% NameFile: Gradient_Method
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 22-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - q0: Initial guess for joint angles (degrees).
%   - r:  Forward Kinematics equations.
%   - pos: Desired E-E(End-Effector) Position.
%   - kmax: Maximum number of Iterations.
%   - error: Error Tolerance.
%   - minjoint: Minimum Joint Increment for Convergence.
%   - lr: Learning Rate.

%   Find: 
%   - The Inverse Kinematic(IK) Solution of a Robot using the Gradient Method.

%%%%%% END TASK %%%%%%

function Gradient_Method(r, q0, pos, lr, error, minjointincrem, kmax, param)

    % Display start message.
    disp('STARTING Gradient Method');
    
    % Initialize error vector.
    e = ones(size(pos)); 
    
    % Initialize iteration counter.
    counter = 0;
    
    % Initialize final joint angles with an offset of pi.
    qf = q0 + pi;
    
    % Set initial joint angles.
    qi = q0;
    
    % Initialize matrix to store final joint angles.
    final = q0;
    
    % Initialize matrices to store errors and error norms.
    errors = [];
    errornorms = [];
    
    % Initialize joint error.
    jointerr = 1;
    
    % Iterative Gradient Method loop.
    while norm(e) >= error && jointerr >= minjointincrem && counter < kmax

        % Display current iteration.
        display(counter)
        
        % Compute Jacobian matrix.
        j = jacobian(r, param);
        
        % Substitute current joint angles into Jacobian matrix.
        jac = subs(j, param, qi.');
        
        % Compute error vector.
        err = pos - subs(r, param, qi.');
        
        % Transpose of Jacobian matrix.
        jacT = jac.';
        
        % Update joint angles using gradient method formula.
        qf = double(qi + lr * jacT * err);
        
        % Calculate joint error.
        jointerr = norm(qf - qi);
        
        % Increment iteration counter.
        counter = counter + 1;
        
        % Update current joint angles.
        qi = qf;
        
        % Update error vector.
        e = double(err);
        
        % Store errors and error norms
        errors = [errors err];
        errornorms = [errornorms norm(e)];
        
        % Store current joint angles
        final = [final qf];
    end
    
    % Display information about final joint angles and error norms.
    disp("In the following 'final' and 'errornorms' results, each column will represent q from iter 0 to " + counter + "and each row indicate the Joint");
    final
    errornorms
    final(1:3:end-3)
    
    % Generate iteration indices.
    y_plot = 0:counter-1;
    
    % Plot convergence of error norm.
    subplot(4,1,1)
    plot(y_plot, errornorms, 'LineWidth', 1.5)
    title('Convergence Gradient Method')
    grid on
    xlabel('Iterations')
    ylabel('error norm')
    hold on

    % Plot joint angles over Iterations.
    subplot(4,1,2)
    plot(y_plot, final(1:3:end-3), 'LineWidth', 1.5)
    title('Joint q1')
    grid on
    xlabel('Iterations')
    ylabel('q1')
    hold on
    
    subplot(4,1,3)
    plot(y_plot, final(2:3:end-3), 'LineWidth', 1.5)
    title('Joint q2')
    grid on
    xlabel('Iterations')
    ylabel('q2')
    hold on

    subplot(4,1,4)
    plot(y_plot, final(3:3:end-3), 'LineWidth', 1.5)
    title('Joint q3')
    grid on
    xlabel('Iterations')
    ylabel('q3')
    hold on

end
