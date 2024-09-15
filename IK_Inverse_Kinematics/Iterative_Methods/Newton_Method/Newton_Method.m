% NameFile: Newton_Method
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - q0: Initial guess for joint angles (degrees).
%   - r:  Forward Kinematics equations.
%   - pos: Desired E-E(End-Effector) Position.
%   - kmax: Maximum number of iterations.
%   - error: Error Tolerance.

%   Find: 
%   - The Inverse Kinematic(IK) Solution of a Robot using the Newton Method (iterative, not closed form).

%%%%%% END TASK %%%%%%

function Newton_Method(r, q0, pos, error, kmax, param)

    % Display start message.
    disp('STARTING Newton Method')
    
    % Initialize error vector.
    e = ones(size(pos)); 
    
    % Initialize iteration counter.
    counter = 0;
    
    % Set initial joint angles.
    qi = q0;
    
    % Initialize final joint angles matrix.
    final = q0;
    
    % Initialize error norms vector.
    errornorms = [];
    
    % Iterative Newton Method loop. Loop up until the norm of the error is
    % greater than the Error Tolerance, and the counter is smaller than the
    % maximum number of iterations.
    while norm(e) >= error && counter <= kmax

        % Display current iteration.
        disp("############################################################");
        disp(["We are at iteration: ", counter]);
        
        % Compute Jacobian matrix.
        j = jacobian(r, param);
        disp("The Jacobian results as: ");
        disp(simplify(j));
        
        % Substitute joint angles into Jacobian matrix.
        jac = subs(j, param, qi.');
        disp("The Sobstituted Jacobian results as: ");
        disp(simplify(jac));
        % Compute error vector.
        err = pos - subs(r, param, qi.');
        disp("The Error Vector is: ");
        disp(simplify(err));
        
        % Get size of Jacobian matrix.
        n = size(jac);

        % Check if Jacobian matrix is NOT squared.
        if ~isequal(n(1), n(2))

            disp('We are in a Redundant Case (n!=m) --> Use the Pseudo-Inverse!')
            % Compute pseudoinverse of Jacobian matrix.
            invjac = pinv(jac);
            disp("The Inverse of the Jacobian is: ")
            disp(simplify(invjac));
        else
            disp('We are in a Regular Case (n!=m) --> Use the Basic Inverse!')
            % Compute inverse of Jacobian matrix.
            invjac = inv(jac);
            disp("The Inverse of the Jacobian is: ")
            disp(simplify(invjac));
        end

        % Update joint angles.
        qf = double(qi + invjac * err);
        disp("The Joint Angles after the Update are equal to: ");
        disp(qf);
        
        % Increment iteration counter.
        counter = counter + 1;
        
        % Update current joint angles.
        qi = qf;
        
        % Update error vector.
        e = double(err);

        % Compute the Norm of the error.
        e_norm = norm(e);
        
        % Compute and store error norm.
        errornorms = [errornorms e_norm];
        disp("The error norms are: ");
        disp(errornorms);
        disp("The Norm of the error at this iteration is: ");
        disp(e_norm);

        % Store current joint angles.
        final = [final qf];
    end
    
    % Display information about final joint angles and error norms.
    disp("In the following 'final' and 'errornorms' results, each column will represent q from iter 0 to " + counter + "and each row indicate the Joint");
    final
    errornorms
    
    % Generate iteration indices.
    y_plot = 0:counter-1;
    
    % Plot convergence of error norm.
    subplot(2, 1, 1)
    plot(y_plot, log10(errornorms), 'LineWidth', 1.5)
    title('Convergence Newton Method')
    grid on
    xlabel('iterations')
    ylabel('log10(error norm)')
    
    % Plot joint angles over iterations.
    subplot(2, 1, 2)
    num_joints = size(q0, 1);
    for i = 1:num_joints
        plot(y_plot, final(i, 1:end-1), 'LineWidth', 1.5)  % Exclude the last column.
        hold on
    end

    title('Joint angles')
    grid on
    xlabel('iterations')
    ylabel('joint angle')
    legend('q1', 'q2') % Update with appropriate joint names.
    
    % Compute final joint angles.
    qf = double(final(:, end));
    
    % Check for Convergence.
    disp("Check if you failed convergence or not with your fixed num of iterations");
    result = norm(double(subs(r, param, double(qf.'))) - pos) <= error;
    if result
        disp("Requisites Satisfied");
    else
        disp("Requisites NOT Satisfied");
    end

end

