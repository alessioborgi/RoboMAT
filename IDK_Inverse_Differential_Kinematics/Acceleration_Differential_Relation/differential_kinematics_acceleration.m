% Define symbolic variables for joint positions (q1, q2) and velocities (dq1, dq2)
syms q1 q2 dq1 dq2 real

% Define joint velocity vector
dq = [dq1; dq2];

% Define the end-effector position
p = [q2*cos(q1); q2*sin(q1)];
disp("Position Vector: ");
disp(p);

% Compute the Jacobian matrix J(q)
J = jacobian(p, [q1, q2]);
disp('Jacobian matrix J:');
disp(J);

% Compute the time derivative of each element in the Jacobian matrix
dJ_dt = sym(zeros(size(J))); % Initialize the time derivative of J
for i = 1:size(J,1)
    for j = 1:size(J,2)
        dJ_dt(i,j) = diff(J(i,j), q1) * dq1 + diff(J(i,j), q2) * dq2;
    end
end
disp('Time derivative of the Jacobian matrix dJ/dt:');
disp(dJ_dt);

q_dot = [dq1;
         dq2];

% Compute the Jacobian Differentiated times the q_dot. 
dJ_dt_q_dot = dJ_dt*q_dot;

% Compute the Joint Acceleration.
q_ddot = simplify(-inv(J)*dJ_dt_q_dot);
disp("The joint acceleration is: ");
disp(q_ddot);