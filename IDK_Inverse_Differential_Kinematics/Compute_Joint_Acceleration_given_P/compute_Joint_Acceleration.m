% NameFile: MAIN_compute_Joint_Acceleration
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - p: E-E Task Position.
%   - q: Set of Joint variables.
% 
%   Find: 
%   - Computes the Joint Acceleration. 

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%
% Define symbolic variables
syms q1 q2 q3 q1_dot q2_dot q3_dot q1_ddot q2_ddot q3_ddot L real

% Define joint variables and their derivatives
q = [q1; q2; q3];
q_dot = [q1_dot; q2_dot; q3_dot];
q_ddot = [q1_ddot; q2_ddot; q3_ddot];

% Define end-effector position.
p = [cos(q1) + cos(q1 + q2) + cos(q1 + q2 + q3);
         sin(q1) + sin(q1 + q2) + sin(q1 + q2 + q3)];

% NUMERICAL DATA
qd = [pi/4; pi/3; -pi/2]; % Desired joint positions [rad]
qd_dot = [-0.8; 1; 0.2]; % Desired joint velocities [rad/s]
pd_ddot = [1; 1]; % Desired end-effector accelerations [m/s^2]
L_num = 1; % Link length

%%%%%% END PARAMETERS TO SET %%%%%%

clc
clear

%%%%%% MAIN %%%%%%

%%% SYMBOLIC %%%

% Compute the Jacobian matrix for the position.
Jp = simplify(jacobian(p, q));
p_dot = Jp*q_dot;

% Display the Jacobian and its derivative.
disp('Jacobian for the Position, Jp, is:');
disp(Jp);

% Extended Jacobian matrix to include angular velocity.
J = [Jp; 1 1 1];

% Compute the time derivative of the Jacobian matrix.
Jdot_p = simplify(diff(Jp, q1) * q1_dot + diff(Jp, q2) * q2_dot + diff(Jp, q3) * q3_dot);
disp('Time derivative of the Jacobian Jdot_p:');
disp(Jdot_p);

% Compute n(q, q_dot) = Jdot_p*q_dot.
n_q_qdot = simplify(Jdot_p*q_dot); 
disp("The n(q, q_dot) result is: ");
disp(n_q_qdot);

% Compute p_ddot = Jp*q_ddot + n_q_qdot.
p_ddot = simplify(Jp*q_ddot + n_q_qdot);
disp("The p_ddot result is: ");
disp(p_ddot);

% Singularity analysis
disp("The extended Jacobian J is: ");
disp(J);
det_J = simplify(det(J));
disp('Determinant of the Jacobian det(J):');
disp(det_J);

% Compute the Joint Acceleration.
q_ddot_result = simplify(inv(J)*[p_ddot-n_q_qdot;0]);
disp("The Joint Acceleration is q_ddot_result: ");
disp(q_ddot_result);


%%%%% NUMERIRICAL %%%%%

% Sobstitute numericla Values for pd.
pd = double(subs(p, q, qd));
disp("pd is: ");
disp(pd);

% Sobstitute numerical Values for pd.
pdot_d = double(subs(p_dot, [q, q_dot], [qd, qd_dot]));
disp("pdot_d is: ");
disp(pdot_d);

% Sobstitute numerical values in the Jacobian.
Jp = double(subs(Jp, q, qd));
J = [Jp; 1 1 1];
disp(J);

% Sobstitute values for n(q, q_dot).
n_q_qdot = double(subs(n_q_qdot, [q, q_dot], [qd, qd_dot]));
disp("n_q_qdot is: ");
disp(n_q_qdot);

% Compute the commanded joint accelerations
qd_ddot = inv(J) * ([pd_ddot; 0] - [n_q_qdot;0]);

disp('qd_ddot is:');
disp(qd_ddot);


%%% KINEMATIC CONTROL %%%%
% Implement feedback control for error correction
% Define gains
% Kp = eye(2); % Proportional gain matrix
% Kd = eye(2); % Derivative gain matrix
% k_omega = 1; % Scalar gain for angular velocity
% 
% % Define the errors (example values)
% ep = [0; 0]; % Position error
% ev = [0; 0]; % Velocity error
% e_omega = [0, 0]; % Angular velocity error
% 
% % Modify the commanded acceleration with feedback control
% qd_ddot_feedback = inv(J) * [pd_ddot + Kd * ev + Kp * ep - n_q_qdot; k_omega * e_omega];
% 
% % Display the results with feedback control
% disp('Commanded joint accelerations with feedback q_ddot_feedback:');
% disp(qd_ddot_feedback);

