% NameFile: Main_Acceleration_Differential_Relation
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%%%%%%%%%%%%%%%%%%%%%%%%%%
%   - variables: Array containing the joint variables (e.g., [q1, q2, .., qn]).
%   - r: Task Kinematics.
%   - L: link-lenght.
%   - q_val: Joints Positions.
%   - q_dot_val: Joints Acceleration.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

clc

syms q1 q2 q3 q1_dot q2_dot q3_dot q1_ddot q2_ddot q3_ddot real
syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 L l1 l2 l3 l4 l5 l6 x y real 

% Exam ....
% Constants
% L = 1; 
% 
% % Task kinematics
% r = simplify([q2*cos(q1) + L*cos(q1 + q3);
%               q2*sin(q1) + L*sin(q1 + q3);
%               q1 + q3]);
% 
% % Numerical Values.
% q_val = [pi/2, 1, 0];
% q_dot_val = [1; -1; -1];
% variables = [q1 q2 q3];

% Task Acceleration.
% q_dot = [q1_dot; q2_dot; q3_dot];
% q_ddot = [q1_ddot; q2_ddot; q3_ddot];

% Exam 24-01-2024
% Constants
% L = 1; 

% Task kinematics
r = simplify([l1*cos(q1) + l2*cos(q1 + q2) + l3*cos(q1 + q2 + q3);
              l1*sin(q1) + l2*sin(q1 + q2) + l3*sin(q1 + q2 + q3);
              q1 + q2 + q3]);

% Numerical Values.
q_val = [pi/2, 1, 0];
q_dot_val = [1; -1; -1];
variables = [q1 q2 q3];

% Task Acceleration.
q_dot = [q1_dot; q2_dot; q3_dot];
q_ddot = [q1_ddot; q2_ddot; q3_ddot];


%%%%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%
% Jacobian.
J = simplify(jacobian(r, variables));

J_dot = sym(zeros(size(J)));
for i = 1:length(q_dot)
    J_dot = J_dot + diff(J, ['q' num2str(i)]) * q_dot(i);
end
J_dot = simplify(J_dot);

h = simplify(J_dot * q_dot);
disp("J_dot is equal to: ");
disp(J_dot);

disp("The h matrix is: ");
disp(h);

r_ddot = simplify(J * q_ddot + h);
fprintf('Task acceleration r_double_dot is: ');
disp(r_ddot);

r_ddot_0 = simplify(J * q_ddot);
fprintf('Task acceleration r_double_dot ( when q_dot = 0, q_ddot not zero) :\n');
disp(r_ddot_0);

% Numerical Evaluation
J_val = double(subs(J, {q1, q2, q3}, q_val));
h_val = double(subs(h, {q1, q2, q3, q1_dot, q2_dot, q3_dot}, [q_val, q_dot_val']));
q_ddot_val = -inv(J_val) * h_val;
fprintf('Numerical example with q = [π/2, 1, 0] and q_dot = [1, -1, -1]:\n');
fprintf('Joint acceleration q_ddot that realizes r_ddot = 0:\n');
disp(q_ddot_val);

% Uniqueness Verification
unique = is_unique(J_val, h_val);
fprintf('Is the joint acceleration unique: %d\n', unique);

% Function to check uniqueness of q_ddot
function unique = is_unique(J_val, h_val)
    q_ddot_val_check = -inv(J_val) * h_val;
    unique = isequal(q_ddot_val_check, -inv(J_val) * h_val);
end