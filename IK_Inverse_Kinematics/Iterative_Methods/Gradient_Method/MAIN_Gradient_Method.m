% NameFile: MAIN_Gradient_Method
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
%   - kmax: Maximum number of iterations.
%   - error: Error Tolerance.
%   - minjoint: Minimum Joint Increment for Convergence.
%   - lr: Learning Rate.

%   Find: 
%   - The Inverse Kinematic(IK) Solution of a Robot using the Gradient Method.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

q0 = [0;0;1];  % Initial guess for joint angles in radians.
r = [q3*cos(q2)*cos(q1);     % Forward kinematics equations.
     q3*cos(q2)*sin(q1);
     d1 + q3*sin(q2)];

pos = [1; 1; 1];                    % Desired end-effector position.
kmax = 15;                          % Maximum number of iterations.
error = 1e-5;                       % Error tolerance.
minjoint = 1e-6;                    % Set minimum joint increment for convergence.
lr = 0.5;                           % Learning rate or step size for updating the joint angles in each iteration
                                    % of the gradient descent algorithm.

%%%%%% MAIN %%%%%
clc
format long
syms q1 q2 q3 q4 q5 q6
syms a1 a2 a3 a4
syms d0 d1 d2 d3 d4 de l1 l2 l3 l4 N L M N d A B C D K dtcp h p L1 L2
syms alpha beta gamma

% Substitute a specific value for the d1 parameter in the Forward Kinematics.
r = subs(r, d1, 0.5);

% Check if the determinant is not 0.
disp('As far as the det is not 0 you are ok !');
jacobian(r, [q1,q2, q3]);
determinant = simplify(det(jacobian(r, [q1,q2, q3])));
disp('The determinant is');
disp(determinant);

% Call for the Gradient Method function.
Gradient_Method(r, q0, pos, lr, error, minjoint, kmax, [q1,q2,q3])
Gradient_Method(r, q0, pos, 1, error, minjoint, kmax, [q1,q2,q3])
Gradient_Method(r, q0, pos, 0.7, error, minjoint, kmax, [q1,q2,q3]);

%%%%%% END MAIN %%%%%
