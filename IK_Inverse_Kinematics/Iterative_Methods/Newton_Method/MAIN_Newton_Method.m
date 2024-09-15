% NameFile: MAIN_Newton_Method
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
%   - variables: This contains the joint variables.

%   Find: 
%   - The Inverse Kinematic(IK) Solution of a Robot using the Newton Method (iterative, not closed form).

%%%%%% END TASK %%%%%%

format long                         % Set format to display numbers in long format.
syms q1 q2 q3 q4 q5 q6 real         % Symbolic variables for joint angles.
syms a1 a2 a3 a4 real               % Symbolic variables for other parameters.
syms d0 d1 d2 d3 d4 de L l1 l2 l3 l4 N L M N d A B C D K dtcp h p L1 L2 real% More symbolic variables.
syms alpha beta gamma real          % Symbolic variables for angles.


%%%%%% PARAMETERS TO SET %%%%%%

% Exam 13-02-2023
% q0 = [deg2rad(-70); deg2rad(100)];  % Initial guess for joint angles in radians.
q0 = [deg2rad(20); deg2rad(-120)];
r = [l1*cos(q1) + l2*cos(q1+q2);      % Forward kinematics equations.
     l1*sin(q1) + l2*sin(q1+q2)];

Link_values = [0.5, 0.4];           % Values of the Links, in order. 
pos = [0.4; -0.3];                  % Desired end-effector position.
kmax = 3;                           % Maximum number of iterations.
error = 1e-4;                       % Error tolerance.
variables = [q1,q2];                % Variables involved in the analysis.


% Exam 08-06-2022 N3
% q0 = [-pi/4; pi/4; pi/4];  % Initial guess for joint angles in radians.
% r = [L*cos(q1) + N*cos(q1+q2)*cos(q3);      % Forward kinematics equations.
%      L*sin(q1) + N*sin(q1+q2)*cos(q3);
%      M + N*sin(q3)];
% 
% Link_values = [0.5, 0.5, 0.5];      % Values of the Links, in order. 
% pos = [0.3; -0.3; 0.7];             % Desired end-effector position.
% kmax = 10;                          % Maximum number of iterations.
% error = 1e-3;                       % Error tolerance.
% variables = [q1,q2, q3];            % Variables involved in the analysis.

%%%%%% END PARAMETERS TO SET %%%%%%

%%%%%% MAIN %%%%%

% Clear command window.
clc                    

% Check if the determinant is not 0.
disp('As far as the det is not 0 you are ok !');
jac_r = jacobian(r, variables);
disp("The Jacobian of the Direct Kinematics Task r is equal to: ");
disp(jac_r);
determinant= simplify(det(jac_r));
disp('The determinant is');
disp(determinant)

% Substitute parameter values.
% Exam 08-06-2022 N3
% r = subs(r, [L,M,N], Link_values);

% Exam 13-02-2023
r = subs(r, [l1, l2], Link_values); 

% Check if the determinant is not 0.

jac_r = jacobian(r,variables);
determinant= simplify(det(jac_r));
disp('The Determinant, after the sobstitutions is equal to: ');
disp(determinant)

% Call for the Newton Method function.
Newton_Method(r, q0, pos, error, kmax, variables) 

%%%%%% END MAIN %%%%%

