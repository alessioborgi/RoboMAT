% NameFile: DH_to_Linear_Jacobian
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - DHTABLE: Denavit-Hartenmberg(DH) matrix written in the column order: alpha, a, d, theta.
%              Ex.
%                   DHTABLE = [pi/2            0            sym('d1')           q1;
%                              pi/2            0                   0            q2;
%                              pi/2            0                  q3             0;
%                                 0     sym('a4')                  0            q4];
%   - variables: Array containing the joint variables (e.g., [q1, q2, .., qn]).
% 
%   Find: 
%   - The Linear Jacobian for a Robot.

%%%%%% END TASK %%%%%%


function [JL] = DH_to_Linear_Jacobian(DHTABLE, variables)
    % Function that finds the Linear Jacobian for a Robot.
    
    % Define four symbolic variables.
    syms alpha_ d a theta_
    
    % Take the number of joints.
    N = size(DHTABLE, 1);

    % Transformation Matrix for DH parameters.
    TDH = [cos(theta_) -sin(theta_)*cos(alpha_) sin(theta_)*sin(alpha_) a*cos(theta_);
           sin(theta_) cos(theta_)*cos(alpha_) -cos(theta_)*sin(alpha_) a*sin(theta_);
           0 sin(alpha_) cos(alpha_) d;
           0 0 0 1];

    A = cell(1, N);

    % Compute Individual Transformation Matrices.
    for i = 1:N
        A{i} = subs(TDH, [alpha_, a, d, theta_], DHTABLE(i, :));
    end

    T = eye(4);
    position_0_to_i = cell(1, N);

    % Compute position vectors from base to each joint.
    for i = 1:N
        T = T * A{i};
        position_0_to_i{i} = T(1:3,4);
    end

    % Position vector from base to End-Effector. (Take the last column)
    p_0_e = position_0_to_i{N}; 
    
    % Compute the Jacobian in one shot.
    JL = simplify(jacobian(p_0_e, variables));

    % Display or return the Jacobian matrix as needed.
    disp('The Linear Jacobian Matrix is: ');
    disp(JL);
end
