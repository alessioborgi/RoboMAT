% NameFile: Compute_Rotation_Matrix
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
%   - last_index: Index of the Rotation Matrix, over which we want to compute the
%        Analysis. Put 0 if you don't want to use the Rotated option.
% 
% 
%   Find: 
%   - The 0_R_i Rotation Matrix.

%%%%%% END TASK %%%%%%


function [R] = Compute_Rotation_Matrix(DHTABLE, last_index)
    
    % Define four symbolic variables.
    syms alpha_ d a theta_
    
    % Take the number of joints.
    N = size(DHTABLE, 1);


    % Transformation matrix for DH parameters.
    TDH = [cos(theta_) -sin(theta_)*cos(alpha_) sin(theta_)*sin(alpha_) a*cos(theta_);
           sin(theta_) cos(theta_)*cos(alpha_) -cos(theta_)*sin(alpha_) a*sin(theta_);
           0 sin(alpha_) cos(alpha_) d;
           0 0 0 1];

    A = cell(1, N);

    % Compute individual transformation matrices.
    for i = 1:N
        A{i} = subs(TDH, [alpha_, a, d, theta_], DHTABLE(i, :));
    end

    T = eye(4);
    rotation_0_to_i = cell(1, N);

    % Compute rotation matrices from base to each joint.
    for i = 1:N
        T = T * A{i};
        rotation_0_to_i{i} = simplify(T);
    end

    % Initialize the Jacobian matrix as symbolic.
    Ja = sym(zeros(3, N));

    % Unit vector along the z-axis.
    z = [0; 0; 1];
    Ja(:, 1) = z;
    
    % Compute the column vectors for the Jacobian matrix.
    for i = 1:last_index
        R = rotation_0_to_i{i}(1:3, 1:3);
        zi = R * z;
        Ja(:, i+1) = zi;
    end
end
