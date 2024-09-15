% NameFile: DH_to_Rotation
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
%   - rot_to_idx: Index of the frame to which we want to rotate. (e.g., 3).
%                 (Notice that this must be 0 < i <= number of rows of the DH Table )
% 
%   Find: 
%   - The Rotation Matrix from the base up to the i-th frame.

%%%%%% END TASK %%%%%%


function [R]= DH_to_Rotation(DHTABLE,rot_to_idx)
    

    syms alpha_ d a theta_

    % Number of rows in DHTABLE
    N = size(DHTABLE, 1);

    % DH Transformation Matrix
    TDH = [cos(theta_) -sin(theta_)*cos(alpha_) sin(theta_)*sin(alpha_) a*cos(theta_);
           sin(theta_) cos(theta_)*cos(alpha_) -cos(theta_)*sin(alpha_) a*sin(theta_);
           0 sin(alpha_) cos(alpha_) d;
           0 0 0 1];

    % Initialize cell array to store transformation matrices for each joint
    A = cell(1, N);

    % Compute transformation matrices for each joint
    for i = 1:N
        % Substitute DH parameters into TDH
        A{i} = subs(TDH, [alpha_, a, d, theta_], DHTABLE(i, :));
    end

    % Initialize transformation matrix T (identity matrix)
    T = eye(4);
    % Initialize cell array to store rotation matrices from base to each joint
    rotation_0_to_i = cell(1, N);

    % Compute rotation matrices from base to each joint
    for i = 1:N
        % Update cumulative transformation matrix
        T = T * A{i};
        % Store rotation matrix from base to current joint
        rotation_0_to_i{i} = simplify(T);
    end

    % Compute the desired rotation
    fprintf("rotation from 0 to %d \n", rot_to_idx)
    % Select rotation matrix from base to specified frame and extract only rotation part
    R = rotation_0_to_i{rot_to_idx}(1:3, 1:3);
    disp(R);
end
