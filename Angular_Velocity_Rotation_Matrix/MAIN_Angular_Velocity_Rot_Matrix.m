% NameFile: MAIN_Angular_Velocity_Rot_Matrix
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - sequence_rotation: A string describing the three rotations over fixed 
%                        axes we are interested in.
%   - phi_value: The angle of the first rotation. It can be either symbolic
%                or normal numbers.
%   - theta_value: The angle of the second rotation. It can be either symbolic
%                  or normal numbers.
%   - psi_value: The angle of the third rotation. It can be either symbolic
%                or normal numbers.
%   Find: 
%   - R: The Rotation Matrix and the mapping with the Angular Velocity.

%%%%%% END TASK %%%%%%

addpath ./..//RPY_Roll_Pitch_Yaw/'RPY_Fixed_Axes_Angles_Direct_Problem_R(phi,theta,psi)'/

%%%%%% PARAMETERS TO SET %%%%%%

% Exam 08-07-2022 N1
% Set the sequence of rotation you want. 
sequence_rotation = "xzy";

% SYMBOLIC:
syms alpha beta gamma real
syms alpha_dot beta_dot gamma_dot real

sequence_angles = [alpha, beta, gamma];

%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%

clc

% Compute the Rotation Matrix.
R = simplify(RPY_Rotation(sequence_rotation, sequence_angles));
disp(" ");
disp("The Rotation Matrix R in RPY Representation is: ");
disp(R);

% Compute the derivative of the Rotation Matrix.
% Compute the time derivative of R
R_dot = simplify(diff(R, alpha) * alpha_dot + diff(R, beta) * beta_dot + diff(R, gamma) * gamma_dot);
disp("R_dot = ");
disp(R_dot);

R_T = R.';
disp("R_T = ");
disp(R_T);

% Angular velocity skew-symmetric matrix S(omega)
S_omega = simplify(R_dot * R_T);
disp("S(ω) = ");
disp(S_omega);

% Extract the components of the angular velocity.
omega_x = S_omega(3, 2);
omega_y = S_omega(1, 3);
omega_z = S_omega(2, 1);

% Express omega as a vector.
omega = [omega_x; omega_y; omega_z];
disp("The components of the Omega Vector are: ");
disp(omega);

% Define the vector of time derivatives of the angles.
phi_dot = [alpha_dot; beta_dot; gamma_dot];

% Solve for the linear mapping T(phi).
T = simplify(jacobian(omega, phi_dot));

% Display the result.
disp('The matrix T(phi) is:');
disp(T);

% Determine the determinant of T(phi) to find singularities.
det_T = simplify(det(T));

disp('The determinant of T(phi) is:');
disp(det_T);

% Identify singularities
singularities = solve(det_T == 0, beta);
disp('Singularities occur at:');
disp(singularities);

%%% WORK IN THE SINGULAR MATRIX T %%% 
% (MAY NEED TO CHANGE IF MORE SINGULARITIES ARE PRESENT)

% Substitute the Singularity in T(phi).
T_singular = subs(T, [beta], [singularities]);
disp("The T(phi) matrix after the Singular Sobstitution is equal to: ")
disp(T_singular);

% Identify Rank of this matrix.
T_singular_rank = rank(T_singular);
disp("The Rank of the Singular Matrix is: ");
disp(T_singular_rank);

% Identify Angular Velocity omega form.
disp("%%%%%");
disp("The set of all Feasible Angular Velocities");
T_singular_range = simplify(orth(T_singular));
if isempty(T_singular_range)
    disp('Range of T_singular: None (The range space is empty)');
else
    disp('Range of T_singular (columns represent basis):');
    for j = 1:size(T_singular_range, 2)
        disp(['Basis vector ', num2str(j), ':']);
        disp(simplify(T_singular_range(:, j)));
    end
end

%%% NOW YOU NEED TO IDENTIFY ANGULAR VELOCITIES NOT IN THE RANGE !!!
% Look at above and change one number, for instance.

% Identify all non-trivial phi_dot s.t. omega=0 (Nullspace). 
disp("%%%%%");
disp("The set of all phi_dot=alpha_dot, beta_dot,psi_dot) velocity vectors s.t. angular velocity omega=0");
T_singular_nullspace = simplify(null(T_singular));
if isempty(T_singular_nullspace)
    disp('Null Space of T_singular: None (The null space is empty)');
else
    disp('Null Space of T_singular (columns represent basis):');
    for j = 1:size(T_singular_nullspace, 2)
        disp(['Basis vector ', num2str(j), ':']);
        disp(T_singular_nullspace(:, j));
    end
end

%%%%%% END PROGRAM %%%%%%
