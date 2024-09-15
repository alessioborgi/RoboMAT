% NameFile: MAIN_DH_to_Geometric_Jacobian
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
%   - prismatic_indices: A list of indices of Prismatic Joints (e.g., [1, 4, 5], corresponding to 
%                        the fact that joints 1,4 and 5 are prismatic. Look where the qs is in the 
%                        third column (the one referring to d).
%                        (Remember to start index from 1)). 
%                        If None, put [].
%   - i: Index of the Rotation Matrix, over which we want to compute the
%        Analysis. Put 0 if you don't want to use the Rotated option.
% 
%   Find: 
%   - The Linear Jacobian for a Robot.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%

syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 l1 l2 l3 l4 l5 l6 real

% Exam 12-06-2023
% alpha_vector = [0, pi/2, 0, 0];
% a_vector = [a1, 0, a3, a4];
% d_vector = [d1, d2, 0, 0];
% theta_vector = [q1, q2, q3, q4];
% variables = [q1, q2, q3, q4];
% prismatic_indices = [];

% Exam 13-02-2023
alpha_vector = [pi/2, pi/2, -pi/2, 0];
a_vector = [0, 0, 0, a4];
d_vector = [0, 0, q3, 0];
theta_vector = [q1, q2, 0, q4];
variables = [q1, q2, q3, q4];
prismatic_indices = [3];


choice = 2;     % Put 1(with Normal Jacobian) or 
%                     2(with Rotated Jacobian) or 
%                     3(with Minors Analysis)

% Index of Jacobian Rotation Parameter. (Set only if choice == 2) 
i = 1;  % Put 0 if you don't want to use the Rotated option.


%%%%%% END PARAMETERS TO SET %%%%%%
%%%%%% MAIN %%%%%

clc

% Set the DH Table with columns being: alpha, a, d, theta.
DHTABLE = [alpha_vector; a_vector; d_vector; theta_vector].';

% Compute the Linear, Angular and whole Geometric Matrix.
result_linear = DH_to_Linear_Jacobian(DHTABLE, variables);
result_angular = DH_to_Angular_Jacobian(DHTABLE, prismatic_indices);
result_final = [result_linear;result_angular];

%%%%% 1) SIMPLE: GEOMETRIC JACOBIAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if choice == 1
    disp("The Final (Simple) Geometric Jacobian will be: ");
    disp(result_final);
    
    % Compute the Determinant.
    [m, n] = size(result_final);
    disp(simplify(result_final));
    det_J = Compute_Jacobian_Determinant(result_final);
    disp("The Determinant of the (Simple) Geometric Jacobian is given by: ");
    disp(simplify(det_J));


%%%%% 2) MIDDLE: ROTATED JACOBIAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif choice==2 && i~=0 
    % Compute the Rotation Matrix up until index i of the Transformation Matrix.
    disp("Computing the Rotation Matrix: ");
    fprintf("The Rotation Matrix 0R%d is: \n", i);
    R = Compute_Rotation_Matrix(DHTABLE, i);
    disp(R);
    
    % Create a 6x6 matrix with the desired structure
    zeros_R = zeros(3);
    extended_R = [R.', zeros_R; zeros_R, R.'];
    disp("The extended R is: ");
    disp(extended_R);
    
    % Multiply the created structure times the Geometric Jacobian.
    rotated_Geometric_Jac = simplify(extended_R*result_final);
    fprintf("The Rotated Geometric Jacobian %dJ(q) is equal to: \n", i);
    result_final = rotated_Geometric_Jac;
    
    % Find the Determinant of the Rotated Jacobian i_J(q).
    [m, n] = size(result_final);
    disp(simplify(result_final));
    det_J = Compute_Jacobian_Determinant(result_final);
    disp("The Determinant of the Rotated Geometric Jacobian is given by: ");
    disp(simplify(det_J));



%%%%% 3) DIFFICULT: MINORS ANALYSIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif  choice==3
    disp("Starting the Minors Analysis... ");
    solutions = Find_Singularity_Minors_Analysis(result_final, variables);
    disp("In order to find the Singulariteis, pose every subDeterminant == 0");
    disp(solutions);
    singularities = solve(solutions, [q2, q3], 'Real', true);
    disp("The Singularities we found with the Minors Analysis are: ");
    disp(singularities);    
   
else 
    disp("You typed a Wrong Choice. Choice must be either 1, 2 or 3.");

end

%%%%%% END MAIN %%%%%

%%%%%%%%%% SINGULARITIES' SOBSTITUTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make Sobstitutions for the Singularities.
subs1 = subs(result_final, q3, 0);
disp("Jacobian after first Substitution");
disp(subs1);


%%%%%%%%%% JACOBIAN RANK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determine the Rank of the new Jacobian with sobstituted value.
rank_j_original = rank(result_final);
rank_subs1 = rank(subs1);

disp("Rank of the Jacobian");
disp(rank_j_original);

disp("Rank of the Sobstituted Jacobian");
disp(rank_subs1);

%%%%%%%%%% JACOBIAN NULLSPACE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MEANING: The set of all the vector that multiplied by the Jacobian 
%          produce zero. 
%      Ex. N(J) = [1   0   0]^T means that, when applying a velocity to a 
%          joint, this will not move the E-E.

% Compute the null space and if it is != 0, find a Basis for the NullSpace.
nullspace_J_original = null(result_final);
nullspace_J_subs1 = null(subs1);

disp('Nullspace of the Jacobian:');
disp(nullspace_J_original);
disp('Nullspace of the Sobstituted Jacobian:');
disp(nullspace_J_subs1);

%%%%%%%%%% JACOBIAN TRANSPOSE NULLSPACE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MEANING: When applying in the given direction a Cartesian force to the E-E, 
%          no robot motion will result and this does not need any active 
%          counteracting joint torque by the motors; such a Cartesian force 
%          is statically balanced by the internal reaction forces of the rigid 
%          structure.

% Compute the null space and if it is != 0, find a Basis for the NullSpace.
nullspace_J_T = null(result_final.');
nullspace_subs1_T = null(subs1.');

disp('Nullspace of the Jacobian Transposed:');
disp(nullspace_J_T);
disp('Nullspace of the subs1 Transposed:');
disp(nullspace_subs1_T);



%%%%%%%%%% JACOBIAN RANGE SPACE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MEANING: The set of basis vector of the Jacobian matrix. 

% Compute the column space (range space) of the Jacobian.
range_J1 = orth(result_final);

% Display the basis for the range space
disp('Range of the Jacobian');
R=simplify(range_J1);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

% Compute the column space (range space) of the Sobstituted Jacobian.
range_J1 = orth(subs1);

% Display the basis for the range space
disp('Range of the Sobstituted Jacobian');
R=simplify(range_J1);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end




%%%%%%%%%% JACOBIAN COMPLEMENT RANGE SPACE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MEANING: The set of all the vector that multiplied by the transpose of the
%          Jacobian produce zero. Do it if it asks for the space of Lost Mobility.

% Cartesian direction(s) where instantaneous mobility of P is lost. %
% Compute the Complement of the Range Space of the Jacobian.
range_J_compl = null(result_final.');
range_subs1_compl = null(subs1.');

disp('Complementary Range Space of the Jacobian:');
disp(range_J_compl);
disp('Complementary Range Space of the Sobstituted Jacobian:');
disp(range_subs1_compl);


%%%%%%%%%% JACOBIAN COMPLEMENT NULL SPACE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the Complement of the Null Space of the Jacobian.

null_J_compl = colspace(result_final.');
disp('Complementary Null Space of the Jacobian:');
R=simplify(null_J_compl);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

% Complement Null Space of the Sobstituted Jacobian 
null_J_compl = colspace(subs1.');
disp('Complementary Null Space of the SobstitutedJacobian:');
R=simplify(null_J_compl);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

%%%%%% END PROGRAM %%%%%%
