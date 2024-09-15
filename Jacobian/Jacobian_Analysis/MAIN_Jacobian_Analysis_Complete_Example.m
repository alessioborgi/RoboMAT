% NameFile: Main_DH
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - alpha_vector: This is a vector of alphas, representing the twist angle 
%               between joint axes, obtained by projecting on a plane O 
%               orthogonal to the link axis. (ALWAYS CONSTANT)
%   - a_vector: This is a vector of a's, representing the displacement AB 
%               between the joint axes. (ALWAYS CONSTANT)
%   - d_vector: This is a vector of d's, representing displacement CD, i.e.,
%               the displacement between axis of joint i-1 and axis of joint i.
%               This is:
%               - VARIABLE: If the joint i is PRISMATIC.
%               - CONSTANT: If the joint i is REVOLUT.
%   - theta_vector: This is a vector of thetas, representing the angle between 
%               link axes (i-1, and i), obtained by projecting these two axes on a
%               orthogonal plane. 
%               This is:
%               - VARIABLE: If the joint i is REVOLUT.
%               - CONSTANT: If the joint i is PRISMATIC.

%   Find: 
%   - T: This is the product of all the matrices corresponding to each
%        vector of arrays.

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%
syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 l1 l2 l3 l4 l5 l6 real 

% Example: 2R Planar Robot
% syms q1 q2 real
% syms l1 l2 real 
% alpha_vector = [0, 0];
% a_vector = [l1, l2];
% d_vector = [0, 0];
% theta_vector = [q1, q2];

% Example: SCARA Robot
% syms a1 a2 real
% syms d1 d4 real
% syms q1 q2 q3 q4 real 
% alpha_vector = [0, 0, 0, pi];
% a_vector = [a1, a2, 0, 0];
% d_vector = [d1, 0, q3, d4];
% theta_vector = [q1, q2, 0, q4];

% Example: Stanford Manipulator
% syms alpha1 alpha2 alpha4 alpha5 real
% syms d1 d2 q3 d6 real
% syms q1 q2 q4 q5 real 
% alpha_vector = [alpha1, alpha2, 0, alpha4, alpha5, 0];
% a_vector = [0, 0, 0, 0, 0, 0];
% d_vector = [d1, d2, q3, 0, 0, d6];
% theta_vector = [q1, q2, -pi/2, q4, q5, 0];

% Midterm 18-11-2022
% syms d1 q1 q2 q3 a3 real
% alpha_vector = [pi/2, pi/2, 0];clc
% a_vector = [0, 0, a3];
% d_vector = [d1, q2, 0];
% theta_vector = [q1, pi/2, q3];

% Midterm 19-11-2021
% syms q1 q2 q3 q4 N real
% alpha_vector = [0, -pi/2, 0, 0];
% a_vector = [0, N, 0, 0];
% d_vector = [q1, 0, q3, 0];
% theta_vector = [0, q2, 0, q4];

% Exam 24-01-2024
% syms q1 q2 q3 q4 q5 q6 a2 a3 d1 d4 d6 real
% alpha_vector = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
% a_vector = [40, 445, 40, 0, 0, 0];
% d_vector = [330, 0, 0, 440, 0, 80];
% theta_vector = [0, 0, 0, 0, 0, 0];

% Medical Robotics: 2nd Exercise
% syms q1 q2 q3 l1 l2 l3 real
% alpha_vector = [0, 0, 0];
% a_vector = [l1, l2, l3];
% d_vector = [0, 0, 0];
% theta_vector = [q1, q2, q3];

% Exam 21-10-2022
% alpha_vector = [pi/2, pi/2, 0];
% a_vector = [0, 0, a3];
% d_vector = [d1, q2, 0];
% theta_vector = [q1, pi/2, q3];

% Exam 24-01-2024
% alpha_vector = [pi/2, 0 pi/2];
% a_vector = [a1, a2, a3];
% d_vector = [d1, 0, 0];
% theta_vector = [q1, q2, q3];
% alpha_vector = [pi/2, 0 pi/2];
% a_vector = [0.04, 0.445, 0.04];
% d_vector = [0.33, 0, 0];
% theta_vector = [q1, q2, q3];

% Exam 16-02-2024
alpha_vector = [0, pi/2, 0];
a_vector = [0, 0, 0];
d_vector = [0, q2, q3];
theta_vector = [q1, pi/2, 0];
%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%

%%% Denavit-Hartenberg Direct Kinematics %%%
clc
disp("The DK(Direct Kinematics) following from the DH Process is: ")
arrays = [alpha_vector; a_vector; d_vector; theta_vector];
[T, A] = DHMatrix(arrays);

% Printing out the Transformation Matrix.
disp('T: ');
disp(T);

% Printing out the xn axis.
n=T(1:3,1);
disp("n");
disp(n);

% Printing out the yn axis.
s=T(1:3,2);
disp("s");
disp(s);

% Printing out the zn axis.
a=T(1:3,3);
disp("a");
disp(a);

% Printing out the position axis.
p = T(1:3,4);
disp("p");
disp(p);





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% JACOBIAN ANALYSIS %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% ANALYTICAL JACOBIAN %%% 
% Printing the Analytical Jacobian of the Position.
jac_point= jacobian(p, [q1, q2, q3]);
disp("Analtical Jacobian of Task Position");
disp(jac_point);

%%% DETERMINANT %%% 
% Printing the Determinant
disp("Determinant(J) is: ");
det_J = simplify(det(simplify(jac_point)));
disp(det_J);

%%% SINGULARITIES' SOBSTITUTIONS %%%
% Make Sobstitutions for the Singularities.
subs1 = subs(jac_point, q3, 0);
subs2 = subs(jac_point, q2, -a3*sin(q3));
subs3 = subs(subs2, q3, 0); % Intersection of the two singularities.

disp("Jacobian after first Substitution");
disp(subs1);
disp("Jacobian after second Substitution");
disp(subs2);
disp("Jacobian after both 1st and 3rd Substitutions");
disp(subs3);

%%% JACOBIAN RANK %%%
% Determine the Rank of the new Jacobian with sobstituted value.
rank_j_original = rank(jac_point);
rank_j_subs1 = rank(subs1);
rank_j_subs2 = rank(subs2);
rank_j_subs3 = rank(subs3);

disp("Rank of the Original Jacobian");
disp(rank_j_original);
disp("Rank of the first substituted Jacobian");
disp(rank_j_subs1);
disp("Rank of the second substituted Jacobian");
disp(rank_j_subs2);
disp("Rank of the both substituted Jacobian");
disp(rank_j_subs3);

%%% JACOBIAN NULLSPACE %%%
% Compute the null space and if it is != 0, find a Basis for the NullSpace.
nullspace_J_subs1 = null(subs1);
nullspace_J_subs2 = null(subs2);
nullspace_J_subs3 = null(subs3);

disp('Nullspace of the Sobstituted Jacobian 1:');
disp(nullspace_J_subs1);
disp('Nullspace of the Sobstituted Jacobian 2:');
disp(nullspace_J_subs2);
disp('Nullspace of the Sobstituted Jacobian 3:');
disp('Warning! Read it by Columns')
disp(nullspace_J_subs3);

%%% JACOBIAN RANGE SPACE %%%
% Compute the column space (range space) of the matrix.
range_J1 = orth(jac_point);

% Display the basis for the range space
disp('Range of the Sobstituted Jacobian 1');
R=simplify(range_J1);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

% Compute the column space (range space) of the matrix
range_J2 = orth(subs2);
% Display the basis for the Range Space
disp('Range of the Sobstituted Jacobian 2');
R=simplify(range_J2);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

% Compute the Range of the Jacobian.
range_J3 = orth(subs3);
disp('Range of the Sobstituted Jacobian 3');
R=simplify(range_J3);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end


%%% JACOBIAN COMPLEMENT RANGE SPACE %%%
% Cartesian direction(s) where instantaneous mobility of P is lost. %
% Compute the Complement of the Range Space of the Jacobian.
range_J_subs1_compl = null(subs1.');
range_J_subs2_compl = null(subs2.');
range_J_subs3_compl = null(subs3.');

disp('Complementary Range Space of the Sobstituted Jacobian 1:');
disp(range_J_subs1_compl);
disp('Complementary Range Space of the Sobstituted Jacobian 2:');
disp(range_J_subs2_compl);
disp('Complementary Range Space of the Sobstituted Jacobian 3:');
disp('Warning! Read it by Columns')
disp(range_J_subs3_compl);



%%% JACOBIAN COMPLEMENT NULL SPACE %%%
% Compute the Complement of the Null Space of the Jacobian.

% Sobstituted Jacobian 1.
null_J_subs1_compl = colspace(subs1.');
disp('Complementary Null Space of the Sobstituted Jacobian 1:');
R=simplify(null_J_subs1_compl);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

% Sobstituted Jacobian 2.
null_J_subs2_compl = colspace(subs2.');
disp('Complementary Null Space of the Sobstituted Jacobian 2:');
R=simplify(null_J_subs2_compl);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

% Sobstituted Jacobian 3.
null_J_subs3_compl = colspace(subs3.');
disp('Complementary Null Space of the Sobstituted Jacobian 3:');
R=simplify(null_J_subs3_compl);
[n,m]=size(R);
for i=1:m
    [numerator, denominator] = numden(R(:,i));
    fprintf('%d basis',i)
    display(simplify(numerator,'Steps',50));
end

%%%%%% END PROGRAM %%%%%%




