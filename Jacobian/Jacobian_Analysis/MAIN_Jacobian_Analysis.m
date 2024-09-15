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
syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a b a1 a2 a3 a4 a5 a6 L l1 l2 l3 l4 l5 l6 x y real 

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
% alpha_vector = [0, pi/2, 0];
% a_vector = [0, 0, 0];
% d_vector = [0, q2, q3];
% theta_vector = [q1, pi/2, 0];

% Exam 12-06-2023
% alpha_vector = [0, pi/2, 0, 0];
% a_vector = [a1, 0, a3, a4];
% d_vector = [d1, d2, 0, 0];
% theta_vector = [q1, q2, q3, q4];

% Exam 13-02-2023
alpha_vector = [pi/2, pi/2, -pi/2, 0];
a_vector = [0, 0, 0, a4];
d_vector = [0, 0, q3, 0];
theta_vector = [q1, q2, 0, q4];

variables = [q1, q2, q3, q4];
%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%

%%%% OPTION 1: %%%%%
%%%%% Denavit-Hartenberg (DH - DK) Direct Kinematics %%%%%
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





%%%% OPTION 2: %%%%%
%%%%% Directly pass the DK Task Vector %%%%%

clc
% Uncomment if you want to start from an already given DK.

% Exam 09-09-2022 N3
% p = [q2*cos(q1)+L*cos(q1+q3);
%      q2*sin(q1)+L*sin(q1+q3);
%             q1 + q3];
% variables = [q1, q2, q3];

% Exam 23-01-2023 N4
% p = [q2*cos(q1)+q4*cos(q1+q3);
%      q2*sin(q1)+q4*sin(q1+q3);
%             q1 + q3];

% Exam 11-09-2023
p = [a+q1+q3;
     b+q2+q4];
variables = [q1, q2, q3, q4];
disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ");

disp("We are given directly the following Position: ");
disp(p);





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%% JACOBIAN ANALYSIS & MOBILITY ANALYSIS %%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

%%%%%%%%%% ANALYTICAL JACOBIAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Printing the Analytical Jacobian of the Position.
J = simplify(jacobian(p, variables));
disp("%%%%%");
disp("Analtical Jacobian of Task Position");
disp(J);

%%%%%%%%%% DETERMINANT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% Printing the Determinant
disp("%%%%%");
disp("Determinant(J) is: ");
det_J = simplify(Compute_Jacobian_Determinant(simplify(J)));
disp(det_J);

%%%%%%%%%% ORIGINAL JACOBIAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate and display properties of the original Jacobian.
disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ");
disp("%%%%%");
disp('Properties of the Original Jacobian:');

%%% RANK %%%%%
disp("%%%%%");
rank_J = rank(J);
if rank_J == 0
    disp('Rank of the Original Jacobian: 0 (The Jacobian is rank deficient)');
else
    disp(['Rank of the Original Jacobian: ', num2str(rank_J)]);
end

%%%%% NULLSPACE %%%%%
% MEANING: The set of all the vector that multiplied by the Jacobian 
%          produce zero. 
%      Ex. N(J) = [1   0   0]^T means that, when applying a velocity to a 
%          joint, this will not move the E-E.
disp("%%%%%");
disp("The set of all joint velocity vectors s.t. task velocity ṙ=0 when the Robot is in a Regular Configuration");
null_J = simplify(null(J));
if isempty(null_J)
    disp('Null Space of the Original Jacobian: None (The null space is empty)');
else
    disp('Null Space of the Original Jacobian (columns represent basis):');
    for j = 1:size(null_J, 2)
        disp(['Basis vector ', num2str(j), ':']);
        disp(null_J(:, j));
    end
end

%%%%% RANGE SPACE %%%%%
% MEANING: The set of basis vector of the Jacobian matrix. 
% Compute the column space (range space) of the matrix.
disp("%%%%%");
disp("The set of all Feasible Directions for the Velocity in a Regular Configuration");
range_J = simplify(orth(J));
if isempty(range_J)
    disp('Range of the Original Jacobian: None (The range space is empty)');
else
    disp('Range of the Original Jacobian (columns represent basis):');
    for j = 1:size(range_J, 2)
        disp(['Basis vector ', num2str(j), ':']);
        disp(simplify(range_J(:, j)));
    end
end

%%%%% COMPLEMENTARY RANGE SPACE %%%%%
% MEANING: The set of all the vector that multiplied by the transpose of the
%          Jacobian produce zero. Do it if it asks for the space of Lost Mobility.
disp("%%%%%");
disp("The set of all directions along which No Task Velcity can be realized when the Robot is in a Singular Configuration. Space of Lost Mobility");
range_J_compl = simplify(null(J.'));
if isempty(range_J_compl)
    disp('Complementary Range Space of the Original Jacobian: None (The complementary range space is empty)');
else
    disp('Complementary Range Space of the Original Jacobian (columns represent basis):');
    for j = 1:size(range_J_compl, 2)
        disp(['Basis vector ', num2str(j), ':']);
        disp(simplify(range_J_compl(:, j)));
    end
end

%%%%% COMPLEMENTARY NULLSPACE %%%%%
disp("%%%%%");
null_J_compl = simplify(colspace(J.'));
if isempty(null_J_compl)
    disp('Complementary Null Space of the Original Jacobian: None (The complementary null space is empty)');
else
    disp('Complementary Null Space of the Original Jacobian (columns represent basis):');
    for j = 1:size(null_J_compl, 2)
        disp(['Basis vector ', num2str(j), ':']);
        disp(simplify(null_J_compl(:, j)));
    end
end

%%%%% JACOBIAN TRANSPOSE NULLSPACE %%%%%
% MEANING: When applying in the given direction a Cartesian force to the E-E, 
%          no robot motion will result and this does not need any active 
%          counteracting joint torque by the motors; such a Cartesian force 
%          is Statically Balanced by the Internal Reaction Forces of the rigid 
%          structure.
disp("%%%%%");
disp("A generalised task force f_0, diverse from 0, that is statistically balanced by the joint torque tau=0, when the Robot is in a Regular Configuration.");
nullspace_J_T = simplify(null(J.'));
if isempty(nullspace_J_T)
    disp('Nullspace of the Original Jacobian Transposed: None (The null space is empty)');
else
    disp('Nullspace of the Original Jacobian Transposed (columns represent basis):');
    for j = 1:size(nullspace_J_T, 2)
        disp(['Basis vector ', num2str(j), ':']);
        disp(simplify(nullspace_J_T(:, j)));
    end
end

disp("REMEMBER TO SET THE SOLUTIONS BY HAND HERE FOR SINGULARITIES !!!!!");












%%%%%%%%%% SINGULARITIES' SUBSTITUTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ");
disp("Singularities Study");

% Printing the Determinant
disp("%%%%%");
disp("Determinant(J) is: ");
det_J = simplify(Compute_Jacobian_Determinant(simplify(J)));
disp(det_J);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% SET THE SOLUTIONS BY HAND HERE FOR SINGULARITIES !!!!! %%%%%%%%%%
% Define the solutions for the singularities manually
solutions = [
    struct('q2', 0, 'q3', 0);
    struct('q2', 0, 'q3', +pi);
    struct('q2', 0, 'q3', -pi);
];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ");


% Iterate through each solution and perform Jacobian analysis
for i = 1:length(solutions)
    % Extract the solution
    disp("%%%%%");
    sol = solutions(i);
    disp(['Solution ', num2str(i), ' is: ']);
    disp(sol);
    
    % Compute the variables within the solution
    variables_in_sol = fieldnames(sol);
    disp(['The variables in the solution', num2str(i), ' are:']);
    disp(variables_in_sol);
    
    % Substitute the solution into the Jacobian
    disp("%%%%%");;
    J_singular = subs(J, {q2, q3}, {sol.q2, sol.q3});
    disp(['Jacobian for singularity ', num2str(i), ':']);
    disp(J_singular);
    
    % Calculate and display properties of the singular Jacobian
    disp("%%%%%");
    disp('Properties of the Singular Jacobian:');
    
    % Rank
    disp("%%%%%");
    rank_J_singular = rank(J_singular);
    if rank_J_singular == 0
        disp('Rank of the Singular Jacobian: 0 (The Jacobian is rank deficient)');
    else
        disp(['Rank of the Singular Jacobian: ', num2str(rank_J_singular)]);
    end
    
    % Null Space
    disp("%%%%%");
    disp("The set of all joint velocity vectors s.t. task velocity ṙ=0 when the Robot is in a Singular Configuration");
    null_J_singular = simplify(null(J_singular));
    if isempty(null_J_singular)
        disp('Null Space of the Singular Jacobian: None (The null space is empty)');
    else
        disp('Null Space of the Singular Jacobian (columns represent basis):');
        for j = 1:size(null_J_singular, 2)
            disp(['Basis vector ', num2str(j), ':']);
            disp(null_J_singular(:, j));
        end
    end
    
    % Range Space
    disp("%%%%%");
    disp("The set of all Feasible Directions for the Velocity in a Singular Configuration");
    range_J_singular = simplify(orth(J_singular));
    if isempty(range_J_singular)
        disp('Range of the Singular Jacobian: None (The range space is empty)');
    else
        disp('Range of the Singular Jacobian (columns represent basis):');
        for j = 1:size(range_J_singular, 2)
            disp(['Basis vector ', num2str(j), ':']);
            disp(range_J_singular(:, j));
        end
    end
    
    % Complementary Range Space
    disp("%%%%%");
    disp("The set of all directions along which No Task Velcity can be realized when the Robot is in a Singular Configuration. Space of Lost Mobility");
    range_J_compl_singular = simplify(null(J_singular.'));
    if isempty(range_J_compl_singular)
        disp('Complementary Range Space of the Singular Jacobian: None (The complementary range space is empty)');
    else
        disp('Complementary Range Space of the Singular Jacobian (columns represent basis):');
        for j = 1:size(range_J_compl_singular, 2)
            disp(['Basis vector ', num2str(j), ':']);
            disp(range_J_compl_singular(:, j));
        end
    end
    
    % Complementary Null Space
    disp("%%%%%");
    
    null_J_compl_singular = simplify(colspace(J_singular.'));
    if isempty(null_J_compl_singular)
        disp('Complementary Null Space of the Singular Jacobian: None (The complementary null space is empty)');
    else
        disp('Complementary Null Space of the Singular Jacobian (columns represent basis):');
        for j = 1:size(null_J_compl_singular, 2)
            disp(['Basis vector ', num2str(j), ':']);
            disp(null_J_compl_singular(:, j));
        end
    end
    
    % Jacobian Transpose Nullspace
    disp("%%%%%");
    disp("A generalised task force f_0, diverse from 0, that is statistically balanced by the joint torque tau=0, when the Robot is in a Singular Configuration.");
    nullspace_J_T_singular = simplify(null(J_singular.'));
    if isempty(nullspace_J_T_singular)
        disp('Nullspace of the Singular Jacobian Transposed: None (The null space is empty)');
    else
        disp('Nullspace of the Singular Jacobian Transposed (columns represent basis):');
        for j = 1:size(nullspace_J_T_singular, 2)
            disp(['Basis vector ', num2str(j), ':']);
            disp(nullspace_J_T_singular(:, j));
        end
    end
    
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
end

%%%%%% END PROGRAM %%%%%%