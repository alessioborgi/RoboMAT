% NameFile: Main_Integrated_Jacobian_Analysis
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
%   - singularity_study_flag: A boolean flag true/false for saying whether we
%                        need or not the singularity study.
%   - choice_input_type: This can be either:
%       - 1: (DH-Table (alpha_vector, a_vector, d_vector, theta_vector). 
%              Ex.
%                   DHTABLE = [pi/2            0            sym('d1')           q1;
%                              pi/2            0                   0            q2;
%                              pi/2            0                  q3             0;
%                                 0     sym('a4')                  0            q4];
%       - 2: (Task Vector (r)). 

% If the DH-Table choice is the one chosen, we need to further define:
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


%%%%%%%%%%%%%%%%%%%%%%%%%%
%   - choice_determinant: This indicates the preference on how we would like to compute the Determinant. 
%               In particular, we need to put it either of this choices:
%               - 1: If we would like to compute the Determinant with the Normal Jacobian. 
%               - 2: If we would like to compute the Determinant with the Rotated Jacobian. 
%               - 3: If we would like to compute the Determinant with the Minors Analysis.

% If we select to compute the Determinant with the Rotated Jacobian, I will to also set:  
%   - i: Index of the Rotation Matrix, over which we want to compute the
%        Analysis. Put 0 if you don't want to use the Rotated option.


%%%%%%%%%%%%%%%%%%%%%%%%%%
%   - choice_jacobian: This indicates the type of Jacobian we would like to
%               compute. In particular, this could be:
%               - 1: If we would like to compute the Linear Jacobian.
%               - 2: If we would like to compute the Angular Jacobian.
%               - 3: If we would like to compute the Geometric Jacobian.

% If we selected either the Angular or the Geometric one, we should also set:
%   - prismatic_indices: A list of indices of Prismatic Joints (e.g., [1, 4, 5], corresponding to 
%               the fact that joints 1,4 and 5 are prismatic. Look where the qs is in the 
%               third column (the one referring to d). (Remember to start index from 1)). 
%               If None, put [].

%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Medical Robotics: 2nd Exercise
% syms q1 q2 q3 l1 l2 l3 real
% alpha_vector = [0, 0, 0];
% a_vector = [l1, l2, l3];
% d_vector = [0, 0, 0];
% theta_vector = [q1, q2, q3];

% % Exam 10-06-2022
% alpha_vector = [0, pi/2, 0];
% a_vector = [L, 0, N];
% d_vector = [0, M, 0];
% theta_vector = [q1, q2, q3];

% Exam 21-10-2022
% alpha_vector = [pi/2, pi/2, 0];
% a_vector = [0, 0, a3];
% d_vector = [d1, q2, 0];
% theta_vector = [q1, pi/2, q3];

% Exam 13-02-2023
% alpha_vector = [pi/2, pi/2, -pi/2, 0];
% a_vector = [0, 0, 0, a4];
% d_vector = [0, 0, q3, 0];
% theta_vector = [q1, q2, 0, q4];

% Exam 12-06-2023
% alpha_vector = [0, pi/2, 0, 0];
% a_vector = [a1, 0, a3, a4];
% d_vector = [d1, d2, 0, 0];
% theta_vector = [q1, q2, q3, q4];

% Exam 24-01-2024
% syms q1 q2 q3 q4 q5 q6 a2 a3 d1 d4 d6 real
% alpha_vector = [pi/2, 0, pi/2, -pi/2, pi/2, 0];
% a_vector = [40, 445, 40, 0, 0, 0];
% d_vector = [330, 0, 0, 440, 0, 80];
% theta_vector = [0, 0, 0, 0, 0, 0];

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

% Exam 12-06-2024
% alpha_vector = [pi/2, 0, 0];
% a_vector = [0, a2, a3];
% d_vector = [0, 0, 0];
% theta_vector = [q1, q2, q3];
% DHTABLE = [alpha_vector; a_vector; d_vector; theta_vector].';



%%%%%%%%

% Exam 09-09-2022 N3
% r = [q2*cos(q1)+L*cos(q1+q3);
%      q2*sin(q1)+L*sin(q1+q3);
%             q1 + q3];

% Exam 23-01-2023 N4
% r = [q2*cos(q1)+q4*cos(q1+q3);
%      q2*sin(q1)+q4*sin(q1+q3);
%             q1 + q3];

% Exam 11-09-2023
% r = [a+q1+q3;
%      b+q2+q4];

r = [l1*cos(q1) + l2*cos(q1+q2);
    l1*sin(q1) + l2*sin(q1+q2)];


%%%%%%%%
variables = [q1, q2];
singularity_study_flag = true;

choice_input_type = 2;      % Put - 1: DH Table. or
                            %     - 2: Task Vector.

choice_determinant = 1;     % Put - 1: Normal Jacobian's Det or 
                            %     - 2: Rotated Jacobian's Det or 
                            %     - 3: Minors Analysis's Det.
% Index of Jacobian Rotation Parameter. (Set only if choice_determinant == 2) 
i = 0;  % Put 0 if you don't want to use the Rotated option.

choice_jacobian = 1;        % Put - 1: Linear Jacobian or
                            %     - 2: Angular Jacobian or
                            %     - 3: Geometric Jacobian.
% Prismatic Indices. (Set only if choice_jacobian == 2 or == 3)
prismatic_indices= [];      % Put [] if none.




%%%%%% END PARAMETERS %%%%%%

%%%%%% MAIN %%%%%

clc

%%%%%% START PROGRAM %%%%%%


%%%%% 1) TASK & JACOBIAN COMPUTATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% OPTION 1: %%%%%
%%%%% Denavit-Hartenberg (DH - DK) Direct Kinematics %%%%%

if choice_input_type == 1

    disp('We are opting for using the DH Table input for Jacobian Analysis.');

    % Set the DH Table with columns being: alpha, a, d, theta.
    DH_Table = [alpha_vector; a_vector; d_vector; theta_vector].';
    
    %%% JACOBIAN COMPUTATION
    if choice_jacobian == 1     % Linear Jacobian
        J = DH_to_Linear_Jacobian(DH_Table, variables);
        disp('Linear/Analytical Jacobian: ');
        disp(J);

    elseif choice_jacobian == 2     % Angular Jacobian
        J = DH_to_Angular_Jacobian(DH_Table, prismatic_indices);
        disp('Angular Jacobian:');
        disp(J);
    elseif choice_jacobian == 3     % Geometric Jacobian
        Jr = DH_to_Linear_Jacobian(DH_Table, variables);
        Ja = DH_to_Angular_Jacobian(DH_Table, prismatic_indices);
        J = [Jr; Ja];
        disp('Geometric Jacobian:');
        disp(J);
    else
        disp('choice_jacobian myst either be 1(Analytical) or 2(Angular) or 3 (Geometric) !!!!!');
    end

%%%% OPTION 2: %%%%%
%%%%% Directly pass the DK Task Vector %%%%%
elseif choice_input_type == 2
    
    disp('We are opting for using directly the Task Vector input for Jacobian Analysis.');

    %%% JACOBIAN COMPUTATION
    if choice_jacobian == 1     % Linear Jacobian
        J = simplify(jacobian(r, variables));
        disp("Linear/Analtical Jacobian of Task Vector");
        disp(J);

    elseif computeType == 2     % Angular Jacobian
        J = Task_Vector_to_Angular_Jacobian(variables, prismatic_indices);
        disp('Angular Jacobian of Task Vector:');
        disp(J);
    elseif computeType == 3     % Geometric Jacobian
        Jr = simplify(jacobian(r, variables));
        Ja = Task_Vector_to_Angular_Jacobian(variables, prismatic_indices);
        J = [Jr; Ja];
        disp('Geometric Jacobian of Task Vector:');
        disp(J);
    else
        disp('choice_jacobian myst either be 1(Analytical) or 2(Angular) or 3 (Geometric) !!!!!');
    end
    

% Wrong. We put choice_input_type diverse from either 1 or 2. 
else
    disp('choice_input_type myst either be 1(DH-Table) or 2(task vector) !!!!!');
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%% JACOBIAN ANALYSIS & MOBILITY ANALYSIS %%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

%%%%%%%%%% 1) "SIMPLE" DETERMINANT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% Printing the Determinant
if choice_determinant == 1  
    
    % Compute the Determinant.
    [m, n] = size(J);
    det_J = Compute_Jacobian_Determinant(J);
    disp("The Determinant of the (Simple) Geometric Jacobian is given by: ");
    disp(simplify(det_J));


%%%%% 2) MIDDLE: ROTATED JACOBIAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif choice_determinant==2 && i~=0 
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
    rotated_Geometric_Jac = simplify(extended_R*J);
    fprintf("The Rotated Geometric Jacobian %dJ(q) is equal to: \n", i);
    J = rotated_Geometric_Jac;
    
    % Find the Determinant of the Rotated Jacobian i_J(q).
    [m, n] = size(J);
    disp(simplify(J));
    det_J = Compute_Jacobian_Determinant(J);
    disp("The Determinant of the Rotated Geometric Jacobian is given by: ");
    disp(simplify(det_J));


%%%%% 3) DIFFICULT: MINORS ANALYSIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif choice_determinant==3
    disp("Starting the Minors Analysis... ");
    solutions = Find_Singularity_Minors_Analysis(J, variables);
    disp("In order to find the Singulariteis, pose every subDeterminant == 0");
    disp(solutions);
    singularities = solve(solutions, variables, 'Real', true);
    disp("The Singularities we found with the Minors Analysis are: ");
    disp(singularities);    
   
else 
    disp("You typed a Wrong Choice. Choice must be either 1, 2 or 3.");

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%% ANALYSIS over the ORIGINAL JACOBIAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

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

















%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% SINGULARITIES' SUBSTITUTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if singularity_study_flag == true
    
    disp(" ");
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ");
    disp("Singularities Study");

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% SET THE SOLUTIONS BY HAND HERE FOR SINGULARITIES !!!!! %%%%%%%%%%
    % Define the solutions for the singularities manually
        
    % solutions = [
    %     struct('q2', 0, 'q3', 0);
    %     struct('q2', 0, 'q3', +pi);
    %     struct('q2', 0, 'q3', -pi);
    % ];

    % Exam 13-02-2023
    % solutions = [
    %     struct('q3', 0);
    %     ];

    % Exam 21-10-2022
    % solutions = [
    %     struct('q3', 0);
    %     struct('q3', pi);
    %     struct('q2', -(a3*sin(q3)));
    %     ];

    % Exam 09-09-2022 N3
    % solutions = [
    %      struct('q2', 0);
    %      ];


    % Exam 10-06-2022
    % solutions = [
            % struct('q2', 0);
            % struct('q2', pi);
            % struct('q3', pi/2);
            % struct('q2', pi/2, 'q3', pi/2);
            % ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Exam 12-06-2024
    solutions = [
            struct('q2', 0, 'q3', 0);
            ];
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
        
        % Prepare the variables and their corresponding values for substitution
        vars = cell(1, length(variables_in_sol));
        vals = cell(1, length(variables_in_sol));
        
        for j = 1:length(variables_in_sol)
            vars{j} = sym(variables_in_sol{j});
            vals{j} = sol.(variables_in_sol{j});
        end
        
        % Substitute the solution into the Jacobian
        disp("%%%%%");
        %J_singular = subs(J, {q2, q3}, {sol.q2, sol.q3});
        J_singular = subs(J, vars, vals);
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
    
end
%%%%%% END PROGRAM %%%%%%





