% NameFile: MAIN_Find_Joint_Velocity
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - J: The (Geometric) Jacobian.
%   - ee_vel: A Vector containing the E-E Linear and Angular Velocities 
%     (in the Cartesian Space), after having assured that the Jacobian 
%     is Full-Rank.
% 
%   Find: 
%   - Computes the IDK Problem, by returning the joint space velocities. 

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%
syms q1 q2 q3 q4 q5 q6 d1 d2 d3 d4 d5 d6 a1 a2 a3 a4 a5 a6 l l1 l2 l3 l4 l5 l6 real 

% Exam 12-06-2023
J = [       0,       0,       0,    0
 a1 + a3 + a4, a3 + a4,       0,    0;
            0,       0, a3 + a4,   a4;
            0,       0,       0,    0;
            0,       0,      -1,   -1;
            1,       1,       0,    0];
% ee_vel = [0, 3, -3, 0, 0, 1].';
ee_vel = [0, 0, 1, 1, 0, 1].';

% Exam 08-07-2022 N4
% qs = [l*cos(q1) + l*cos(q1+q2) + l*cos(q1+q2+q3);
%       l*sin(q1) + l*sin(q1+q2) + l*sin(q1+q2+q3);
%               q1+q2+q3];
% variables = [q1, q2, q3];
% J = jacobian(qs, variables);
% ee_vel = [1; -1; 0];

%%%%%% END PARAMETERS TO SET %%%%%%

clc

%%%%% ADMISSIBILITY CHECK %%%%%
disp("Given the Jacobian: ");
disp(J);
disp("And given the E-E Velocity: ");
disp(ee_vel);

J_ext = [J, ee_vel];
rank_J = rank(J);
rank_J_ext = rank(J_ext);

% ADMISSIBLE CASE
if rank_J_ext == rank_J

    % (IDK) Inverse Differential Kineamtics
    disp('ee_vel is Admissible');
    joint_vel = simplify(Find_Joint_Velocity_Pseudo_Inverse(J, ee_vel));
    disp("The Joint Velocity (Inverse Differential Kinematics Solution) is: ");
    disp(joint_vel);

    %(DDK) Direct Differential Kineamtics
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    disp("We can now make the Direct Differential Kinematics Check: ");
    disp("Given the Jacobian: ");
    disp(J);
    disp("And given the Joint Velocity: ");
    disp(joint_vel);

    ee_vel_check = J*joint_vel;
    disp("The E-E Velocity (Differential Kinematics Solution) is: ");
    disp(simplify(ee_vel_check));

    % CONSISTENCY CHECK
    if isAlways(simplify(ee_vel - ee_vel_check) == 0)
        disp("The computed E-E Velocity matches the initial E-E Velocity.");
    else
        disp("There is a discrepancy between the computed E-E Velocity and the initial E-E Velocity.");
        error = simplify(ee_vel - ee_vel_check);
        disp("The error is: ");
        disp(error);
    end


% NOT ADMISSIBLE CASE
else
    disp('ee_vel is NOT Admissible');
    joint_vel = simplify(Find_Joint_Velocity_Pseudo_Inverse(J, ee_vel));
    disp("The Joint Velocity (Inverse Differential Kinematics Solution) is: ");
    disp(joint_vel);

    %(DDK) Direct Differential Kineamtics
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    disp("We can now make the Direct Differential Kinematics Check: ");
    disp("Given the Jaacobian: ");
    disp(J);
    disp("And given the Joint Velocity: ");
    disp(joint_vel);

    ee_vel_check = J*joint_vel;
    disp("The E-E Velocity (Differential Kinematics Solution) is: ");
    disp(simplify(ee_vel_check));

    % CONSISTENCY CHECK
    if isAlways(simplify(ee_vel - ee_vel_check) == 0)
        disp("The computed E-E Velocity matches the initial E-E Velocity.");
    else
        disp("There is a discrepancy between the computed E-E Velocity and the initial E-E Velocity.");
        error = simplify(ee_vel - ee_vel_check);
        disp("The error is: ");
        disp(error);
    end
end



