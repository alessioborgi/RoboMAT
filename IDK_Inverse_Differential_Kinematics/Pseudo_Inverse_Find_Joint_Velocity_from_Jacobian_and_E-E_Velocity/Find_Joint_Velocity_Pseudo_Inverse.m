% NameFile: Find_Joint_Velocity_Pseudo_Inverse
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

function joint_vel = Find_Joint_Velocity_Pseudo_Inverse(J, ee_vel)

    % Function that computes the Joint Velocity of a Jacobian, given the
    % E-E Velocity.

    % Compute the Jacobian Size.
    [m, n] = size(J);

    % Check whether we have Full Rank in the Jacobian of the Robot.
    rank_J = rank(J);
    
    % FULL RANK CASE:
    if rank_J == min(m, n)
        disp("I checked that the Jacobian has Full Rank. Thus, this will be the Unique IDK Solution! ");
        disp("We are applying Inverse Differential Kinematics through Pseudo-Inverse Method in Closed-Form.");

        if m == n
            disp("We are in the Simple case (m = n) of a Squared Jacobian.");
            disp("The result will be of: inv(J)*ee_vel ");
            joint_vel = inv(J)*ee_vel;
            
        elseif m < n 
            disp("We are in the Redundant Case (m < n) (more Joints than variables) of a Squared Jacobian.");
            disp("The result will be of: (J.'*inv(J*J.'))*ee_vel ");
            joint_vel = (J.'*inv(J*J.'))*ee_vel;
        
        elseif m > n 
            disp("We are in the case (m > n) where we have more Equations than Unknowns.");
            disp("The result will be of: (inv(J.'*J)*J.')*ee_vel ");
            joint_vel = (inv(J.'*J)*J.')*ee_vel;
        end
    
    % NOT FULL RANK CASE (SVD):
    else
        disp("The Jacobian is not Full Rank. Applying SVD for Pseudo-Inverse Calculation.");
        % Compute the SVD of the Jacobian
        [U, S, V] = svd(J);
        
        % Compute the pseudoinverse of the singular values matrix
        S_pseudo = zeros(size(S'));
        for i = 1:min(m,n)
            if S(i,i) > 1e-6 % Tolerance to avoid division by zero
                S_pseudo(i,i) = 1/S(i,i);
            end
        end
        
        % Compute the pseudoinverse of J using the SVD components
        J_pseudo = V * S_pseudo * U';
        
        % Calculate the joint velocities
        joint_vel = J_pseudo * ee_vel;
    end
end

