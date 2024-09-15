% NameFile: Euler_Rotation_Inverse
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - sequence: A string describing the three rotations over fixed 
%               axes we are interested in.
%   - R: The Rotation Matrix.

%   Find: 
%   - phi_value: The angle (in Radians) of the first rotation. It can be either symbolic
%                or normal numbers.
%   - theta_value: The angle (in Radians) of the second rotation. It can be either symbolic
%                or normal numbers.
%   - psi_value: The angle (in Radians) of the third rotation. It can be either symbolic
%                or normal numbers.

% Note that upon execution of the function, you will be required for an
% input which will determine which solution you want from the inverse euler
% problem.
% If you want the positive one, put: "+", "pos", "p" or "positive".
% If you want the negative one, put: "-", "neg", "n" or "negative".

%%%%%% END TASK %%%%%%


function [phi, theta, psi] = Euler_Rotation_Inverse(sequence, R)

    % Euler rotations work about moving-axes
    if isa(R, 'sym')
        disp("R must by non-symbolic matrix")
        phi = -1; theta = -1; psi = -1;
        return
    end
    
    % Check the length of the sequence.
    if strlength(sequence) ~= 3
        disp("The Sequence you are providing is NOT VALID. It must be 3 characters long.")
        return;
    end
    
    % Check whether no two Consecutive Rotations on the same axis.
    sequence = lower(char(sequence));
    if (sequence(2) == sequence(1) || sequence(2) == sequence(3))
        disp("The Sequence you are providing is NOT VALID. You cannot perform Two Consecutive Rotations along the Same Axis (They will sum up simply :) ).")
        return;
    end
    
    % Ask whether we want the Positive or Negative solution.
    risp = input("There is always a pair of solutions to the inverse euler problem. Do you want to use the positive or negative sin(theta)? (pos, neg)\n", "s");
    sign_solution = ((risp == "pos") || (risp == "p") || (risp == "+") || (risp == "positive"));
    
    % Analyse each of the 12 possible requences.
    switch lower(sequence)

        case "xyx"
            theta = atan2(sqrt(R(1, 2)^2 + R(1, 3)^2), R(1, 1))*sign_solution + atan2(-sqrt(R(1, 2)^2 + R(1, 3)^2), R(1, 1))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(1, 2)/sin(theta), R(1, 3)/sin(theta));
            phi = atan2(R(2, 1)/sin(theta), -R(3, 1)/sin(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "xyz"
            theta = atan2(R(1, 3), sqrt(R(1, 1)^2 + R(1, 2)^2))*sign_solution + atan2(R(1, 3), -sqrt(R(1, 1)^2 + R(1, 2)^2))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(-R(1, 2)/cos(theta), R(1, 1)/cos(theta));
            phi = atan2(-R(2, 3)/cos(theta), R(3, 3)/cos(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "xzx"
            theta = atan2(sqrt(R(1, 2)^2 + R(1, 3)^2), R(1, 1))*sign_solution + atan2(-sqrt(R(1, 2)^2 + R(1, 3)^2), R(1, 1))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(1, 3)/sin(theta), -R(1, 2)/sin(theta));
            phi = atan2(R(3, 1)/sin(theta), R(2, 1)/sin(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "xzy"
            theta = atan2(-R(1, 2), sqrt(R(1, 1)^2 + R(1, 3)^2))*sign_solution + atan2(-R(1, 1), -sqrt(R(1, 3)^2 + R(2, 1)^2))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(1, 3)/cos(theta), R(1, 1)/cos(theta));
            phi = atan2(R(3, 2)/cos(theta), R(2, 2)/cos(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "yxy"
            theta = atan2(sqrt(R(2, 3)^2 + R(2, 1)^2), R(2, 2))*sign_solution + atan2(-sqrt(R(2, 3)^2 + R(2, 1)^2), R(2, 2))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(2, 1)/sin(theta), -R(2, 3)/sin(theta));
            phi = atan2(R(1, 2)/sin(theta), R(3, 2)/sin(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "yxz"
            theta = atan2(-R(2, 3), sqrt(R(2, 2)^2 + R(2, 1)^2))*sign_solution + atan2(-R(2, 3), -sqrt(R(2, 2)^2 + R(2, 1)^2))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(2, 1)/cos(theta), R(2, 2)/cos(theta));
            phi = atan2(R(1, 3)/cos(theta), R(3, 3)/cos(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "yzx"
            theta = atan2(R(2, 1), sqrt(R(2, 2)^2 + R(2, 3)^2))*sign_solution + atan2(R(2, 1), -sqrt(R(2, 2)^2 + R(2, 3)^2))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(-R(2, 3)/cos(theta), R(2, 2)/cos(theta));
            phi = atan2(-R(3, 1)/cos(theta), R(1, 1)/cos(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "yzy"
            theta = atan2(sqrt(R(2, 1)^2 + R(2,3)^2), R(2, 2))*sign_solution + atan2(-sqrt(R(2, 1)^2 + R(2,3)^2), R(2, 2))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(2, 3)/sin(theta), R(2, 1)/sin(theta));
            phi = atan2(R(3, 2)/sin(theta), -R(1, 2)/sin(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "zxy"
            theta = atan2(R(3, 2), sqrt(R(3, 1)^2 + R(3,3)^2))*sign_solution + atan2(R(3, 2), -sqrt(R(3, 1)^2 + R(3,3)^2))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(-R(3, 1)/cos(theta), R(3, 3)/cos(theta));
            phi = atan2(-R(1, 2)/cos(theta), R(2, 1)/cos(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "zxz"
            theta = atan2(sqrt(R(1, 3)^2 + R(2, 3)^2), R(3, 3))*sign_solution + atan2(-sqrt(R(1, 3)^2 + R(2, 3)^2), R(3, 3))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(3, 1)/sin(theta), R(3, 2)/sin(theta));
            phi = atan2(R(1, 3)/sin(theta), -R(2, 3)/sin(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "zyx"
            theta = atan2(-R(3, 1), sqrt(R(3, 2)^2+R(3, 3)^2))*sign_solution + atan2(-R(3, 1), -sqrt(R(3, 2)^2+R(3, 3)^2))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(3, 2)/cos(theta), R(3, 3)/cos(theta));
            phi = atan2(R(2, 1)/cos(theta), R(1, 1)/cos(theta));
            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        case "zyz"
            cos_theta = R(3, 3);
            sin_theta = sqrt(R(3, 1)^2 + R(3, 2)^2);
            disp(cos_theta);
            disp(sin_theta);

            theta = atan2(sin_theta, cos_theta)*sign_solution + atan2(-sqrt(R(3, 1)^2 + R(3, 2)^2), R(3, 3))*(1-sign_solution);
            disp("The value for theta is: ");
            disp(theta);

            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            cos_psi = -R(3, 1)/sin(theta);
            sin_psi = R(3, 2)/sin(theta);
            disp("cos(ψ): ");
            disp(cos_psi);
            disp("sin(ψ): ");
            disp(sin_psi);
            psi = atan2(sin_psi, cos_psi);
            
            cos_phi = R(1, 3)/sin(theta);
            sin_phi = R(2, 3)/sin(theta);
            disp("cos(φ): ");
            disp(cos_phi);
            disp("sin(φ): ");
            disp(sin_phi);
            phi = atan2(sin_phi, cos_phi);


            disp("The value for psi is: ");
            disp(psi);
            disp("The value for phi is: ");
            disp(phi);
            
        otherwise
            disp("Invalid sequence")
    end
    
end