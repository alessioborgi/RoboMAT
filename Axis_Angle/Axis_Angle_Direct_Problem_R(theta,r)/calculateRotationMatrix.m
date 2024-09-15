% NameFile: calculateRotationMatrix
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 22-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given two data, namely:
%   - theta: An angle.
%   - r: A vector in 3 dimensions ([rx, ry, rz]).
%
%   Compute the Rotation Matrix R(theta, r).

%%%%%% END TASK %%%%%%


function R = calculateRotationMatrix(theta_value, r)

    % Define rx, ry, rz.
    rx = r(1);
    ry = r(2);
    rz = r(3);

    % Define (1-cos(theta_value)).
    new_theta = 1 - cosd(theta_value); % Assuming the input angle is in degrees.
    cs_t = cosd(theta_value);
    si_t = sind(theta_value);

    % Compute the transformation R(θ, r) = C * Rz(θ) * C'
    R = [
        rx^2 * new_theta + cs_t,                rx * ry * new_theta - rz * si_t,         rx * rz * new_theta + ry * si_t;
        rx * ry * new_theta + rz * si_t,        ry^2 * new_theta + cs_t,                 ry * rz * new_theta - rx * si_t;
        rx * rz * new_theta - ry * si_t,        ry * rz * new_theta + rx * si_t,         rz^2 * new_theta + cs_t
    ];

    % Display the result
    disp(" ");
    disp("Given θ: ");
    disp(theta_value);
    disp("and r: ");
    disp(r);
    disp("Transformation Matrix R(θ, r):");
    disp(R);

    % Compute the Trace of R(θ, r).
    tr_R = 1 + 2 * cs_t;
    disp('The Trace of R(θ, r) is: ');
    disp(tr_R);

    % Compute the Eigenvalues of R(θ, r).
    eigenvalues = eig(R);

    % Display the eigenvalues
    disp('The Eigenvalues of the Rotation Matrix R(θ, r) are: ');
    disp(eigenvalues);
    
end



