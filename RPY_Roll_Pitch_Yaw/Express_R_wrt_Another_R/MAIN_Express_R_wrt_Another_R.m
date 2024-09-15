% NameFile: MAIN_Express_R_wrt_Another_R
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 31-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - R: The rotation matrix representing the final orientation.
%   - alpha_plus, beta_plus: The angles of the first solution from the inverse problem.
%   - alpha_minus, beta_minus: The angles of the second solution from the inverse problem.
%   Find: 
%   - ZXR_plus: The orientation with respect to the rotated frame using {alpha_plus, beta_plus}.
%   - ZXR_minus: The orientation with respect to the rotated frame using {alpha_minus, beta_minus}.

%%%%%% END TASK %%%%%%

clc

%%%%%% PARAMETERS TO SET %%%%%%

% Exam 24-01-2024
% Define the given rotation matrix R.
R = [5*sqrt(2)/8, sqrt(2)/8, -sqrt(3)/4;
     sqrt(2)/4, sqrt(2)/4, sqrt(3)/2;
     sqrt(6)/8, -3*sqrt(6)/8, 1/4];

% Define the angles obtained from the inverse problem.
alpha_plus = pi/4;
beta_plus = -pi/3;
gamma_plus = -pi/3;

alpha_minus = -3*pi/4;
beta_minus = -2*pi/3;
gamma_minus = 2*pi/3;

%%%%%% END PARAMETERS %%%%%%

%%%%%% START PROGRAM %%%%%%

% Rotation matrices for ZX sequence
Rz_alpha_plus = [cos(alpha_plus), -sin(alpha_plus), 0;
                 sin(alpha_plus), cos(alpha_plus), 0;
                 0, 0, 1];

Rx_beta_plus = [1, 0, 0;
                0, cos(beta_plus), -sin(beta_plus);
                0, sin(beta_plus), cos(beta_plus)];

Rz_alpha_minus = [cos(alpha_minus), -sin(alpha_minus), 0;
                  sin(alpha_minus), cos(alpha_minus), 0;
                  0, 0, 1];

Rx_beta_minus = [1, 0, 0;
                 0, cos(beta_minus), -sin(beta_minus);
                 0, sin(beta_minus), cos(beta_minus)];

% Additional rotation matrices for future examples
% Ry_alpha_plus = [cos(alpha_plus), 0, sin(alpha_plus);
%                  0, 1, 0;
%                  -sin(alpha_plus), 0, cos(alpha_plus)];
% 
% Ry_alpha_minus = [cos(alpha_minus), 0, sin(alpha_minus);
%                   0, 1, 0;
%                   -sin(alpha_minus), 0, cos(alpha_minus)];
% 
% Ry_beta_plus = [cos(beta_plus), 0, sin(beta_plus);
%                 0, 1, 0;
%                 -sin(beta_plus), 0, cos(beta_plus)];
% 
% Ry_beta_minus = [cos(beta_minus), 0, sin(beta_minus);
%                  0, 1, 0;
%                  -sin(beta_minus), 0, cos(beta_minus)];

% Compute RZX for both solutions
disp("Starting from R and having the Rz and Rx Rotation Matrices for both Inverse Problem Solutions.");
RZX_plus = Rx_beta_plus * Rz_alpha_plus;
RZX_minus = Rx_beta_minus * Rz_alpha_minus;

% Compute the orientation with respect to the rotated frame
disp("Compute the partial Rotation Matrix and compute the RZX^T * R solutions.");
ZXR_plus = RZX_plus' * R;
ZXR_minus = RZX_minus' * R;

% Display the results
disp('Orientation with respect to the rotated frame using {alpha_plus, beta_plus}:');
disp(ZXR_plus);

disp('Orientation with respect to the rotated frame using {alpha_minus, beta_minus}:');
disp(ZXR_minus);

%%%%%% END PROGRAM %%%%%%