% NameFile: MAIN_Quintic_Polynomial
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - qin_: Initial position.
%    - qfin_: Final position.
%    - vin_: Initial velocity.
%    - vfin_: Final velocity.
%    - ain_: Initial Acceleration.
%    - afin_: Final Acceleration.
%    - T_: Total time duration.
%    - print_info: Flag to print symbolic and numerical solutions.

%   Find: 
%   - Symbolic & Numerical Solutions for a,b,c,d,e,f coefficients

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Define input parameters

% Exam 08-07-2022 N4
% Joint 1
qin_ = -pi/4;
qfin_ = 0;
vin_ = 1;
vfin_ = 0;
ain_ = 0;
afin_ = 0;
T_ = 2;
print_info = true;

% Joint 2
% qin_ = pi/4;
% qfin_ = 0;
% vin_ = -1;
% vfin_ = 0;
% ain_ = 0;
% afin_ = 0;
% T_ = 2;
% print_info = true;

% Joint 3
% qin_ = pi/4;
% qfin_ = pi/4;
% vin_ = 0;
% vfin_ = 0;
% ain_ = 0;
% afin_ = 0;
% T_ = 2;
% print_info = true;

%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%

% Call the function
clc
[q_tau] = quintic_poly_compute_coeff(qin_, qfin_, vin_, vfin_, ain_, afin_, T_, print_info)

