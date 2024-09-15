% NameFile: MAIN_Polynomial
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics
% Version: v1.0

%%%%%% TASK %%%%%%
% Given:
% - qin: Initial position for each of the Joints.
% - qfin: Final position for each of the Joints.
% - vin: Initial velocity.
% - vfin: Final velocity.
% - T: Total time duration.
% - Vmax: Set of Velocity Bounds for each of the Joints.
% - Amax: Set of Acceleration Bounds for each of the Joints.
% - poly_type: Choose for the type of Smooth Polynomial you want to use:
%              Cubic,Quintic or Seventic. It must be one of:
%              - 3
%              - 5
%              - 7
% - joints: Set of Joints.

% Find:
% - Symbolic & Numerical Solutions for a,b,c,d,e,f coefficients of the quintic polynomial trajectory.
% - Plot the position, velocity, and acceleration profiles for the given joint trajectories.
% - Check if the velocity and acceleration bounds are satisfied and adjust the time duration accordingly.
%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%
syms t T Vmax Amax q1 q2 q3 q4 q5 q6 q7 real

% Exam 16-02-2024 N3

% Rest-to-Rest Cubic Timing Law
qin = [pi/2; pi];
qfin = [0; 0];
vin = [0; 0];
vfin = [0; 0];
T = 2;
Vmax = [2, 3];
Amax = [0, 0];
deltaq = qfin - qin;

% Change the number of joints accordingly to your problem!
joints = [q1, q2];

%3: Cubic Polynomial
poly_type = "3";



%%%%%% END PARAMETERS %%%%%%

%%%%%% START PROGRAM %%%%%%
clc

% Setting the Polynomial Type.
if poly_type == "3"
    disp("You have opted for implementing a Rest-to-Rest Cubic Polynomial! ");
    poly = qin + deltaq*(-2*(t/T)^3 + 3*(t/T)^2);
elseif poly_type == "5"
    disp("You have opted for implementing a Rest-to-Rest Quintic Polynomial! ");
    poly = qin + deltaq*(10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5);
elseif poly_type == "7"
    disp("You have opted for implementing a Rest-to-Rest Seventic Polynomial! ");
    poly = qin + deltaq*(-20*(t/T)^7 + 70*(t/T)^6 - 84*(t/T)^5 + 35*(t/T)^4);
else
    disp("You inserted a not implemented Polynomial! It must be either 3, 5 or 7!");
end


% Call the Polynomial Function.
disp("You opted for the following Smooth Trajectory Polynomial: ");
disp(poly);
[T_min, T_max] = polynomial(qin, qfin, vin, vfin, T, joints, Vmax, Amax, poly);

% Print the Minimum and Maximum Time Durations.
disp(['Minimum time duration: ', num2str(T_min), ' seconds']);
disp(['Maximum time duration: ', num2str(T_max), ' seconds']);

