% NameFile: MAIN_Double_Cameras
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 22-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

% Given data and computations to solve Exercise 1 from 16-02-2024 from the Robotics 1 course.
% This involves computing the position of a point feature P as seen by two cameras
% and solving the inverse kinematics for a PPR planar robot.

clc

% Provided data
L = 0.4; % Distance from the robot base to the cameras [m]
H = 0.8; % Height of the cameras from the base plane [m]
alpha1 = pi/3; % Angle of the first camera [rad]
alpha2 = 2*pi/3; % Angle of the second camera [rad]
f1 = 10; % Focal length of the first camera [mm]
f2 = 12; % Focal length of the second camera [mm]
d1 = 6; % Distance from the optical center of the first camera to the point P [mm]
d2 = -2; % Distance from the optical center of the second camera to the point P [mm]
ell = 0.5; % Length of the third link of the robot [m]

% Convert focal lengths and distances from mm to meters
f1 = f1 / 1000; % Convert to meters
f2 = f2 / 1000; % Convert to meters
d1 = d1 / 1000; % Convert to meters
d2 = d2 / 1000; % Convert to meters

bTc1 = [cos(alpha1), 0, sin(alpha1), L;
        sin(alpha1), 0, -cos(alpha1), H;
        0, 1, 0, 0;
        0, 0, 0, 1];
bTc2 = [cos(alpha2), 0, sin(alpha2), L;
        sin(alpha2), 0, -cos(alpha2), -H;
        0, 1, 0, 0;
        0, 0, 0, 1];

% Perspective relations matrix setup for the cameras
A = [f1 * cos(alpha1 - alpha2) + d1 * sin(alpha1 - alpha2), -f1 * sin(alpha1 - alpha2) + d1 * cos(alpha1 - alpha2);
     f2, d2];

b = [d1 * f1 + 2 * H * (f1 * sin(alpha1) - d1 * cos(alpha1));
     d2 * f2];

% Solve the linear system A * [X2; Z2] = b numerically
X_Z = A \ b;
X2 = double(X_Z(1)); % X coordinate in the second camera frame
Z2 = double(X_Z(2)); % Z coordinate in the second camera frame
c2_P = [X2, Z2];
c2_p_hom = [X2; 0; Z2; 1];

% Compute coordinates of P in the first camera frame using the transformation
X1 = X2 * cos(alpha1 - alpha2) - Z2 * sin(alpha1 - alpha2) - 2 * H * sin(alpha1);
Z1 = X2 * sin(alpha1 - alpha2) + Z2 * cos(alpha1 - alpha2) + 2 * H * cos(alpha1);
c1_P = [X1, Z1];
c1_p_hom = [X1; 0; Z1; 1];


% Display intermediate numerical results for verification
fprintf('Numerical values:\n');
fprintf('Position of P in camera 1 frame: c1_P = [X1, Z1] = [%.4f, %.4f] m\n', c1_P);
fprintf('Position of P in camera 2 frame: c2_P = [X2, Z2] = [%.4f, %.4f] m\n', c2_P);


% Transform coordinates of P to the base frame of the robot
base_P = bTc1*c1_p_hom;

% Display the position of P in the base frame
fprintf('Position of P in the base frame: Xb = %.4f m, Yb = %.4f m\n', base_P);

%%%%%% INVERSE KINEMATICS %%%%%%

% Given desired end-effector position and orientation for the robot
pxd = base_P(1);
pyd = base_P(2);
alpha_d = -2.4553; % Desired orientation [rad]

% Calculate the joint variables for the PPR robot
q1 = pxd - ell * cos(alpha_d); % Joint variable q1
q2 = pyd - ell * sin(alpha_d); % Joint variable q2
q3 = alpha_d; % Joint variable q3

% Display the joint variables
fprintf('Joint variables: q1 = %.4f m, q2 = %.4f m, q3 = %.4f rad\n', q1, q2, q3);

%%%%%% END TASK %%%%%%