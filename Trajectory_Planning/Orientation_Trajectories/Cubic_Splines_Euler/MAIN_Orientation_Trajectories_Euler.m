% NameFile: MAIN_Orientation_Trajectories_Euler
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 23-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - R1, R...: A set of Rotation Matrices.
%   - T1, T2: A set of Time Intervals.

%   Find: 
%   The Orientation Trajectories employing Cubic Splines.
%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Exam 12-07-2021 N3
% Define the given rotation matrices
R1 = [ 0, -sqrt(2)/2, sqrt(2)/2;
       1,       0,       0;
       0,  sqrt(2)/2, sqrt(2)/2];

Rvia = [ sqrt(6)/4, sqrt(2)/4, -sqrt(2)/2;
        -sqrt(6)/4, -sqrt(2)/4, -sqrt(2)/2;
        -1/2,      sqrt(3)/2,   0];

R2 = [ sqrt(2)/2,  1/2,  -1/2;
       0,       -sqrt(2)/2, -sqrt(2)/2;
      -sqrt(2)/2,  1/2,  -1/2];


% Time intervals
T1 = 2.5; % seconds
T2 = 1; % seconds
T = T1 + T2;

%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%% %

% Convert rotation matrices to ZYX Euler angles
eul1 = rotm2eul(R1, 'ZYX');
eulvia = rotm2eul(Rvia, 'ZYX');
eul2 = rotm2eul(R2, 'ZYX');

alpha1 = eul1(1); beta1 = eul1(2); gamma1 = eul1(3);
alphavia = eulvia(1); betavia = eulvia(2); gammavia = eulvia(3);
alpha2 = eul2(1); beta2 = eul2(2); gamma2 = eul2(3);

% Plot the results
plot_trajectories(alpha1, alphavia, alpha2, beta1, betavia, beta2, gamma1, gammavia, gamma2, T1, T2);

%%%%%% END PROGRAM %%%%%%









