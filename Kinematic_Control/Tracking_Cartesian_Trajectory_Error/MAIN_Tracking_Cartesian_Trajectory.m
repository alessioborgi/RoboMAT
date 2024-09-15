% Define the parameters in a separate file
end_effector_pos = [0.5; 0.5];
P1 = [1; 0.5];
P2 = [1; 1.5];
vd = 0.5; % desired constant speed

% Define the limits on joint velocities
Vmax1 = 3; % [rad/s]
Vmax2 = 2; % [rad/s]

% Define proportional gain
kP = 4.2915;

% Define direct kinematics function
forward_kinematics = @(q) [cos(q(1)) + cos(q(1) + q(2)); sin(q(1)) + sin(q(1) + q(2))];

% Inverse kinematics function for the initial position
inverse_kinematics = @(P) [atan2(P(2), P(1)) - acos((P(1)^2 + P(2)^2 - 2) / 2); acos((P(1)^2 + P(2)^2 - 2) / 2)];

% Initial joint angles
q0 = inverse_kinematics(end_effector_pos);

% Save the parameters in a structure
params.end_effector_pos = end_effector_pos;
params.P1 = P1;
params.P2 = P2;
params.vd = vd;
params.Vmax1 = Vmax1;
params.Vmax2 = Vmax2;
params.kP = kP;
params.forward_kinematics = forward_kinematics;
params.q0 = q0;

% Call the tracking function
track_trajectory(params);