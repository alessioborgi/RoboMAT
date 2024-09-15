% NameFile: MAIN_Ellipse_Smooth_Parametrization
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 07-06-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - path_parametrization_type: The typology of path.
%    - Additional Parameters depending on the Path Typology.

%   Find: 
%   - p, p_prime, p_second, p_prime_norm, p_second_norm. 

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

% Exam 11-01-2022 N5
syms s a b real
p = [-a*sin(2*pi*s); b*cos(2*pi*s)]
p = subs(p, [a,b], [1, 0.3])

Vmax = 3;
Amax = 6;


%%%%%% END PARAMETERS %%%%%%

clc

%%%%%% START PROGRAM %%%%%%

subplot(3,1,1)
fplot(p, LineWidth=1.5)

title('Position')
xlabel('s')
ylabel('m')
grid on
xlim([0 1])

p_prime = diff(p, s)
subplot(3,1,2)
fplot(p_prime, LineWidth=1.5)

title('Velocity')
xlabel('s')
ylabel('m/s')
grid on
xlim([0 1])

p_pprime = diff(p_prime, s)
subplot(3,1,3)
fplot(p_pprime, LineWidth=1.5)

title('Accelleration')
xlabel('s')
ylabel('m/s^2')
grid on
xlim([0 1])

% Compute s(t), s_dot(t), and s_ddot(t)
s = v * t;
s_dot = v ; % Since s_dot(t) = v
s_ddot = 0; % Since s_ddot(t) = 0
 
% Compute p_dot, p_ddot and their norms.
p_dot = p_prime*s_dot
p_ddot = p_prime*s_ddot + p_pprime*(s_dot^2)
p_dot_norm = norm(p_dot)
p_ddot_norm = norm(p_ddot);


%%% MAXIMUM FEASIBLE SPEED COMPUTATION %%% 
% Calculate the maximum feasible speed vf.
vf = double(subs(1/(2*pi) * min(Vmax/a, sqrt(Amax/a)), [a], [1]))

% Calculate the motion time Tf
Tf = double(1/vf)

% Display the calculated speed and motion time
fprintf('Calculated speed vf = %.4f [s^-1]\n', vf);
fprintf('Calculated motion time Tf = %.4f [s]\n', Tf);






