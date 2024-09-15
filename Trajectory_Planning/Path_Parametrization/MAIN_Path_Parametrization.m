% NameFile: MAIN_Path_Parametrization
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
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

% Define input parameters

%%%%%%%%%% Path Parametrization %%%%%%%%%%
% Line
% syms p_f p_i real
% path_parametrization_type = "line";

% Circle
% syms R phi real
% path_parametrization_type = "circle";

% Ellipse
% syms phi a b real 
% path_parametrization_type = "ellipse";

% Helix 
syms r real
path_parametrization_type = "helix";
C = [0; 0; r];
axis = "y";

%%%%%%%%%% Time Parametrization %%%%%%%%%%

% Cubic Rest-to-Rest
syms qin real 
time_parametrization = "cubic_rest_to_rest";



%%%%%% END PARAMETERS %%%%%%

%%%%%% START PROGRAM %%%%%%

clc

%%%%%%%%%% Path Parametrization %%%%%%%%%%
disp("%%%%%%%%%% Path Parametrization %%%%%%%%%%");
if path_parametrization_type == "line"
    disp("Line Parametrization")
    [p, p_prime, p_second, p_prime_norm, p_second_norm] = Path_Parametrization_Line(p_i, p_f);

elseif path_parametrization_type == "helix"
    disp("Helix Parametrization: ");
    [p, p_prime, p_second, p_prime_norm, p_second_norm] = Path_Parametrization_Helix(C, axis);

elseif path_parametrization_type == "circle"
    disp("Circle Parametrization: ");
    [p, p_prime, p_second, p_prime_norm, p_second_norm] = Path_Parametrization_Circle(R, 0);

elseif path_parametrization_type == "ellipse"
    disp("Helix Parametrization: ");
    [p, p_prime, p_second, p_prime_norm, p_second_norm] = Path_Parametrization_Ellipse(a, b, phi);
end

%%%%%%%%%% Time Parametrization %%%%%%%%%%
% disp(" ");
% disp("%%%%%%%%%% Time Parametrization %%%%%%%%%%");
% if time_parametrization == "cubic_rest_to_rest"
%     % s = qin + delta*(3*tau^2 - 2*tau^3);
%     s = delta*(3*tau^2 - 2*tau^3);
%     s_dot = (6*delta)/T * (tau-tau^2);
%     s_double_dot = (6*delta^2)/T * (1-2*tau);
% end
% 
% disp("ṗ(s)= p'(s)* ṡ is equal to: ");
% p_dot = p_prime*s_dot;
% disp(p_dot);
% 
% disp("p̈(s)= p'(s)* s̈ + p''(s)*ṡ²");
% p_double_dot = p_prime*s_double_dot + p_second*s_dot^2;
% disp(p_double_dot);


%%%%%%%%%% Frenet Frames %%%%%%%%%%
% 
disp("%%%%%%%%%% Frenet Frames %%%%%%%%%%");

% Tangential Axis.
t = simplify(p_prime / p_prime_norm);
disp("The Tangential Axis of the Frenet Frame t(s) is:");
disp(t);

% Compute Tangential Axis Derivate and Norm of the Derivate.
t_prime = simplify(diff(t));
disp(" ");
disp("The Derivate of the Tangential Axis of the Frenet Frame t'(s) is:");
disp(t_prime);

t_prime_norm = simplify(norm(t_prime));
disp("The Norm of the Derivate of the Tangential Axis of the Frenet Frame ||t'(s)|| is:");
disp(t_prime_norm);

% Normal Axis.
n = simplify(t_prime / t_prime_norm);
disp("The Normal Axis of the Frenet Frame n(s) is:");
disp(n);

% Binormal Axis.
b = simplify(cross(t, n));
disp("The Bi-Normal Axis of the Frenet Frame b(s) is:");
disp(b);



