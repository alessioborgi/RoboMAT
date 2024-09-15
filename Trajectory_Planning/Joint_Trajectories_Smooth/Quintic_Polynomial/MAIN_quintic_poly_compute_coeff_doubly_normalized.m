% NameFile: MAIN_quintic_poly_coeff_doubly_normalized
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 25-05-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - qin: Initial position.
%    - qfin: Final position.
%    - vin: Initial velocity (pdot0).
%    - T: Total time duration.
%    - r: Direct Kinematics.
%    - joint_variables: List of joint variables included.
%    - link_variables: List of link variables included.
%    - link_variables: List of link variable values included.
%   Find: 
%   - Symbolic & Numerical Solutions for a,b,c,d,e,f coefficients

%%%%%% END TASK %%%%%%


%%%%%% PARAMETERS TO SET %%%%%%

syms q1 q2 q3 q4 q5 q6 real 
syms a1 a2 a3 a4 a4 a5 a6 real 
syms d0 d1 d2 d3 d4 de l1 l2 l3 l4 N L M N d A B C D K dtcp h p L1 L2 real 
syms alpha beta gamma t real 

% Define input parameters.
% Exam 08-07-2022 N4

% quintic example with a0 = 0 and a2 = 0 (3 joints).
vin = [1;-1;0];
T = 2;
qfin = [0; 0; pi/4];
qin = [-pi/4; pi/4; pi/4];

r = [L*cos(q1) + N*cos(q1+q2)*cos(q3);
     L*sin(q1) + N*sin(q1+q2)*cos(q3);
     M + N*sin(q3)];

joint_variables = [q1,q2,q3];
link_variables = [L, M, N];
link_variables_value = [0.5, 0.5, 0.5];


%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%
clc
deltaq = qfin-qin;
r = subs(r, link_variables, link_variables_value);
r = jacobian(r, joint_variables);
r = subs(r, joint_variables, qin.');

qdot0 = double(inv(r)*vin);

%change this in order to find the specific joint !
fprintf("Solver \n")
for num = 1:3
    if isequal(deltaq(num),0.0)
        display("No changes for joint q"+num)

        p = qin(num)
        subplot(3,1,1)
        fplot(p, LineWidth=1.5)
        title('Position')
        xlabel('s')
        ylabel('m')
        xlim([0 T])
        grid on
        hold on

        pdot = diff(p)
        subplot(3,1,2)
        fplot(0, LineWidth=1.5)

        title('Velocity')
        xlabel('s')
        ylabel('m/s')
        xlim([0 T])
        grid on
        hold on

        pddot = diff(pdot)
        subplot(3,1,3)
        fplot(0, LineWidth=1.5)

        title('Accelleration')
        xlabel('s')
        ylabel('m/s^2')
        xlim([0 T])
        grid on
        hold on
        break

    end
    
    A1 = vpa( (qdot0(num)*T) / deltaq(num) )
    eq1 = a3 + a4 + a5 == 1 - A1;
    eq2 = 3*a3 + 4*a4 + 5*a5 == -A1;
    eq3 = 3*a3 + 6*a4 + 10*a5 == 0;
    [A, b] = equationsToMatrix([eq1;eq2;eq3], [a3,a4,a5]);
    display("Solution for q"+num)
    sol = inv(A)*b

    p = qin(num) + deltaq(num)*(A1*(t/T) + sol(1)*(t/T)^3 + sol(2)*(t/T)^4 + sol(3)*(t/T)^5);
    subplot(3,1,1)
    fplot(p, LineWidth=1.5)

    title('Position')
    xlabel('s')
    ylabel('m')
    xlim([0 T])
    grid on
    hold on

    pdot = diff(p, t)
    subplot(3,1,2)
    fplot(pdot, LineWidth=1.5)

    title('Velocity')
    xlabel('s')
    ylabel('m/s')
    xlim([0 T])
    grid on
    hold on

    pddot = diff(pdot, t)
    subplot(3,1,3)
    fplot(pddot, LineWidth=1.5)

    title('Accelleration')
    xlabel('s')
    ylabel('m/s^2')
    xlim([0 T])
    grid on
    hold on
end