% NameFile: quintic_poly_compute_coeff
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



function [q_tau] = quintic_poly_compute_coeff(qin_, qfin_, vin_, vfin_, ain_, afin_, T_, print_info)
    % Define symbolic variables
    syms tau real
    syms a b c d e f real
    syms qin qfin real
    syms vin vfin real
    syms ain afin real
    syms T real positive
    % Set digits for numerical computations
    digits(5)

    % Define quintic polynomial q(tau) Normal
    % q_tau = f*tau^5 + e*tau^4 + d*tau^3 + c*tau^2 + b*tau + a;
    q_tau = a*tau^5 + b*tau^4 + c*tau^3 + d*tau^2 + e*tau + f;

    % Define quintic polynomial q(tau) Doubly Normalized
    % q_tau = (1-tau^3)*(qin_ + (3*qin_+vin_*T_)*tau + (ain_*T^2 + 6*vin_*T + 12*qin_)*(tau^2/2)) + tau^3*(qfin_ + (3*qfin_+vfin_*T_)*(1-tau) + (afin_*T^2 + 6*vfin_*T + 12*qfin_)*((1-tau)^2/2));
    
    % Print symbolic solution if print_info is true
    if print_info
        fprintf("##############################################################\n");
        fprintf("Quintic Polynomial (tau belongs to [0, 1])");
        display(q_tau);
    end

    % Compute coefficients by solving equations
    q_tau_0 = simplify(subs(q_tau, {tau}, {0}));
    eq_1 = q_tau_0 == qin;

    q_tau_1 = simplify(subs(q_tau, {tau}, {1}));
    eq_2 = q_tau_1 == qfin;

    q_tau_diff = diff(q_tau, {tau});
    q_tau_diff_0 = subs(q_tau_diff, {tau}, {0});
    eq_3 = q_tau_diff_0 == vin*T;

    q_tau_diff_1 = subs(q_tau_diff, {tau}, {1});
    eq_4 = q_tau_diff_1 == vfin*T;

    q_tau_dd = diff(q_tau_diff, {tau});
    q_tau_dd_0 = subs(q_tau_dd, {tau}, {0});
    eq_5 = q_tau_dd_0 == ain*T^2;

    q_tau_dd_1 = subs(q_tau_dd, {tau}, {1});
    eq_6 = q_tau_dd_1 == afin*T^2;

    % Solve the equations symbolically
    sol = solve([eq_1, eq_2, eq_3, eq_4, eq_5, eq_6], [a,b,c,d,e,f]);
    a = simplify(sol.a);
    b = simplify(sol.b);
    c = simplify(sol.c);
    d = simplify(sol.d);
    e = simplify(sol.e);
    f = simplify(sol.f);

    % Display symbolic solutions
    if print_info
        fprintf("##############################################################\n");
        fprintf("Symbolic solutions:\n");
        display(a);
        display(b);
        display(c);
        display(d);
        display(e);
        display(f);
    end

    % Assign input values
    qin = qin_;
    qfin = qfin_;

    vin = vin_;
    vfin = vfin_;

    ain = ain_;
    afin = afin_;

    T = T_;

    % Evaluate coefficients numerically
    a = double(subs(a));
    b = double(subs(b));
    c = double(subs(c));
    d = double(subs(d));
    e = double(subs(e));
    f = double(subs(f));

    % Print numerical solutions
    if print_info
        fprintf("##############################################################\n");
        fprintf("Numerical solutions:\n");
        display(a);
        display(b);
        display(c);
        display(d);
        display(e);
        display(f);
    end

    % Output the symbolic expression for q_tau
    q_tau = vpa(subs(q_tau));
end
