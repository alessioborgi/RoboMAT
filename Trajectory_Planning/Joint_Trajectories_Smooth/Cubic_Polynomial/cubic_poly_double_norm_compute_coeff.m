% NameFile: cubic_poly_double_norm_compute_coeff
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 26-10-2023
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%    - qin: Initial position.
%    - qfin: Final position.
%    - vin: Initial velocity.
%    - vfin: Final velocity.
%    - T: Total time duration.
%    - print_info: Flag to print symbolic and numerical solutions.

%   Find: 
%   - Symbolic & Numerical Solutions for a,b,c,d coefficients

%%%%%% END TASK %%%%%%


function [qn] = cubic_poly_double_norm_compute_coeff(qin_, qfin_, vin_, vfin_, T_, print_info)
    % Define symbolic variables
    syms tau real 
    syms a b c d real
    syms qin qfin real
    syms vin vfin real
    syms Dq real
    syms T real positive
    
    % Define the normalized cubic polynomial and the expression for q(tau).
    qn = a*tau^3 + b*tau^2 + c*tau + d;
    q_tau = qin + Dq*qn;
    
    % Display information if print_info is true.
    if print_info
        % Cubic Polynomial.
        fprintf("Cubic Polynomial Double Normalized (tau belongs to [0, 1])\n");
        display(q_tau);
        fprintf("Where Dq = (qfin - qin)\n");
        
        % First Derivative of the Cubic Polynomial.
        q_tau_prime = diff(q_tau);
        fprintf("The fist derivative of the Cubic Polynomial is: ");
        display(q_tau_prime);

        % Second Derivative of the Cubic Polynomial.
        q_tau_second = diff(q_tau_prime);
        fprintf("The second derivative of the Cubic Polynomial is: ");
        display(q_tau_second);
    end
    
    % Compute coefficients of the cubic polynomial.
    eq_1 = d == 0;
    eq_2 = a + b + c == 1;
    eq_3 = c == vin*T/Dq;
    eq_4 = 3*a + 2*b + c == vfin*T/Dq;

    % Solve the equations symbolically.
    s = solve([eq_1, eq_2, eq_3, eq_4], [a, b, c, d]);
    a = simplify(s.a);
    b = simplify(s.b);
    c = simplify(s.c);
    d = simplify(s.d);
  
    % Display equations and solutions if print_info is true.
    if print_info
        fprintf("---------------------------------------------------------\n");
        fprintf("Equations:\n");
        display(eq_1);
        display(eq_2);
        display(eq_3);
        display(eq_4);

        fprintf("---------------------------------------------------------\n");
        fprintf("Symbolic solutions:\n");
        display(a);
        display(b);
        display(c);
        display(d);
    end
    
    % Assign input values.
    qin = qin_;
    qfin = qfin_;
    
    vin = vin_;
    vfin = vfin_;

    Dq = (qfin - qin);
    T = T_;
    
    % Evaluate coefficients numerically.
    a = subs(a);
    b = subs(b);
    c = subs(c);
    d = subs(d);

    % Display numerical solutions if print_info is true.
    if print_info
        fprintf("---------------------------------------------------------\n");
        fprintf("Numerical solutions:\n");
        display(a)
        display(b)
        display(c)
        display(d)
    end
    
    % Output the symbolic expression for qn.
    qn = subs(qn);
end
