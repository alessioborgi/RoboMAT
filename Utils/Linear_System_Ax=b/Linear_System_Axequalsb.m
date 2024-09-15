% Define your coefficient matrix A and right-hand side vector b
syms N q3 px py  real
q3 = sqrt(px^2+py^2-N^2);

A = [N -q3; q3 N];
b = [px; py];


% Solve the linear system Ax = b using linsolve
% x = linsolve(A, b);

% Display the solution
% disp('Solution vector x:');
% disp(simplify(x));

A_inv = inv(A);
x = A_inv*b;
disp(x);