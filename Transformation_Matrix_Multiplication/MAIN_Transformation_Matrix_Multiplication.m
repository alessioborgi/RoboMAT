


wTc = [ 1/(2)^(1/2),     0,      -1/(2)^(1/2),       2;
                0,      -1,                 0,       0;
       -1/(2)^(1/2),     0,      -1/(2)^(1/2),       2; 
                0,       0,                 0,       1];

wT0 = [1  0  0  -1; 
       0 -1  0   1;
       0  0 -1 3.5;
       0  0  0   1];


cTe = [1  0  0  0;
       0 -1  0  0;
       0  0 -1  1;
       0  0  0  1];


% wT0_inv = inverseTransformMatrix(wT0);
% disp("Inverse is: ");
% disp(wT0_inv);

disp("Final Transformation is: ");
disp(inv(wT0)*wTc*cTe);


