function invMatrix = inverseTransformMatrix(T)
    
    % Extract the rotation matrix and translation vector from the input matrix
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    
    disp(R);
    disp(t);
    
    % Compute the inverse rotation matrix
    R_inv = inv(R);
    disp(R_inv);
    
    % Compute the inverse translation vector
    R_inv_minus = -R_inv;
    disp(R_inv_minus);
    t_inv = R_inv_minus * t;
    disp(t_inv);
    
    % Construct the inverse transformation matrix
    invMatrix = [R_inv, t_inv; 0 0 0 1];
end


