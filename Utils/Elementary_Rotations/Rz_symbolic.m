function Rz = Rz_symbolic(theta)
    % Rz_symbolic: Generates the symbolic representation of the rotation matrix around the z-axis
    theta = deg2rad(theta);
    Rz = [cos(theta), -sin(theta), 0; 
          sin(theta), cos(theta), 0; 
          0, 0, 1];
end