function Ry = Ry_symbolic(theta)
    % Ry_symbolic: Generates the symbolic representation of the rotation matrix around the y-axis
    theta = deg2rad(theta);
    Ry = [cos(theta), 0, sin(theta); 
          0, 1, 0; 
          -sin(theta), 0, cos(theta)];
end

